"""3D view of the real CAD hand, driven by the URDF exported from Fusion.

Loads the per-link STL meshes referenced by the URDF (see urdf_model.py),
places each with forward kinematics, and moves them by transforming the
mesh vertices directly each frame. Each firmware joint drives its own URDF
joint, so the view is a near-identity map of the hand's real geometry.

Why vertices are transformed directly rather than via the actor's
user/transform matrix: an earlier version set each actor's VTK UserMatrix,
and on a real machine the links visibly *dislocated* (the transform was
composing with something else in the actor), even though the underlying
math is an exact rigid rotation (verified vertex-by-vertex: every point
keeps its distance to the joint pivot). Baking `world[link] @ visual_origin`
straight into a fresh copy of the rest vertices sidesteps all actor-
transform ambiguity -- the points that get drawn are exactly the points FK
computes. The rest vertices are kept immutable and every update recomputes
from them, so nothing compounds.

Not renderable in the build sandbox (no display/GPU); the kinematics are
unit-tested in urdf_model.py and the vertex transform is checked against
the real STLs; the view itself is verified by running urdf_finger_test.py.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor
from PySide6.QtWidgets import QVBoxLayout, QWidget

from .hrh1_urdf_map import (  # noqa: F401  (re-exported for existing callers)
    HRH1_URDF_NAMES as _HRH1_URDF_NAMES,
    HRH1_URDF_SIGNS as _HRH1_URDF_SIGNS,
    hrh1_assembly_xacro,
    hrh1_firmware_map,
)
from .urdf_model import URDFModel, read_stl_facet_colors, read_stl_vertices

_BACKGROUND = "#20242c"
_PART_COLOR = "#c9ccd1"     # brushed-metal grey for the printed/CAD parts
_BEARING_COLOR = "#8a8f98"  # slightly darker for the bearings

# Performance: the raw CAD export is ~1.5M triangles (the bearings alone are
# ~1.1M and are barely visible), which is far too heavy to re-pose in real
# time. So by default we (a) don't render the bearings at all, and (b)
# decimate any remaining part heavier than this face target. Together this
# takes the view from >1M triangles to a few tens of thousands.
_BEARING_MARKERS = ("623ZZ", "MR93ZZ", "5x16x5", "Bearing", "bearing")
_DECIMATE_TARGET_FACES = 2500

# Cosmetic parts added to the CAD after the kinematics were settled: the 18
# servos on the rack, and the wrist joint covers. They carry no kinematics --
# every one hangs off a FIXED joint -- and the servos in particular are bulky
# (~20k triangles each) and mostly hidden inside the rack.
#
# They cost nothing per frame: they are all children of Servo_Rack, so they
# only re-transform when W_1 moves, and _apply skips unchanged meshes. What
# they do cost is load time, since each one still has to be read and
# decimated. Rendering them takes the scene from ~55k to ~134k triangles.
# Set show_decor=False to get the load time back.
_DECOR_MARKERS = ("Servo+", "Cover")


def _is_bearing(link_name: str) -> bool:
    return any(k in link_name for k in _BEARING_MARKERS)


def _is_decor(link_name: str) -> bool:
    return any(k in link_name for k in _DECOR_MARKERS)


# Colours come from the STLs, not the URDF: fusion2urdf gives every link the
# same "silver" material, but Fusion writes the CAD appearance into each
# facet's attribute field (see read_stl_facet_colors). So the view follows
# whatever the parts look like in CAD, with no name-matching table to keep in
# sync -- recolour a part in Fusion, re-export, and it shows up.
#
# The one liberty taken: very dark parts are lifted to a floor. The MG996R
# servo tops and the thumb grip come out around #181818, which is darker than
# this view's own background (#20242c), so faithful rendering would sink them
# into it. A real matte-black part under studio lighting reads lighter than
# its base colour anyway.
_MIN_LUMA = 0.14
#: facets below this share of a mesh are folded into its dominant colour
#: rather than becoming their own actor -- not worth a draw call
_MIN_COLOR_SHARE = 0.02


@dataclass
class _Part:
    """One drawable piece of geometry, rigidly attached to `link`.

    Holds both the immutable rest geometry (points AND normals) and live
    views straight into the VTK arrays the mapper draws from, so a pose
    update is two matrix multiplies written in place -- no allocation, and
    no pipeline stage in between. See _add_part for why that matters.
    """
    link: str
    mesh: "pv.PolyData"
    rest_pts: np.ndarray       # immutable rest vertices (N, 3)
    rest_nrm: np.ndarray       # immutable rest point normals (N, 3)
    visual_origin: np.ndarray  # the link's visual origin transform (4, 4)
    pts: np.ndarray = None     # live view of the drawn points (VTK memory)
    nrm: np.ndarray = None     # live view of the drawn normals (VTK memory)

    @property
    def key(self) -> str:
        return f"{self.link}#{id(self)}"


def _with_normals(mesh: "pv.PolyData") -> "pv.PolyData":
    """Point normals computed exactly as pyvista's smooth shading does.

    Same call and same arguments as pyvista's own SmoothShadingAlgorithm
    (`cell_normals=False, split_vertices=False`), so the surface shades
    identically -- the only difference is that this runs ONCE, at load,
    instead of on every frame.
    """
    return mesh.compute_normals(
        cell_normals=False, point_normals=True, split_vertices=False,
    )


def _lift(rgb) -> tuple:
    """Raise a colour to _MIN_LUMA if it is darker, keeping its hue."""
    r, g, b = (float(v) for v in rgb)
    luma = 0.2126 * r + 0.7152 * g + 0.0722 * b
    if luma >= _MIN_LUMA:
        return (r, g, b)
    if luma < 1e-6:
        return (_MIN_LUMA,) * 3
    k = _MIN_LUMA / luma
    return tuple(min(1.0, v * k) for v in (r, g, b))


def _polydata(verts: np.ndarray) -> "pv.PolyData":
    """PolyData from a flat (3F, 3) triangle-soup vertex array."""
    n = len(verts) // 3
    faces = np.c_[np.full(n, 3), np.arange(3 * n).reshape(n, 3)].ravel()
    return pv.PolyData(verts, faces).clean()  # clean() merges duplicate points


def _load_colored_meshes(mesh_path, scale, use_cad_colors: bool):
    """[(mesh, color), ...] for one STL -- several when the part is multi-coloured.

    Single-colour parts (everything that moves) take the fast path: pyvista's
    own reader, exactly as before. Only multi-coloured parts are rebuilt from
    the raw triangle soup so they can be split, and those are all servos,
    which never move during hand tracking.
    """
    colors = read_stl_facet_colors(mesh_path) if use_cad_colors else None
    if colors is not None:
        packed = (np.round(colors * 31).astype(np.uint32)
                  @ np.array([1, 32, 1024], dtype=np.uint32))
        uniq, counts = np.unique(packed, return_counts=True)
        keep = uniq[counts >= _MIN_COLOR_SHARE * len(packed)]
        if len(keep) > 1:
            verts = read_stl_vertices(mesh_path) * np.asarray(scale, float)
            tri = verts.reshape(-1, 3, 3)
            out = []
            dominant = uniq[np.argmax(counts)]
            others = keep[keep != dominant]
            for code in keep:
                # The dominant colour also absorbs any facets whose own colour
                # was too rare to earn its own actor, so no triangle is lost.
                sel = ~np.isin(packed, others) if code == dominant else (packed == code)
                share = float(sel.sum()) / len(packed)
                sub = _polydata(tri[sel].reshape(-1, 3))
                rgb = colors[int(np.argmax(sel))]
                out.append((_decimate(sub, _DECIMATE_TARGET_FACES * share), _lift(rgb)))
            return out

    mesh = pv.read(str(mesh_path))
    mesh.points = mesh.points * scale        # mm -> m (scale is 0.001)
    color = _lift(colors[0]) if colors is not None else _PART_COLOR
    return [(_decimate(mesh), color)]


def _decimate(mesh: "pv.PolyData", target: Optional[int] = None) -> "pv.PolyData":
    """Reduce a mesh toward `target` faces (default _DECIMATE_TARGET_FACES).

    `target` is explicit so a part that gets split by colour can divide ONE
    budget between its pieces rather than spending the full budget on each --
    otherwise splitting the 18 servos three ways would have tripled their
    triangle count instead of leaving it unchanged.
    """
    budget = _DECIMATE_TARGET_FACES if target is None else max(int(target), 200)
    try:
        n = mesh.n_cells  # STL -> all triangles, so cells == faces
        if n > budget:
            return mesh.triangulate().decimate(1.0 - budget / n)
    except Exception:
        pass
    return mesh

# The firmware<->URDF joint name/sign maps and the URDF path now live in
# hrh1_urdf_map.py (imported above), so the retargeting optimiser -- which
# runs on the vision thread and must not import pyvista -- can share them.
HRH1_URDF_JOINT_MAP = hrh1_firmware_map()


class URDFHandView(QWidget):
    """Renders a URDFModel and poses it from joint angles.

    Two ways to drive it:
      * set_joint_angles({urdf_joint_name: degrees}) -- direct, used by the
        test sliders.
      * set_pose(angles_deg, joint_map) -- from a firmware angle array via a
        {firmware_index: (urdf_joint_name, sign)} map, for app use.
    """

    def __init__(self, xacro_path, mesh_dir=None, show_joints: bool = False,
                 firmware_map: Optional[Dict[int, tuple]] = None,
                 show_bearings: bool = False, show_decor: bool = True,
                 use_cad_colors: bool = True, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.plotter = QtInteractor(self)
        layout.addWidget(self.plotter)
        self.plotter.set_background(_BACKGROUND)
        self.plotter.enable_lightkit()
        self.plotter.enable_trackball_style()  # left-drag orbit, scroll zoom

        self._show_joints = show_joints
        # firmware index -> (urdf joint name, sign); used by set_pose so the
        # app can drive this like the other hand views. Empty for the
        # generic test viewer (which drives urdf joints directly).
        self._firmware_map = firmware_map or {}
        self.model = URDFModel.load(xacro_path, mesh_dir=mesh_dir)

        # Renderable geometry, as a flat list of _Part. One link usually gives
        # one part, but a multi-coloured mesh (the servos) is split into one
        # part per colour so each can be drawn in its own solid colour -- that
        # is simpler and far more robust than carrying per-facet scalars
        # through decimation, which does not reliably survive it.
        self._parts: List[_Part] = []
        self._last_m: Dict[str, bytes] = {}  # per-link last transform, for change-skip
        for name, link in self.model.links.items():
            vis = link.visual
            if vis is None or vis.mesh_path is None or not vis.mesh_path.exists():
                continue
            if not show_bearings and _is_bearing(name):
                continue  # skip the (heavy, near-invisible) bearing meshes
            if not show_decor and _is_decor(name):
                continue  # skip the servos / wrist covers
            for mesh, color in _load_colored_meshes(vis.mesh_path, vis.scale, use_cad_colors):
                self._add_part(name, mesh, color, vis.origin)

        # Debug joint markers: a bright sphere at each movable joint's pivot
        # and a line along its rotation axis, so the hinge point/axis is
        # visible. sphere/line vertices are updated the same direct way as
        # the link meshes. _sphere_base is the sphere centred at the origin.
        self._joint_spheres: Dict[str, pv.PolyData] = {}
        self._joint_axes: Dict[str, pv.PolyData] = {}
        self._sphere_base: Dict[str, np.ndarray] = {}
        if self._show_joints:
            for jname in self.model.movable_joint_names:
                sph = pv.Sphere(radius=0.004)
                self._sphere_base[jname] = np.asarray(sph.points, dtype=float)
                self._joint_spheres[jname] = sph
                self.plotter.add_mesh(sph, color="#ff5555", render=False, reset_camera=False)
                line = pv.Line((0, 0, 0), (0.03, 0, 0))
                self._joint_axes[jname] = line
                self.plotter.add_mesh(line, color="#ffe14d", line_width=4, render=False, reset_camera=False)

        self._last_angles: Optional[Dict[str, float]] = None
        self._apply(self.model.link_world_transforms({}))

        # CAD-style interactive orientation gizmo in the corner: click a face
        # or axis to snap the camera to it, exactly like a view cube. This is
        # VTK's own vtkCameraOrientationWidget, surfaced by pyvista -- no
        # need to hand-draw one. Guarded because it is a relatively recent
        # pyvista addition and a missing gizmo must never cost us the view.
        try:
            self.plotter.add_camera_orientation_widget()
        except Exception:  # noqa: BLE001 - older pyvista, or no widget support
            try:
                self.plotter.add_axes()  # plain orientation marker instead
            except Exception:
                pass

        self.set_named_view("iso")

    def _add_part(self, link: str, mesh: "pv.PolyData", color, visual_origin) -> None:
        """Add one mesh to the scene with NO pipeline stage in front of it.

        This is the performance-critical decision in this file. Adding a mesh
        with pyvista's `smooth_shading=True` inserts a SmoothShadingAlgorithm
        -- a *Python-implemented* VTK filter that recomputes point normals
        whenever its input changes. This view moves the hand by rewriting
        vertices, so every frame dirtied the meshes and the filter re-ran:
        measured at 90 executions and ~285 ms per frame, 60% of the frame,
        holding the GIL the whole time. That is why hand tracking in Target
        mode ran at ~6 fps, and why the *status poll* timer slowed in lockstep
        with the 3D view -- the whole GUI thread was starved, and the vision
        thread's solver slowed too (7 ms in isolation, 15 ms in the app).

        But a rigid motion does not change a surface's shape, so its normals
        never need *recomputing*; they only rotate with it. So normals are
        computed once here (identically to pyvista's call), then rotated each
        frame by the same 3x3 that moves the vertices. pyvista still builds
        the actor -- keeping its colour, lighting and Phong material exactly
        as before -- but the mapper is then pointed straight at our mesh,
        bypassing the Python stage entirely.
        """
        try:
            mesh = _with_normals(mesh)
            fast = True
        except Exception:  # noqa: BLE001 - e.g. a part with no polygons
            # Can't precompute normals for this one, so let pyvista's own
            # pipeline handle it as before. Slower, but only for this part,
            # and a part that shades slowly beats a view that fails to load.
            fast = False
        actor = self.plotter.add_mesh(
            mesh, color=color, smooth_shading=True, render=False, reset_camera=False
        )
        if fast:
            # Cut pyvista's per-frame normals stage out of the pipeline. The
            # actor keeps its smooth-shading property (Phong); the normals it
            # needs are already on `mesh`, and _apply keeps them current.
            actor.GetMapper().SetInputData(mesh)
            actor.GetProperty().SetInterpolationToPhong()

        part = _Part(
            link=link,
            mesh=mesh,
            rest_pts=np.array(mesh.points, dtype=np.float64),
            rest_nrm=np.array(mesh.point_normals, dtype=np.float64) if fast else None,
            visual_origin=visual_origin,
        )
        # Views directly into the VTK arrays the mapper draws from. Writing
        # through these updates the rendered geometry in place; the arrays
        # are never reallocated after this point, so the views stay valid.
        part.pts = mesh.points
        part.nrm = mesh.point_data["Normals"] if fast else None
        self._parts.append(part)

    # -- camera -------------------------------------------------------------

    def set_named_view(self, key: str) -> None:
        """Snap the camera to a named view, then frame the hand.

        The names are anatomical rather than axis names ("front" = looking
        at the palm) because that is how you think about a hand, and the
        mapping between the two is a property of how the assembly happens
        to sit in the CAD's world frame -- which has already changed once,
        when a re-export rotated everything 180 deg about Z. Keeping the
        mapping in one table here means a future re-orientation is a one-
        line fix rather than a hunt through the GUI.

        The assembly stands up +Z with the fingertips highest, the palm and
        back of the hand face along -/+Y, and the thumb and pinky sit at
        -/+X (measured from the CAD, see ik_retarget's palm frame).

        Which of those axes gets called "front" or "left" is a *naming*
        choice, not a geometric one, and the pairs below are deliberately
        the opposite way round from the obvious reading of the axes: they
        were settled by looking at the real render. For a right hand,
        viewing from the thumb side is viewing the hand from its own right,
        so that is "Right" even though the thumb sits at -X; and "Front"
        shows the back of the hand, matching the CAD convention rather than
        the anatomical one. Flip a pair here if it ever reads wrong -- the
        geometry underneath is unaffected either way.
        """
        p = self.plotter
        try:
            if key == "fit":
                p.reset_camera()
                p.render()
                return
            view = {
                "front": lambda: p.view_xz(negative=True),  # back of the hand
                "back": lambda: p.view_xz(),                # palm toward camera
                "left": lambda: p.view_yz(),                # pinky side
                "right": lambda: p.view_yz(negative=True),  # thumb side
                "top": lambda: p.view_xy(),                 # down onto the fingertips
                "iso": lambda: p.view_isometric(),
            }.get(key)
            if view is None:
                return
            view()
            p.reset_camera()      # keep the whole (tall) assembly in frame
            p.render()
        except Exception:  # noqa: BLE001 - a camera preset is never worth a crash
            pass

    # -- posing -------------------------------------------------------------

    def _apply(self, world: Dict[str, np.ndarray]) -> None:
        changed = False
        for part in self._parts:
            T = world.get(part.link)
            if T is None:
                continue
            M = T @ part.visual_origin
            # Only re-transform a part whose world transform actually changed.
            # Moving one joint changes just that joint's subtree, so most
            # parts are skipped every frame -- which is what keeps the 18
            # servos (all welded to Servo_Rack, so static unless W_1 moves)
            # free during hand tracking.
            key = M.tobytes()
            if self._last_m.get(part.key) == key:
                continue
            self._last_m[part.key] = key
            # Rigid motion: vertices get R and t, normals get R alone (for an
            # orthonormal R the inverse-transpose normals need is R itself).
            # Written in place into the arrays the mapper draws from, then
            # flagged modified so VTK re-uploads just this part.
            R = M[:3, :3]
            part.pts[:] = part.rest_pts @ R.T + M[:3, 3]
            part.mesh.GetPoints().Modified()
            if part.nrm is not None:  # None only for a fallback part, see _add_part
                part.nrm[:] = part.rest_nrm @ R.T
                part.mesh.GetPointData().GetNormals().Modified()
            changed = True
        if self._show_joints:
            self._update_joint_markers(world)
            changed = True
        if changed:
            self.plotter.render()

    def _update_joint_markers(self, world: Dict[str, np.ndarray]) -> None:
        for joint in self.model.joints:
            if joint.name not in self._joint_spheres:
                continue
            Tp = world.get(joint.parent)
            if Tp is None:
                continue
            Tj = Tp @ joint.origin           # joint frame in world
            pivot = Tj[:3, 3]
            axis = np.asarray(joint.axis, float)
            n = np.linalg.norm(axis)
            axis_w = Tj[:3, :3] @ (axis / n) if n > 1e-12 else np.array([1.0, 0, 0])
            self._joint_spheres[joint.name].points = self._sphere_base[joint.name] + pivot
            self._joint_axes[joint.name].points = np.array(
                [pivot - axis_w * 0.03, pivot + axis_w * 0.03]
            )

    def set_joint_angles(self, angles_deg: Dict[str, float]) -> None:
        """angles_deg: {urdf_joint_name: degrees}. Missing joints -> 0."""
        if angles_deg == self._last_angles:
            return  # unchanged -> skip, keep orbiting smooth
        self._last_angles = dict(angles_deg)
        radians = {k: np.radians(v) for k, v in angles_deg.items()}
        self._apply(self.model.link_world_transforms(radians))

    def set_pose(self, angles_deg) -> None:
        """Drive from a firmware angle array (joints.py order) via the view's
        firmware->URDF map. Same call shape as the GUI's placeholder view,
        so gui.py can use any of the three interchangeably. A per-joint sign
        in the map flips direction to match the robot without re-exporting."""
        urdf_angles: Dict[str, float] = {}
        for idx, (jname, sign) in self._firmware_map.items():
            if idx < len(angles_deg):
                urdf_angles[jname] = sign * float(angles_deg[idx])
        self.set_joint_angles(urdf_angles)
