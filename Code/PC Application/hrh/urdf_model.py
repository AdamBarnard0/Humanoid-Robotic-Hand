"""Minimal URDF / xacro loader + forward kinematics for the HRH1 hand.

Reads the URDF that fusion2urdf exports from the Fusion CAD (see the
"v2 migration" / URDF notes in README). fusion2urdf writes the robot
description as a `.xacro` file, but that file is plain URDF XML apart from
three `<xacro:include>` lines at the top (materials / transmissions /
gazebo) -- none of which carry geometry or kinematics. So this parser just
reads the `<link>` and `<joint>` elements directly with the stdlib XML
parser and ignores everything xacro-namespaced; no ROS, no xacro tool, no
Isaac Sim / USD conversion needed (a common point of confusion -- Isaac
Sim uses USD, which is a different path entirely).

Deliberately dependency-light: xml.etree + numpy only, and NO rendering in
here, so the whole thing is unit-testable without a display/GPU (this
sandbox has neither). The PyVista view (hand_view3d_urdf.py) is a thin
layer on top that just draws what this computes.

Units: fusion2urdf meshes are in millimetres and each `<mesh>` carries a
`scale` of 0.001 to bring them to metres; link/joint origins are already
in metres. This module keeps everything in metres and hands the mesh scale
through so the view can apply it.

Coordinate/axis note: joint angles here are whatever the URDF axis says
(right-hand rule about that axis). Whether a positive firmware angle curls
the finger the intended way is a per-joint sign that gets settled against a
render, exactly like the MANO retarget's sign constants -- so the mapping
from firmware joints to URDF joints (see hrh1_urdf_map.py / the view) keeps
per-joint sign flips rather than baking them in here.
"""
from __future__ import annotations

import re
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


#: Fusion bakes a component's version into exported names ("Palm2_v1_1",
#: "I_DI_v1_1"). It carries no kinematic meaning, and it changes whenever the
#: CAD part is edited, so names are compared with it removed.
_VERSION_SUFFIX = re.compile(r"_v\d+(?=_|$)")


def _strip_version(name: str) -> str:
    return _VERSION_SUFFIX.sub("", name)


def read_stl_facet_colors(path):
    """Per-facet colours baked into a binary STL, or None if there are none.

    Binary STL has a 2-byte "attribute byte count" per facet which CAD tools
    hijack to store colour, 5 bits per channel (the Magics / SolidWorks
    convention). Fusion writes it, so the appearance set in CAD survives the
    export -- which the URDF itself does not carry: fusion2urdf gives every
    link the same "silver" material regardless of how it looks in Fusion.

    Returns (N, 3) float RGB in 0..1, one row per facet. A file with the
    field left at zero (genuinely no colour) returns None so callers can fall
    back to a default rather than render everything black.
    """
    data = Path(path).read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:512]:
        return None  # ASCII STL carries no colour in this convention
    count = struct.unpack("<I", data[80:84])[0]
    if count == 0:
        return None
    raw = np.frombuffer(data[84:84 + 50 * count], dtype=np.uint8).reshape(count, 50)
    attr = raw[:, 48:50].copy().view("<u2").ravel().astype(np.uint32)
    if not np.any(attr):
        return None
    rgb = np.stack([attr & 31, (attr >> 5) & 31, (attr >> 10) & 31], axis=1)
    return rgb.astype(float) / 31.0


def read_stl_vertices(path) -> np.ndarray:
    """(N, 3) float array of every triangle vertex in an STL, in the file's
    own units (millimetres for the fusion2urdf exports).

    Written by hand rather than pulled from pyvista/trimesh so this module
    stays numpy-only and importable with no display/GPU -- the retargeter
    needs mesh geometry (to locate fingertips) but must run headless on the
    vision thread. Handles both binary STL (what Fusion writes: 80-byte
    header, uint32 triangle count, then 50 bytes per facet = 12 floats +
    2 pad bytes) and ASCII STL.
    """
    data = Path(path).read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:512]:
        verts = []
        for line in data.decode("ascii", "ignore").splitlines():
            parts = line.split()
            if len(parts) >= 4 and parts[0] == "vertex":
                verts.append([float(v) for v in parts[1:4]])
        return np.asarray(verts, dtype=float).reshape(-1, 3)
    count = struct.unpack("<I", data[80:84])[0]
    raw = np.frombuffer(data[84:84 + 50 * count], dtype=np.uint8).reshape(count, 50)
    # bytes 0-11 are the facet normal, 12-47 the three vertices, 48-49 pad.
    tri = raw[:, 12:48].copy().view("<f4").reshape(count, 3, 3)
    return tri.reshape(-1, 3).astype(float)


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis rpy (X then Y then Z, i.e. R = Rz @ Ry @ Rx)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _make_transform(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _rpy_to_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def _axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """4x4 rotation of `angle` radians about `axis` (Rodrigues)."""
    a = np.asarray(axis, dtype=float)
    n = np.linalg.norm(a)
    T = np.eye(4)
    if n < 1e-12 or abs(angle) < 1e-15:
        return T
    a = a / n
    x, y, z = a
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    T[:3, :3] = np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)
    return T


def _floats(text: Optional[str], default) -> np.ndarray:
    if not text:
        return np.array(default, dtype=float)
    return np.array([float(v) for v in text.split()], dtype=float)


@dataclass
class Visual:
    mesh_path: Optional[Path]  # resolved absolute path to the STL, or None
    origin: np.ndarray         # 4x4, mesh frame relative to the link frame
    scale: np.ndarray          # (3,) mesh scale (mm->m = 0.001)


@dataclass
class Link:
    name: str
    visual: Optional[Visual] = None


@dataclass
class Joint:
    name: str
    jtype: str                 # "revolute" | "fixed" | "continuous" | "prismatic"
    parent: str
    child: str
    origin: np.ndarray         # 4x4, child frame relative to parent frame at angle 0
    axis: np.ndarray           # (3,)
    lower: float = 0.0         # radians (or metres for prismatic)
    upper: float = 0.0

    @property
    def movable(self) -> bool:
        return self.jtype in ("revolute", "continuous", "prismatic")


@dataclass
class URDFModel:
    links: Dict[str, Link] = field(default_factory=dict)
    joints: List[Joint] = field(default_factory=list)
    root: str = ""
    # parent link name -> joints whose parent is that link
    _children: Dict[str, List[Joint]] = field(default_factory=dict)

    @property
    def movable_joint_names(self) -> List[str]:
        return [j.name for j in self.joints if j.movable]

    # -- loading ------------------------------------------------------------

    @classmethod
    def load(cls, xacro_path, mesh_dir=None) -> "URDFModel":
        """Parse a fusion2urdf .xacro (or plain .urdf). `mesh_dir` overrides
        where `package://.../meshes/<file>` references resolve to; by default
        it's a `meshes/` folder next to (or one level up from) the xacro,
        matching fusion2urdf's `_description/{urdf,meshes}` layout."""
        xacro_path = Path(xacro_path)
        tree = ET.parse(xacro_path)
        robot = tree.getroot()

        if mesh_dir is None:
            mesh_dir = _guess_mesh_dir(xacro_path)
        mesh_dir = Path(mesh_dir) if mesh_dir else None

        model = cls()
        for elem in robot:
            tag = elem.tag.split("}")[-1]  # strip any namespace
            if tag == "link":
                link = _parse_link(elem, mesh_dir)
                model.links[link.name] = link
            elif tag == "joint":
                joint = _parse_joint(elem)
                if joint is not None:
                    model.joints.append(joint)

        model._index()
        return model

    def _index(self) -> None:
        self._children = {}
        child_links = set()
        for j in self.joints:
            self._children.setdefault(j.parent, []).append(j)
            child_links.add(j.child)
        roots = [name for name in self.links if name not in child_links]
        # Prefer a link literally called base_link; else the first root.
        self.root = "base_link" if "base_link" in roots else (roots[0] if roots else "")

    # -- forward kinematics -------------------------------------------------

    def forward(self, joint_angles: Optional[Dict[str, float]] = None,
                only_links: Optional[set] = None):
        """One tree walk producing both products of forward kinematics:

          world:  link name -> 4x4 world transform
          frames: movable joint name -> (pivot_xyz, axis_unit) in world

        Angles are in RADIANS, missing joints are 0, and each is clamped to
        its URDF limits. `link_world_transforms` and `joint_world_frames`
        are thin wrappers so existing callers are unchanged; the retargeter
        calls this directly because it needs both every iteration and the
        walk is the expensive part.

        `only_links` prunes the walk to a set of link names (plus whatever is
        needed to reach them). The full HRH1 export has 58 links, most of
        them bearings the retargeter never asks about; pruning to the ~25 it
        does ask about is a straight speedup on its inner loop. Rendering
        passes None and gets every link, as before.
        """
        angles = joint_angles or {}
        world: Dict[str, np.ndarray] = {}
        frames: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        if not self.root:
            return world, frames
        world[self.root] = np.eye(4)
        stack = [self.root]
        while stack:
            parent = stack.pop()
            Tp = world[parent]
            for joint in self._children.get(parent, []):
                if only_links is not None and joint.child not in only_links:
                    continue
                Tj = Tp @ joint.origin  # joint frame in world, before its own rotation
                q = 0.0
                if joint.movable:
                    q = float(angles.get(joint.name, 0.0))
                    if joint.lower < joint.upper:  # 0/0 = unset, don't clamp to zero
                        q = max(joint.lower, min(joint.upper, q))
                    axis = np.asarray(joint.axis, dtype=float)
                    n = np.linalg.norm(axis)
                    axis_w = Tj[:3, :3] @ (axis / n) if n > 1e-12 else np.array([1.0, 0.0, 0.0])
                    frames[joint.name] = (Tj[:3, 3].copy(), axis_w)
                    world[joint.child] = Tj @ _axis_angle_matrix(joint.axis, q)
                else:
                    world[joint.child] = Tj
                stack.append(joint.child)
        return world, frames

    def link_world_transforms(self, joint_angles: Optional[Dict[str, float]] = None) -> Dict[str, np.ndarray]:
        """link name -> 4x4 world transform, given a dict of movable-joint
        angles in RADIANS (missing joints = 0). Angles are clamped to each
        joint's URDF limits. Walks the tree from the root."""
        return self.forward(joint_angles)[0]

    def joint_world_frames(self, joint_angles: Optional[Dict[str, float]] = None):
        """For each movable joint: (pivot_xyz, axis_unit) in world coords at
        the given pose. The pivot is the point the joint rotates about; the
        axis is its rotation axis in world. Used to draw debug markers so
        the hinge location/axis is visible."""
        return self.forward(joint_angles)[1]

    # -- structure queries (for the retargeting optimiser) -------------------

    def joint_by_name(self, name: str) -> Optional[Joint]:
        for j in self.joints:
            if j.name == name:
                return j
        return None

    def resolve_link(self, name: str) -> str:
        """Match a link name, tolerating the part-version suffixes Fusion
        bakes in.

        fusion2urdf names links after the CAD component, so a re-export after
        editing the model can turn `I_DI_1` into `I_DI_v1_1` without anything
        kinematic changing -- which would otherwise break every hardcoded
        link name in the retargeter. Matching on the name with `_vN` removed
        makes those renames a non-event. Exact matches always win, so a model
        that genuinely has both stays unambiguous.
        """
        if name in self.links:
            return name
        cache = getattr(self, "_link_alias", None)
        if cache is None:
            cache = self._link_alias = {}
            for real in self.links:
                cache.setdefault(_strip_version(real), real)
        hit = cache.get(_strip_version(name))
        if hit is None:
            raise KeyError(f"no link matching {name!r} (tried version-stripped)")
        return hit

    def rigid_descendants(self, link_name: str) -> List[str]:
        """`link_name` plus every link welded to it through fixed joints only.

        These move as one body, so for any purpose that cares about the shape
        of a part -- locating a fingertip, say -- they should be treated as
        one part. Stops at the first movable joint, since anything past that
        is a separate body.
        """
        out = [link_name]
        stack = [link_name]
        while stack:
            cur = stack.pop()
            for j in self._children.get(cur, []):
                if j.movable:
                    continue
                out.append(j.child)
                stack.append(j.child)
        return out

    def _parent_joint(self) -> Dict[str, Joint]:
        if not hasattr(self, "_parent_joint_cache"):
            self._parent_joint_cache = {j.child: j for j in self.joints}
        return self._parent_joint_cache

    def ancestor_movable_joints(self, link_name: str) -> List[str]:
        """Names of the movable joints between the root and `link_name`, in
        root-first order. These are exactly the joints whose angle moves a
        point attached to that link -- i.e. the nonzero columns of that
        point's Jacobian. Cached: the tree is static, and the optimiser asks
        for the same handful of links every iteration."""
        cache = getattr(self, "_ancestor_cache", None)
        if cache is None:
            cache = self._ancestor_cache = {}
        hit = cache.get(link_name)
        if hit is not None:
            return hit
        pj = self._parent_joint()
        chain: List[str] = []
        cur = link_name
        seen = set()
        while cur in pj and cur not in seen:
            seen.add(cur)
            j = pj[cur]
            if j.movable:
                chain.append(j.name)
            cur = j.parent
        chain.reverse()
        cache[link_name] = chain
        return chain

    def link_extreme_point(self, link_name: str, reference: np.ndarray,
                           angles: Optional[Dict[str, float]] = None,
                           include_attached: bool = True) -> np.ndarray:
        """The point furthest from the world point `reference`, over
        `link_name`'s visual mesh, expressed in the LINK's own frame (metres).

        Used to locate fingertips: the tip of a distal phalanx is the vertex
        furthest from that phalanx's own hinge. Returning it in link-local
        coordinates makes it pose-independent, so it's computed once at load
        and then just rides along with the link's transform.

        `include_attached` also considers anything welded on through fixed
        joints. That matters for HRH1: the CAD gained rubber fingertip grips
        as separate links bolted to the distal phalanges, and the grip -- not
        the plastic underneath -- is the surface that actually touches
        things. Ignoring them would put the fingertip keypoint a few mm
        short, which the pinch terms would then faithfully reproduce.
        """
        link_name = self.resolve_link(link_name)
        world, _ = self.forward(angles)
        names = self.rigid_descendants(link_name) if include_attached else [link_name]
        ref = np.asarray(reference, float)
        best, best_d = None, -np.inf
        for name in names:
            link = self.links.get(name)
            if link is None or link.visual is None or link.visual.mesh_path is None:
                continue
            if not link.visual.mesh_path.exists():
                continue
            verts = read_stl_vertices(link.visual.mesh_path) * np.asarray(link.visual.scale, float)
            M = world[name] @ link.visual.origin
            world_pts = verts @ M[:3, :3].T + M[:3, 3]
            d = np.linalg.norm(world_pts - ref, axis=1)
            i = int(np.argmax(d))
            if d[i] > best_d:
                best_d, best = d[i], world_pts[i]
        if best is None:
            raise ValueError(f"link {link_name!r} has no usable visual mesh")
        return (np.linalg.inv(world[link_name]) @ np.append(best, 1.0))[:3]

    def joint_origin_in_parent(self, joint_name: str) -> Tuple[str, np.ndarray]:
        """(parent link name, pivot point in that link's frame) for a joint.

        A joint's pivot is rigidly attached to its PARENT link, so this turns
        "the P_IP hinge" into the same (link, local point) pair a fingertip
        uses -- letting the optimiser treat pivots and mesh points alike.
        """
        j = self.joint_by_name(joint_name)
        if j is None:
            raise ValueError(f"no joint named {joint_name!r}")
        return j.parent, j.origin[:3, 3].copy()

    def required_links(self, link_names: Sequence[str]) -> set:
        """Every link needed to place `link_names` -- themselves plus all
        their ancestors. Feed to forward(only_links=...) to prune the walk."""
        pj = self._parent_joint()
        need = set()
        for name in link_names:
            cur = name
            while cur is not None and cur not in need:
                need.add(cur)
                j = pj.get(cur)
                cur = j.parent if j is not None else None
        return need

    def point_jacobians(
        self,
        targets: Sequence[Tuple[str, np.ndarray]],
        dof_names: Sequence[str],
        angles: Optional[Dict[str, float]] = None,
        only_links: Optional[set] = None,
    ):
        """Positions and analytic Jacobians for a set of attached points.

        `targets` is a sequence of (link name, point in that link's frame).
        Returns (P, J) with P (K, 3) world positions and J (K, 3, D) where
        D = len(dof_names), so J[k, :, i] = d P[k] / d q_i.

        For a revolute joint the column is the textbook geometric Jacobian
        `axis_i x (p_k - pivot_i)`: rotating about a world axis through a
        world pivot moves any attached point along that cross product. It is
        exact (not a finite difference), which is what makes a Gauss-Newton
        step cheap and well-scaled. Columns for joints that aren't ancestors
        of the point's link stay zero, since they can't move it.
        """
        world, frames = self.forward(angles, only_links=only_links)
        dof_index = {n: i for i, n in enumerate(dof_names)}
        K, D = len(targets), len(dof_names)
        P = np.zeros((K, 3))
        J = np.zeros((K, 3, D))
        for k, (link_name, local) in enumerate(targets):
            T = world[link_name]
            p = T[:3, :3] @ np.asarray(local, float) + T[:3, 3]
            P[k] = p
            for jname in self.ancestor_movable_joints(link_name):
                i = dof_index.get(jname)
                if i is None:
                    continue  # a joint we're deliberately not solving for
                pivot, axis = frames[jname]
                J[k, :, i] = np.cross(axis, p - pivot)
        return P, J


def _guess_mesh_dir(xacro_path: Path) -> Optional[Path]:
    # fusion2urdf: .../<pkg>/urdf/robot.xacro  with meshes at .../<pkg>/meshes
    for cand in (xacro_path.parent / "meshes", xacro_path.parent.parent / "meshes"):
        if cand.is_dir():
            return cand
    return None


def _resolve_mesh(filename: str, mesh_dir: Optional[Path]) -> Optional[Path]:
    if not filename:
        return None
    base = filename.split("/")[-1]  # strip package://.../meshes/ prefix
    if mesh_dir is not None:
        p = mesh_dir / base
        if p.exists():
            return p
        return p  # return the expected path even if missing; caller can warn
    return None


def _parse_link(elem, mesh_dir) -> Link:
    link = Link(name=elem.get("name", ""))
    visual = elem.find("visual")
    if visual is not None:
        geom = visual.find("geometry")
        mesh = geom.find("mesh") if geom is not None else None
        origin_el = visual.find("origin")
        xyz = _floats(origin_el.get("xyz") if origin_el is not None else None, [0, 0, 0])
        rpy = _floats(origin_el.get("rpy") if origin_el is not None else None, [0, 0, 0])
        scale = _floats(mesh.get("scale") if mesh is not None else None, [1, 1, 1])
        mesh_path = _resolve_mesh(mesh.get("filename") if mesh is not None else "", mesh_dir)
        link.visual = Visual(mesh_path=mesh_path, origin=_make_transform(xyz, rpy), scale=scale)
    return link


def _parse_joint(elem) -> Optional[Joint]:
    parent_el = elem.find("parent")
    child_el = elem.find("child")
    if parent_el is None or child_el is None:
        return None
    origin_el = elem.find("origin")
    xyz = _floats(origin_el.get("xyz") if origin_el is not None else None, [0, 0, 0])
    rpy = _floats(origin_el.get("rpy") if origin_el is not None else None, [0, 0, 0])
    axis_el = elem.find("axis")
    axis = _floats(axis_el.get("xyz") if axis_el is not None else None, [1, 0, 0])
    limit_el = elem.find("limit")
    lower = float(limit_el.get("lower", 0.0)) if limit_el is not None else 0.0
    upper = float(limit_el.get("upper", 0.0)) if limit_el is not None else 0.0
    return Joint(
        name=elem.get("name", ""),
        jtype=elem.get("type", "fixed"),
        parent=parent_el.get("link", ""),
        child=child_el.get("link", ""),
        origin=_make_transform(xyz, rpy),
        axis=axis,
        lower=lower,
        upper=upper,
    )
