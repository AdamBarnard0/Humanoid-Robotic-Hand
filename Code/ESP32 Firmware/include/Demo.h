#pragma once

// ---------------------------------------------------------------------
// Scripted movement demonstration — shows off the hand's general range of
// motion. Purely for show; nothing else depends on it.
//
// Sequence:
//   1. all abduction swinging together, right-left x2
//   2. abduction wave rippling across the hand, alternating direction
//   3. P/IP wave — P curl, then P up as IP curls, then IP extends as
//      P curls again, then flat; the bend passes back and forth along
//      the fingers
//   4. thumb swings clear, fingers close pinky->index, thumb folds back
//      over the fist
//   5. thumb swings clear again, index points out of the fist and its tip
//      circles
//
// Blocking, ~15 s. Send any character while it runs to abort — the hand
// returns to zero.
// ---------------------------------------------------------------------
void runMovementDemo();

// ---------------------------------------------------------------------
// demo2 — the thumb touches each fingertip in turn: pinky, ring, middle,
// index. Starts from zero and returns to zero.
//
// The four poses were captured by hand on the real hand rather than
// derived, so they already account for the thumb having to reach further
// and the fingers needing to stay clear of each other.
//
// Transitions are interpolated. Send any character to abort.
// ---------------------------------------------------------------------
void runTouchDemo();

// demo3 — wrist twirl, then pick-and-place: rotate to the side and dip, grab, lift,
// carry across while lowering, release, return to neutral. Eased motion
// throughout. Any key aborts, easing back to neutral.
void runWristDemo();
