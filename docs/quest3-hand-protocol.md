# Quest 3 hand-frame protocol

The integrated Quest transport accepts one versioned JSON packet type:
`quest3.hand_frame.v1`. Positions are in metres, rotations are XYZW
quaternions, and all poses share the WebXR `local` reference space.

Each packet carries a browser-local sequence number, WebXR timestamp, optional
head pose, and explicit `left` and `right` hand records. A tracked hand contains
25 positions, 25 rotations, and 25 joint radii in WebXR joint order. An
untracked hand is a valid partial frame. Invalid lengths, non-finite values,
bad quaternion norms, negative radii, unsupported packet types, and unsupported
reference spaces are rejected without replacing the last valid frame.

The desktop receiver adds a monotonic sequence number and
`received_monotonic_ns`. Consumers use that receive time for freshness because
the sender clock is not synchronized with the desktop.

The robot adapter removes the four non-thumb metacarpal entries to produce the
21-joint MANO order. It expresses joint positions relative to the selected
wrist, converts the wrist basis with the matching operator-to-MANO transform,
then applies the configured sensor-to-robot calibration and relative initial
wrist alignment. Transport data remains robot-independent; motor command and
safety policy belong to the downstream execution backend.
