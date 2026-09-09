# CRX-5iA + LEAP Hand (left)

This bundle combines the existing FANUC CRX-5iA arm geometry with the official
left-hand LEAP V1 geometry from
`leap-hand/Bidex_VisionPro_Teleop`, preserving the repository's CRX+LEAP frame
and joint-name contract (`J1..J6`, then `joint_0..joint_15`). The right-hand
bundle is unchanged.

The URDF reuses arm visual and collision meshes from the sibling
`crx5ia_leap_paxini/meshes/crx5ia/` directory through relative paths. Keep both
robot directories together; only the left-hand meshes are stored locally.

The left hand uses its upstream left palm and finger meshes and joint origins.
The repository adds `wrist` and fingertip-center frames. The `flange_to_leap`
translation compensates for the different upstream palm origin: the middle
finger root (`joint_5`) matches the working right-hand model in flange
coordinates, with the same mount rotation. Specifically,
`t_left = t_right + R_mount * (p_right_joint5 - p_left_joint5)`.
The resulting translation is `[0.037336626243399, 0.047897767636037,
0.140188227821871]` metres. `palm_to_wrist` subtracts that palm-coordinate
offset to preserve the reference flange-to-wrist transform. This is a
model-based alignment using the right-hand mount, not a measured hardware
adapter calibration. The left CRX home
is configured separately from the right CRX home in
`configs/robots/crx5ia_leap_paxini_left.yaml`.

## Mount orientation and future merge

This branch has not applied the 180-degree LEAP mount flip. According to the
maintainer, the physical-test version on `main` already uses that flip.
When merging, adopt `main`'s mount orientation, then recompute or validate the
translation compensation and `palm_to_wrist` transform against that orientation.
The offset documented above is for this branch's current unflipped mount;
do not use it to overwrite the installation changes from `main`.

Source: <https://github.com/leap-hand/Bidex_VisionPro_Teleop>,
`leap_hand_mesh_left/`. Source license text is retained in
`provenance_LICENSE.md`; verify any downstream asset licensing requirements
before redistribution.
