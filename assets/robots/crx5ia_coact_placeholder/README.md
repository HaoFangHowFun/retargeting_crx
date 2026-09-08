# CRX-5iA + Co-act EGP-C 40 placeholder

This is a visual and frame-chain placeholder for the left arm. It is not the
measured Co-act EGP-C 40 geometry and must not be used for collision, mass,
reachability, TCP dimensions, force limits, or hardware control.

The model has seven actuated joints:

```text
J1 J2 J3 J4 J5 J6 coact_opening
```

`coact_opening` is a provisional 0–12 mm prismatic opening coordinate. The
right jaw is fixed and the left jaw is the visual moving proxy; a real EGP-C
model will need the measured adapter, TCP, jaw travel and controller semantics.
The CRX meshes are reused from `crx5ia_leap_paxini`, so no second CRX mesh copy
is stored. The generated URDF is intentionally separate from the 22-DOF CRX +
LEAP asset and does not change its single-arm behavior.

Generate it with:

```bash
env -u PYTHONPATH /home/howard/.venvs/retargeting_crx/bin/python \
  scripts/import_coact_placeholder.py \
  --fanuc-source /home/howard/ws_fanuc/src/fanuc_description \
  --xacro-python-path /opt/ros/jazzy/lib/python3.12/site-packages \
  --output assets/robots/crx5ia_coact_placeholder/urdf/crx5ia_coact_placeholder.urdf
```

The Co-act model currently has no retargeting profile. The intended future
mapping is left Quest wrist pose plus left thumb-index distance to gripper
opening, while the right arm continues using the existing LEAP profile.
