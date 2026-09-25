# RH56DFX MuJoCo assets

The hand MJCF and meshes in this directory are derived from
[`correlllab/h1_mujoco`](https://github.com/correlllab/h1_mujoco), commit
`1208f0e95dcfa55ccb845583a73fed15315ec966` (MIT license). They model the
left and right Inspire RH56DFX hands.

`run_sim_loop.py --hand-type inspire` attaches the two hand roots to the G1
left/right wrist-yaw bodies at load time. Prefixes are applied during MJCF
composition, so the original standalone hand files remain easy to compare
with upstream.

The current G1 attachment transforms use the validated H1-2 Inspire mounting
orientation from upstream and a 54 mm wrist offset. They are suitable for the
first teleoperation-in-simulation milestone; the transforms must still be
measured against the physical G1 adapters before sim-to-real geometry is
treated as calibrated.
