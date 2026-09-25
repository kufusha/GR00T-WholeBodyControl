# G1 Inspire valve simulation

This folder contains the print-only valve rotor and the vertical/horizontal
free-standing MuJoCo fixtures. Object loading is opt-in, so a normal simulator
launch keeps its current behavior.

Run the vertical fixture from the repository root:

```bash
python gear_sonic/scripts/run_sim_loop.py \
    --hand-type inspire \
    --object-load valve
```

Run the horizontal fixture:

```bash
python gear_sonic/scripts/run_sim_loop.py \
    --hand-type inspire \
    --object-load valve-horizontal
```

The valve axis is 0.95 m above the floor. The stand defaults to `(0.55, 0, 0)`
in front of G1 and the vertical wheel faces the robot.

The same launcher can attach another MJCF without editing the simulator:

```bash
python gear_sonic/scripts/run_sim_loop.py \
    --hand-type inspire \
    --object-load '/absolute/path/to/object.xml#object-root'
```
