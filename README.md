# ME/EE/CS 133a Final Project

## Instructions

Clone this repository into `proj_ws` (or any workspace of your choosing)

```bash
cd proj_ws
colcon build
source install/setup.bash
ros2 launch final_project final_project.launch.py
```

TODOs (by Thursday night)
- prismatic joint for the tip position (2 joints for where to hit)
    - secondary task for tip
- velocity of hitting is free (right now we only have nominal velocity, if we don't enforce these constraints then we can choose different paths)
- joint trajectories vs weighted inverse
- swingiing motion and concentrating movement around lower joints. Use freedom in velocioty to choose preferred joints
- Tip trajectory formulation asap
