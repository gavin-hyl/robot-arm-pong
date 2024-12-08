# ME/EE/CS 133a Final Project

## Instructions

Clone this repository into `proj_ws` (or any workspace of your choosing)

```bash
cd proj_ws
colcon build
source install/setup.bash
ros2 launch final_project final_project.launch.py
```

TODOs
- velocity of hitting is free (right now we only have nominal velocity, if we don't enforce these constraints then we can choose different paths)
- joint trajectories (preferred, use NR to convert final task position to joint position) vs weighted inverse
- swinging motion and concentrating movement around lower joints. Use freedom in velocity to choose preferred joints
- velocity 1 DoF-ish, position 5 DoF-ish

- Multiple balls at once (push until later)

- fore and back hand
- choosing a better impact position
- Explore speed to hit the ball