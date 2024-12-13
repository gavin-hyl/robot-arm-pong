# ME/EE/CS 133a Final Project

## Instructions

Clone this repository into `proj_ws` (or any workspace of your choosing)

```bash
cd proj_ws
bash run.sh
```

TODOs
- velocity of hitting is free (right now we only have nominal velocity, if we don't enforce these constraints then we can choose different paths)
- swinging motion and concentrating movement around lower joints. Use freedom in velocity to choose preferred joints
- velocity 1 DoF-ish, position 5 DoF-ish

- Multiple balls at once (push until later)

- fore and back hand
- choosing a better impact position
- Explore speed to hit the ball

Questions
- If doing joint trajectory, 

- Rotate 180 degrees to achieve back hand impact
- freeze the ball for a bit when it hits

Features:
1. joint-based trajectories (done - gavin)
2. singularity handling with weighted inverses (done)
3. primary and secondary task (done)
4. considering condition numbers (progress)
5. graceful handling of ball throws outside of work space (done)
6. workspace based ball generation (and workspace visualization)
7. considering joint limits