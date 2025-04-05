# 7-DoF Robot Arm Ping Pong

## Instructions

Clone this repository into `proj_ws` (or any workspace of your choosing)

```bash
cd proj_ws
bash run.sh
```

The bash script cleans the build artifacts, builds the project from scratch, and launches all nodes.

Features:
1. joint-based trajectories
2. singularity handling with weighted inverses
3. primary and secondary tasks
4. considering condition numbers
5. graceful handling of ball throws outside of workspace
6. workspace based ball generation
