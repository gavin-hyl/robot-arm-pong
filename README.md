# 🏓 Robot Arm Ping Pong

A sophisticated robotics simulation where a 7-DOF robot arm plays ping pong using advanced inverse kinematics, trajectory planning, and real-time ball tracking.

## 🎯 Project Overview

This project implements an intelligent robot arm controller that can track and hit a simulated ping pong ball using advanced robotics techniques. The system features a complete physics simulation with gravity, collision detection, and real-time trajectory planning to intercept the ball and direct it towards a target.


## ✨ Key Features

### 🤖 Advanced Robot Control
- **7-DOF Robot Arm**: Full 7 degrees of freedom for complex maneuvering
- **Inverse Kinematics**: Sophisticated IK solver with multiple solution strategies
- **Singularity Handling**: Robust weighted inverse solutions near singularities
- **Joint Trajectory Planning**: Smooth spline-based joint space trajectories

### 🎾 Intelligent Ball Tracking
- **Physics Simulation**: Real-time ball simulation with gravity and collision
- **Predictive Interception**: Forward integration to predict optimal impact points
- **Workspace Awareness**: Ball generation within robot's reachable workspace
- **Impact Optimization**: Condition number-based impact point selection

### 🎯 Multi-Task Management
- **Primary Task**: Ball interception and redirection
- **Secondary Task**: Maintaining optimal arm configuration
- **Graceful Degradation**: Smooth fallback to idle position for unreachable balls
- **Collision Detection**: Real-time paddle-ball collision with physics response

## 🏗️ System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Ball Engine   │    │   Controller    │    │  Kinematic      │
│                 │    │                 │    │  Chain          │
│ • Physics Sim   │◄──►│ • Trajectory    │◄──►│ • Forward Kin   │
│ • Collision     │    │   Planning      │    │ • Jacobians     │
│ • Visualization │    │ • Inverse Kin   │    │ • Transforms    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                    ┌─────────────────┐
                    │      RVIZ       │
                    │                 │
                    │ • 3D Visualizer │
                    │ • Robot Model   │
                    │ • Ball Tracking │
                    └─────────────────┘
```

## 🚀 Quick Start

### Prerequisites
- ROS 2 (tested with Humble)
- Python 3.8+
- NumPy
- RVIZ2

### Installation & Usage

1. **Clone the repository** into your ROS workspace:
   ```bash
   cd your_ros_workspace/src
   git clone <repository-url> robot-arm-pong
   ```

2. **Build and run** the project:
   ```bash
   cd robot-arm-pong
   bash run.sh
   ```

The `run.sh` script will:
- Clean previous build artifacts
- Build the project from scratch
- Launch all necessary ROS nodes
- Start RVIZ for visualization

### What You'll See
- **RVIZ Visualization**: 3D robot arm model with moving joints
- **Ball Simulation**: Red sphere following realistic physics
- **Real-time Tracking**: Robot arm moving to intercept the ball
- **Trajectory Visualization**: Smooth arm movements towards impact points

## 🧠 Technical Details

### Inverse Kinematics Solver
The system uses a weighted pseudo-inverse approach with:
- **Condition Number Monitoring**: Prevents numerical instability
- **Multiple Solution Strategies**: Handles kinematic redundancy
- **Singularity Avoidance**: Weighted inverses near singular configurations

### Trajectory Planning
- **Spline Interpolation**: Smooth joint space trajectories
- **Predictive Control**: Forward integration for impact prediction
- **Workspace Constraints**: Ensures all planned motions are feasible

### Ball Physics
- **Gravity Simulation**: Realistic projectile motion
- **Collision Response**: Elastic collision with paddle
- **Boundary Conditions**: Automatic ball regeneration

## 🎮 Controls & Behavior

### Automatic Operation
The system runs autonomously:
1. **Ball Generation**: New balls spawn randomly in the workspace
2. **Trajectory Planning**: Robot calculates optimal interception
3. **Execution**: Smooth movement to impact point
4. **Return**: Graceful return to idle position
5. **Repeat**: Continuous operation with new balls

### Failure Modes
- **Unreachable Balls**: Robot returns to idle position
- **Kinematic Limits**: Smooth degradation to safe configurations
- **Collision Avoidance**: Prevents self-collision and workspace violations

## 📊 Performance Metrics

- **Success Rate**: High percentage of successful ball interceptions
- **Trajectory Smoothness**: Continuous joint velocities and accelerations
- **Computational Efficiency**: Real-time performance at 100+ Hz
- **Robustness**: Graceful handling of edge cases and failures

## 🔧 Configuration

Key parameters can be adjusted in the source files:
- **Arm Weights**: `ARM_WEIGHTS` in `Controller.py`
- **Task Space**: `TASK_SPACE_R` and `TASK_SPACE_P` in `Controller.py`
- **Ball Properties**: `GRAVITY` and physics parameters in `BallNode.py`
- **Trajectory Timing**: `t_to_idle` and other timing parameters

## 🏆 Advanced Features

1. **Joint-based Trajectories**: Smooth motion in joint space
2. **Singularity Handling**: Robust behavior near kinematic singularities
3. **Multi-objective Optimization**: Primary and secondary task management
4. **Condition Number Analysis**: Intelligent impact point selection
5. **Workspace-based Generation**: Smart ball placement for reachability
6. **Graceful Degradation**: Smooth fallback for impossible tasks


## 📝 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

---

*This project demonstrates advanced robotics concepts including inverse kinematics, trajectory planning, and real-time control in a fun and engaging ping pong simulation.*
