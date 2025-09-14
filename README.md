# Bug0 Robot Controller for Autonomous Navigation (ROS2)
This project implements the **Bug0 algorithm** for autonomous navigation using **ROS2** and **TurtleBot3** in simulation.  
The robot navigates toward a predefined goal position, detects obstacles using a laser scanner, and applies a wall-following behavior until the path is clear.  

## Features
- Autonomous navigation toward a goal `(x, y)` position
- Obstacle avoidance using **LaserScan** data
- State machine with three states:
  - **Go Straight** → Move directly toward the goal
  - **Follow Wall** → Navigate around obstacles
  - **Finish** → Stop when goal is reached
- Configurable parameters (goal position, speeds, obstacle thresholds)

## Project Structure
```
bug0/
├── launch/
│ └── bug0.launch.py # Launch file for simulation + controller
├── src/
│ └── bug_move.py # Main Bug0 controller node
├── package.xml
└── setup.py
```

## Requirements
- **ROS2 Humble** (recommended)
- **TurtleBot3 Simulation** packages
- Python3 with `tf_transformations`

## Usage
### Option 1: Single launch file
```bash
ros2 launch bug0 bug0.launch.py
```
### Option 2: Start separately
```bash
1. Start TurtleBot3 simulation: ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage4.launch.py
2. Run Bug0 controller: ros2 run bug0 bug_move
```

## Demo
<video src="videos/bug0_demo.mp4" width="600" controls></video>
