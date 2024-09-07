# Agentic Robotics Framework

## Vision

The Agentic Robotics Framework aims to create intelligent agents capable of perceiving their environment and performing actions in the real world. My goal is to develop a versatile system that can be applied to various robotic platforms, from simple differential drive robots to complex legged robots, robotic arms, and autonomous vehicles.

By integrating advanced perception, decision-making, and action execution capabilities, we seek to push the boundaries of robotic autonomy and adaptability in diverse environments.

## Current Implementation

This repository currently demonstrates a proof-of-concept implementation using a simple differential drive robot equipped with a LIDAR sensor. This serves as a starting point for our broader vision of agentic behavior in robotics.

### Differential Drive Robot with agentic behavior

- Integration with ROS2 for robotic control and sensor data processing
- Empty world with few obstacles and a LIDAR-based environment perception
- Natural language interaction with robot to perform actions
- Decision-making based on sensor data and user queries

## Getting Started

### Major Prerequisites

- ROS2 (tested on Humble)
- LangGraph
- LangChain
- Groq ( or any inference provider )

### Installation

1. Clone the repository:

```bash
git clone https://github.com/Sentient-Beings/agentic_workflow.git
```

2. Install dependencies:

```bash
pip install -r requirements.txt
```

3. Build the workspace:

```bash
colcon build
```

4. Source the workspace:

```bash
source install/setup.bash
```

### Running the Demo

1. Launch the simulation environment:

```bash
ros2 launch articubot_one launch_sim.launch.py
```

2. In a new terminal, start the sensor processing node:

```bash
ros2 run agent_controller sensor_processor
```
3. In another terminal, run the agent:

```bash
ros2 run conscience agent
```

4. In another terminal, run the user interface:

```bash
ros2 run conscience user_interface
```
5. To interact with the agent, open a new terminal and publish a message:

```bash
ros2 topic pub /user_input std_msgs/msg/String "data: 'in which directions do we have an obstacle and at what distance ?'" -1
```

