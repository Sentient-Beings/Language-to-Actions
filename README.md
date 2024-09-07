# Agentic Behavior in Robotics

## Vision

The Agentic behavior aims to create intelligent robots capable of perceiving their environment and performing actions in the real world, just like humans do. Our goal is to develop a versatile system that can be applied to various robotic platforms, from simple differential drive robots to complex legged robots, robotic arms, and autonomous vehicles.

As humans, we use our brain to interpret the world around us and perform actions to achieve our goals. We believe that the same process can be replicated in robots to create truly intelligent and adaptive behavior.

The LLM and VLM can act as the brain of the robot, using chain of thought and reasoning to make decisions and perform actions. The robot can use its sensors to perceive the environment and use the LLM and VLM to understand the world and perform actions.

By integrating advanced perception, decision-making, and action execution capabilities, we seek to push the boundaries of robotic autonomy and adaptability in diverse environments.

## Current Implementation

This repository is a work in progress and currently demonstrates a proof-of-concept implementation using a simple differential drive robot equipped with a LIDAR sensor, user can ask the robot to perform actions in the environment and the agent will respond with the appropriate action using the tools at its disposal. This serves as a starting point for our broader vision of agentic behavior in robotics.

### Differential Drive Robot with Agentic Behavior

- Integration with ROS2
- A simple Gazebo world with obstacles for LIDAR-based environment perception
- Natural language interaction for robot control, actions including:
  - Exploration
  - Navigation
  - Obstacle avoidance
  - Querying robot state and environment
- Decision-making based on sensor data and user queries

## Getting Started

### Prerequisites

- ROS2 (tested on Humble)
- LangGraph
- LangChain
- Groq (or any fast inference provider)

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/Sentient-Beings/agentic_workflow.git
   cd agentic_workflow
   ```

2. Install dependencies:
    setup a python virtual environment and install the dependencies:
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   ```

3. Build the ROS2 workspace:
   ```bash
   colcon build 
   ```

4. Source the ROS2 environment:
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

3. In a new terminal, start the agentic behavior node:
   ```bash
    ros2 run conscience agent
   ```

4. To interact with the agent, open a new terminal and publish a message:
   ```bash
   ros2 topic pub /user_input std_msgs/msg/String "data: 'In which directions do we have an obstacle and at what distance ?'" -1
   ```

In order to run this successfully, you need to have a Groq API key. You can get one https://groq.com/

## Project Structure

It is a work in progress, currently you can only query the agent with what it is observing from the output of the lidar data. 

We will be integrating the additional behaviours like giving it capability to execute tasks, navigate, perform actions in the environment. 

## Future Work

- Integration with additional robotic platforms (legged robots, robotic arms, autonomous vehicles)
- Expansion of sensor suite and perception capabilities
- Implementation of more complex decision-making algorithms
- Development of a standardized interface for different robotic systems
- Continuous learning and adaptation to new environments and tasks   

## Acknowledgments

- [LangChain](https://www.langchain.com/) for developing [LangGraph](https://langchain-ai.github.io/langgraph/), which was critical in developing this system. LangGraph's flexible framework for building stateful, multi-step applications with LLMs has been instrumental in creating our agentic behavior system.
- The ROS2 community for providing a robust framework for robotic software development.

We're grateful for these open-source tools and the communities behind them, which make projects like this possible.