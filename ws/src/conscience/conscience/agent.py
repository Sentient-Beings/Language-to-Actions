import rclpy
from rclpy.node import Node
from agent_interfaces.srv import GetSensorDistances, GetImage
from agent_interfaces.msg import ControllerInterface
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import json
from groq import Groq
import base64
from . import graph_util
import time
from typing import Callable
import os
GROQ_API_KEY = os.getenv("GROQ_API_KEY")

if not GROQ_API_KEY:
    raise ValueError("GROQ_API_KEY environment variable is not set")

"""Setup the Control Interface: This is a high-level control which will be filled by the Agent"""
High_level_control = Callable[[], str]
_high_level_control: High_level_control = lambda: ""
class Agent_control:
    """Encapsulate the high level control logic"""
    def getter_high_level_command(getter: High_level_control):
        global _high_level_control
        _high_level_control = getter
    def get_high_level_command():
        command = _high_level_control()
        return command
    
class Agent(Node): 
    """ROS comm"""
    def __init__(self): 
        super().__init__('agent')
        self.declare_parameter('user_input', '')
        user_callback_group = MutuallyExclusiveCallbackGroup()
        sensor_callback_group = MutuallyExclusiveCallbackGroup()
        camera_callback_group = MutuallyExclusiveCallbackGroup()
        
        self.create_subscription(String, 'user_input', self.user_input_callback, 10, callback_group=user_callback_group)
        self.user_input = ""
        
        # we check for any high level command from the agent and execute them right away
        self.controller_interface_publisher_ = self.create_publisher(ControllerInterface, 'controller_interface', 10)
        self.create_timer(1, self.publish_high_level_command)
        
        self.sensor_client_ = self.create_client(GetSensorDistances, 'get_saftey_distance', callback_group=sensor_callback_group)
        self.lidar_sensor_reading = {}
        self.sensor_response = None 
        
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10,
            callback_group=camera_callback_group
        )
        self.cv_bridge = CvBridge()
        self.base64_image = None
        
        self.client = Groq()
        self.get_logger().info('Agent initialized')
    
    ### ROBOT CONTROL ###
    def publish_high_level_command(self):
        """Publishes the high level command to the Simple Controller"""
        high_level_command = Agent_control.get_high_level_command()
        msg = ControllerInterface()
        
        if high_level_command:
            msg.move_direction = high_level_command
            self.get_logger().info('Publishing high level command: {}'.format(high_level_command))
            self.controller_interface_publisher_.publish(msg)
        else:
            self.get_logger().debug('No high level command to publish, therefore stopping the robot')
            msg.move_direction = "stop"
            self.controller_interface_publisher_.publish(msg)
    
    ### USER INTERACTION ###
    def user_input_callback(self, msg):
        self.user_input = msg.data
        # Set the user input in the graph utility
        graph_util.User_Input_Retrieval.setter_user_input(self.send_user_input_to_agent) 
        self.get_logger().info('Received user input: {}'.format(self.user_input))
        # Invoke the reasoning
        result = self.invoke_reasoning(self.user_input)
        while not result:
            self.get_logger().warn('Waiting for result from AI assistant')
            time.sleep(0.1)
        self.process_result(result)
        
    def send_user_input_to_agent(self):
        return self.user_input
    
    ### AGENT EXECUTION ###
    def invoke_reasoning(self, user_input: str):
        """Invokes the graph with the user input and also updates the lidar sensor reading"""
        try:
            graph_util.Observation_Retrieval.setter_observation_data(self.analyze_scene)    
            return graph_util.run_graph(user_input)
        except Exception as e:
            self.get_logger().error('Error in invoke_reasoning method: {}'.format(e))
            return None
        finally:
            # Always reset the sensor response
            self.sensor_response = None  
    
    def process_result(self, result):
        """Processes the result from the Agent"""
        if result is None:
            self.get_logger().warn('Failed to get a response from the Agent')
        else:
            self.get_logger().info('Received result from the Agent: {}'.format(result))
            
    ### LIDAR ###
    def get_minimum_distance(self, request):
        """Sends a request to the sensor service to get the data"""
        self.get_logger().debug('Requesting minimum distance to obstacle from sensor')
        
        while not self.sensor_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting again...')
            
        self.sensor_future = self.sensor_client_.call_async(request)
        self.sensor_future.add_done_callback(self.future_callback)
    
    def future_callback(self, future): 
        """Handles the future object"""
        try:
            self.sensor_response = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            self.sensor_response = None
        finally:
            self.sensor_future = None

    def get_laser_scan_data(self):
        """Returns the lidar sensor reading from the last sensor response"""
        if not self.lidar_sensor_reading:
            self.get_logger().warn('Lidar sensor reading is empty or None')
            # Return an empty dictionary instead of None
            return {}  
        return self.lidar_sensor_reading
    
    def send_lidar_request(self):
        """Sends a request to the sensor service to get the data"""
        request = GetSensorDistances.Request()
        # Reset the response
        self.sensor_response = None 
        self.get_minimum_distance(request)
        self.get_logger().debug('Waiting for sensor response')
        
        while self.sensor_response is None:
            # Wait for the sensor response
            time.sleep(0.1) 
        
        self.lidar_sensor_reading = {
            "Obstacle in front right": self.sensor_response.min_distance_front_right,
            "Obstacle in front left": self.sensor_response.min_distance_front_left,
            "Obstacle in front": self.sensor_response.min_distance_front
        }
        
    ### VISION ###
    def camera_callback(self, msg):
        """Callback for the camera image"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, buffer = cv2.imencode('.jpg', cv_image)
            # Encode the image to base64 -> this will be used by the agent
            self.base64_image = self.encode_image(buffer.tobytes())
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {str(e)}')

    def encode_image(self, img_bytes):
        """Encode image bytes to base64 string."""
        return base64.b64encode(img_bytes).decode('utf-8')
    
    def create_prompt(self,lidar_data):
        """Create a structured prompt combining vision and LiDAR data."""
        prompt = f"""You are analyzing a real-world scene. The following observations have been made from LiDAR data:

- LiDAR detects obstacles at:
    - Front right: {lidar_data['Obstacle in front right']:.2f} meters away
    - Front left: {lidar_data['Obstacle in front left']:.2f} meters away
    - Directly in front: {lidar_data['Obstacle in front']:.2f} meters away

Your task is to:
1. Identify and name the objects you see.
2. Note any significant features (like shape and size) for each object.
3. Use the LiDAR data to mention the distances of these objects if applicable.
4. Describe any potential paths or openings that are clear of obstacles.

Keep the description clear and focused on object identification and navigable areas.
        """
        return prompt
    
    def analyze_scene(self):
        try:
            # request the lidar data
            self.send_lidar_request()
            # update the lidar data
            lidar_data = self.get_laser_scan_data()
            
            # Safety check: See if there is no image or lidar data
            if not self.base64_image and not lidar_data:
                self.get_logger().warn('No scene observation, STOP the robot')
                return "WARNING!: No scene observation, STOP the robot"
            
            prompt = self.create_prompt(lidar_data)
            response = self.client.chat.completions.create(
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{self.base64_image}",
                                },
                            },
                        ],
                    }
                ],
                model="llama-3.2-90b-vision-preview"
            )
            result = response.choices[0].message.content
            return result
        
        except Exception as e:
            return {"WARNING!: Error in scene analysis: {}".format(e)}
        
    # def observation(self):
    #     """Get the scene observation grounded in the real world using lidar data."""
    #     return self.analyze_scene()


def main(args=None):
    rclpy.init(args=args)
    agent = Agent()
    executor = MultiThreadedExecutor()
    executor.add_node(agent)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        agent.get_logger().info('Keyboard interrupt received, shutting down')
    finally:
        agent.get_logger().info('Destroying node and shutting down')
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
