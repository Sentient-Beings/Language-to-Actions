import rclpy
from rclpy.node import Node
from agent_interfaces.srv import GetSensorDistances
from agent_interfaces.msg import ControllerInterface
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
# from .graph_util import run_graph , Lidar_retrieval
from . import graph_util
import time
from typing import Callable
# Setup the Control Interface: This is a high-level control which will be filled by the Agent
High_level_control = Callable[[], str]
_high_level_control: High_level_control = lambda: ""
class Agent_control:
    def getter_high_level_command(getter: High_level_control):
        global _high_level_control
        _high_level_control = getter
    def get_high_level_command():
        command = _high_level_control()
        return command
####################################################################################################################
############################################## ROS2 COMM Class #####################################################
####################################################################################################################
class Agent(Node): 
    def __init__(self): 
        super().__init__('agent')
        self.declare_parameter('user_input', '')
        user_callback_group = MutuallyExclusiveCallbackGroup()
        sensor_callback_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(String, 'user_input', self.user_input_callback, 10, callback_group=user_callback_group)
        self.controller_interface_publisher_ = self.create_publisher(ControllerInterface, 'controller_interface', 10)
        self.create_timer(1, self.publish_high_level_command)
        
        self.sensor_client_ = self.create_client(GetSensorDistances, 'get_saftey_distance', callback_group=sensor_callback_group)
        self.lidar_sensor_reading = {}
        self.sensor_response = None 
        self.user_input = ""
        self.get_logger().info('Agent initialized')
    
    def get_minimum_distance(self, request):
        '''
        Description: Sends a request to the sensor service to get the data
        '''
        self.get_logger().info('Requesting minimum distance from sensor')
        while not self.sensor_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting again...')
        self.sensor_future = self.sensor_client_.call_async(request)
        self.sensor_future.add_done_callback(self.future_callback)

    def future_callback(self, future): 
        '''
        Description: Callback function for the future object
        '''
        try:
            self.sensor_response = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            self.sensor_response = None
        finally:
            self.sensor_future = None

    def get_laser_scan_data(self):
        '''
        Description: Returns the lidar sensor reading from the last sensor response
        '''
        if not self.lidar_sensor_reading:
            self.get_logger().warn('Lidar sensor reading is empty or None')
            # Return an empty dictionary instead of None
            return {}  
        self.get_logger().debug('Returning lidar sensor reading')
        return self.lidar_sensor_reading
    
    def publish_high_level_command(self):
        '''
        Description: Publishes the high level command to the Simple Controller 
        '''
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
            
    def invoke_reasoning(self, user_input: str):
        '''
        Description: Invokes the graph with the user input and also updates the lidar sensor reading 
        '''
        try:
            request = GetSensorDistances.Request()
             # Reset the response
            self.sensor_response = None 
            self.get_minimum_distance(request)
            self.get_logger().info('Waiting for sensor response')
            while self.sensor_response is None:
                 # Short sleep to avoid busy waiting
                time.sleep(0.1) 
            
            self.lidar_sensor_reading = {
                "The distance to obstacle in the right direction": self.sensor_response.min_distance_right,
                "The distance to obstacle in the left direction": self.sensor_response.min_distance_left,
                "The distance to obstacle in the front direction": self.sensor_response.min_distance_front
            }
            
            graph_util.Lidar_retrieval.setter_sensor_data(self.get_laser_scan_data)    
            return graph_util.run_graph(user_input)
        except Exception as e:
            self.get_logger().error('Error in invoke_reasoning method: {}'.format(e))
            return None
        finally:
            self.sensor_response = None  

    def user_input_callback(self, msg):
        self.user_input = msg.data
        # Set the user input in the graph utility
        graph_util.User_Input_Retrieval.setter_user_input(self.send_user_input_to_agent) 
        self.get_logger().info('Received user input: {}'.format(self.user_input))
        result = self.invoke_reasoning(self.user_input)
        while not result:
            self.get_logger().warn('Waiting for result from AI assistant')
            time.sleep(0.1)
        self.process_result(result)
        
    def send_user_input_to_agent(self):
        return self.user_input
        
    def process_result(self, result):
        if result is None:
            self.get_logger().warn('Failed to get a response from the AI Agent')
        elif result.get("execution_summary"):
            self.get_logger().info(result["execution_summary"])

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