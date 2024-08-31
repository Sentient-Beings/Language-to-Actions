import rclpy
from rclpy.node import Node
from agent_interfaces.srv import GetSensorDistances
from std_msgs.msg import String
from .graph_util import run_graph , set_sensor_data_getter
import time

####################################################################################################################
############################################## ROS2 COMM Class #####################################################
####################################################################################################################
class Agent(Node):
    def __init__(self):
        super().__init__('agent')
        self.declare_parameter('user_input', '')
        self.create_subscription(String, 'user_input', self.user_input_callback, 10)
        self.sensor_client_ = self.create_client(GetSensorDistances, 'get_saftey_distance')
        self.lidar_sensor_reading = {}
        self.sensor_response = None
        self.get_logger().info('Agent initialized')
    
    def get_minimum_distance(self, request):
        self.get_logger().info('Requesting minimum distance from sensor')
        while not self.sensor_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting again...')
        future = self.sensor_client_.call_async(request) 
        future.add_done_callback(self.future_callback)
        self.get_logger().debug('Async service call made')

    def future_callback(self, future):
        self.get_logger().info('Future callback called')    
        try:
            self.sensor_response = future.result()
            self.get_logger().info('Received sensor response')
        except Exception as e:
            self.get_logger().error('Service call failed: %s', str(e))
            self.sensor_response = None

    def get_laser_scan_data(self):
        if not self.lidar_sensor_reading:
            self.get_logger().warn('Lidar sensor reading is empty or None')
            return None
        self.get_logger().debug('Returning lidar sensor reading')
        return self.lidar_sensor_reading  # Return the original, since we're only reading it
    
    def invoke_common_sense(self, user_input: str):
        try:
            request = GetSensorDistances.Request()
            self.sensor_response = None  # Reset the response
            self.get_minimum_distance(request)
            
            self.get_logger().info('Waiting for sensor response')
            while self.sensor_response is None:
                time.sleep(0.1)  # Short sleep to avoid busy waiting
            
            self.lidar_sensor_reading = {
                "min_distance_right": self.sensor_response.min_distance_right,
                "min_distance_left": self.sensor_response.min_distance_left,
                "min_distance_front": self.sensor_response.min_distance_front
            }
            self.get_logger().info('Lidar sensor reading updated: {}'.format(self.lidar_sensor_reading))
            
            set_sensor_data_getter(self.get_laser_scan_data)    
            self.get_logger().info('Running graph with user input')
            return run_graph(user_input)
        except Exception as e:
            self.get_logger().error('Error in invoke_common_sense: {}'.format(e))
            return None
        finally:
            self.sensor_response = None  # Reset for the next invocation

    def user_input_callback(self, msg):
        user_input = msg.data
        self.get_logger().info('Received user input: {}'.format(user_input))
        self.get_logger().info('Invoking common sense with user input')
        result = self.invoke_common_sense(user_input)
        while not result:
            self.get_logger().info('Waiting for result from AI assistant')
            time.sleep(0.1)
        self.process_result(result)

    def process_result(self, result):
        if result is None:
            self.get_logger().warn('Failed to get a response from the AI assistant')
            print("Failed to get a response from the AI assistant")
        elif result.get("final_answer"):
            self.get_logger().info('Received final answer from AI assistant')
            print("Agent Response:", result["final_answer"])
        else:
            self.get_logger().warn('AI assistant did not provide a final answer')
            print("AI assistant did not provide a final answer")

def main(args=None):
    rclpy.init(args=args)
    agent = Agent()
    
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Keyboard interrupt received, shutting down')
    finally:
        agent.get_logger().info('Destroying node and shutting down')
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()