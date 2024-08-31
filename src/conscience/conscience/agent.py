import rclpy
from rclpy.node import Node
from agent_interfaces.srv import GetSensorDistances
from std_msgs.msg import String
from .graph_util import run_graph , set_sensor_data_getter


####################################################################################################################
############################################## ROS2 COMM Class #####################################################
####################################################################################################################
class Agent(Node):
    def __init__(self):
        super().__init__('agent')

        self.declare_parameter('user_input', 'What is the nearest distance to the object?')
        self.last_user_input = self.get_parameter('user_input').value
        # self.create_timer(1.0, self.check_user_input)
        self.sensor_client_ = self.create_client(GetSensorDistances, 'get_saftey_distance')
        self.lidar_sensor_reading = {}
        self.sensor_response = None
        self.get_logger().info('Agent initialized')
    
    def check_user_input(self):
        '''
        check if the user input has changed, if so, update the last_user_input
        '''
        current_user_input = self.get_parameter('user_input').value
        if current_user_input != self.last_user_input:
            self.last_user_input = current_user_input

    def get_minimum_distance(self, request):
        '''
        client function to get the minimum distance from the sensor
        '''
        while not self.sensor_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.sensor_client_.call_async(request) 
        future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        try:
            self.sensor_response = future.result()
        except Exception as e:
            self.get_logger().info(f'Error: {e}')
    
    def get_laser_scan_data(self):
        if self.lidar_sensor_reading is not None:
            return self.lidar_sensor_reading
    
    def invoke_common_sense(self, user_input:str):
        request = GetSensorDistances.Request()
        while True:
            self.get_minimum_distance(request)
            if self.sensor_response is None:
                self.get_logger().info('No response from the server')
                continue
            self.lidar_sensor_reading["min_distance_right"] = self.sensor_response.min_distance_right
            self.lidar_sensor_reading["min_distance_left"] = self.sensor_response.min_distance_left
            self.lidar_sensor_reading["min_distance_front"] = self.sensor_response.min_distance_front
            break 
        set_sensor_data_getter(self.get_laser_scan_data)
        return run_graph(user_input)


def main(args=None):
    rclpy.init(args=args)
    agent = Agent()
    
    while rclpy.ok():
        rclpy.spin_once(agent) 
        user_input = input("where is the closest object? and what is the distance to it? ")
        if (user_input == 'exit'):
            break
        result = agent.invoke_common_sense(user_input)
        if result.get("final_answer"):
            print("Agent Response :", result["final_answer"])
        else:
            print("No Observation")

    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()