#TODO: Following script aims to separate the image processing from the agent and provide it as a service to the agent
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# from agent_interfaces.srv import GetImage
# import cv2
# import base64
# import json
# import os
# from groq import Groq
# GROQ_API_KEY = os.getenv("GROQ_API_KEY")

# if not GROQ_API_KEY:
#     raise ValueError("GROQ_API_KEY environment variable is not set")

# class ImageServer(Node):
#     def __init__(self):
#         super().__init__('image_server')
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/image_raw',
#             self.camera_callback,
#             10)
#         # self.srv = self.create_service(GetImage, 'get_image', self.get_image_callback)
#         self.bridge = CvBridge()
#         self.client = Groq()
#         self.base64_image = None

#     def camera_callback(self, msg):
#         """Callback for the camera image"""
#         try:
#             self.get_logger().info('Received image from camera')
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#             _, buffer = cv2.imencode('.jpg', cv_image)
#             # Encode the image to base64 -> this will be used by the agent
#             self.base64_image = self.encode_image(buffer.tobytes())
#         except Exception as e:
#             self.get_logger().error(f'Error processing camera image: {str(e)}')

#     def encode_image(self, img_bytes):
#         """Encode image bytes to base64 string."""
#         self.get_logger().info('Encoding image to base64')
#         return base64.b64encode(img_bytes).decode('utf-8')
    
#     def create_prompt(self,lidar_data):
#         """Create a structured prompt combining vision and LiDAR data."""
#         self.get_logger().info('Creating prompt')
#         prompt = f"""Analyze this image from the robot's front-facing camera in accordance with the provided LiDAR data.
#         Return a JSON object describing the scene and distance to the nearest obstacle

#         LiDAR readings which provide distances to obstacles near the robot. 
#         - Distance to obstacle in front right: {lidar_data['front_right']:.2f} meters
#         - Distance to obstacle in front left: {lidar_data['front_left']:.2f} meters
#         - Distance to obstacle in front: {lidar_data['front']:.2f} meters

#         You must return a JSON object with this structure:
#         {{
#             "scene_description": {{
#                 "description": "describe the scene in a way that helps in planning the motion of the robot and avoid hitting the obstacles",
#                 "obstacles": "list the nearest obstacles with their positions and distances from the robot"
#             }}
#         }}
#         Following is just an example of a valid JSON output: 
#         Example scenario:
#         Image shows: A room with two cardboard boxes - a red box directly in front and a blue box far away on the right
#         LiDAR readings:
#         - Front right: 5.20 meters
#         - Front left: 4.80 meters
#         - Front: 1.50 meters

#         Output:
#         Example response:
#         {{
#             "scene_description": {{
#                 "description": "The robot's path is partially blocked by a red cardboard box directly in front. There is more open space on the right side of the room",
#                 "obstacles": "Nearest obstacle is a red cardboard box at 1.50 meters directly in front of the robot. Another box visible on the right but at a safe distance of 5.20 meters"
#             }}
#         }}

#         Important:
#         - Focus on obstacles that could affect the robot's movement
#         - Use actual distances from the LiDAR data provided
#         - Describe positions of obstacles (front, left, right) relative to the robot
#         - Keep descriptions concise but include critical navigation details
#         - Return valid JSON format only
#         """
#         return prompt
    
#     def get_lidar_data(self):
#         """Generate simulated LiDAR readings."""
#         lidar_readings = {
#             "front_right": 5.7,
#             "front_left": 3.5,
#             "front": 3
#         }
#         return lidar_readings

#     def analyze_scene(self):
#         """Analyze the scene using both vision and LiDAR data with JSON output."""
#         try:
#             self.get_logger().info('Getting lidar data')
#             # update the lidar data
#             lidar_data = self.get_lidar_data()
            
#             # Safety check: See if there is no image or lidar data
#             if not self.base64_image and not lidar_data:
#                 self.get_logger().warn('No scene observation, STOP the robot')
#                 return {"error": "No scene observation, STOP the robot"}
            
#             prompt = self.create_prompt(lidar_data)
#             response = self.client.chat.completions.create(
#                 messages=[
#                     {
#                         "role": "user",
#                         "content": [
#                             {"type": "text", "text": prompt},
#                             {
#                                 "type": "image_url",
#                                 "image_url": {
#                                     "url": f"data:image/jpeg;base64,{self.base64_image}",
#                                 },
#                             },
#                         ],
#                     }
#                 ],
#                 model="llama-3.2-90b-vision-preview",
#                 response_format={"type": "json_object"}
#             )
#             self.get_logger().info(response.choices[0].message.content)
#             result = json.loads(response.choices[0].message.content)
#             return result
        
#         except json.JSONDecodeError as e:
#             return {"error": f"No scene observation, STOP the robot: {str(e)}"}
#         except Exception as e:
#             return {"error": f"No scene observation, STOP the robot: {str(e)}"}
        
#     def observation(self):
#         """Get the scene observation grounded in the real world using lidar data."""
#         output = self.analyze_scene()
#         if "error" in output:
#             scene_description = output["error"]
#             combined = f"WARNING!: {scene_description}"
#             return combined
#         scene_description = output["scene_description"]
#         combined = f" Scene description: {scene_description['description']}. Details about the obstacles: {scene_description['obstacles']}"
#         return combined

#     # def get_image_callback(self, request, response):
#     #     if self.latest_image is not None:
#     #         response.image = self.latest_image
#     #         self.get_logger().info('Sending image to client')
#     #     else:
#     #         self.get_logger().warn('No image available')
#     #     return response

# def main(args=None):
#     rclpy.init(args=args)
#     image_server = ImageServer()
#     rclpy.spin_once(image_server)
#     print(image_server.observation())
#     image_server.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

