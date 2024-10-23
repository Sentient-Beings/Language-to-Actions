#TODO: This is a temporary server to get the image from the camera
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# from agent_interfaces.srv import GetImage

# class ImageServer(Node):
#     def __init__(self):
#         super().__init__('image_server')
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/image_raw',
#             self.image_callback,
#             10)
#         self.srv = self.create_service(GetImage, 'get_image', self.get_image_callback)
#         self.bridge = CvBridge()
#         self.latest_image = None

#     def image_callback(self, msg):
#         self.latest_image = msg

#     def get_image_callback(self, request, response):
#         if self.latest_image is not None:
#             response.image = self.latest_image
#             self.get_logger().info('Sending image to client')
#         else:
#             self.get_logger().warn('No image available')
#         return response

# def main(args=None):
#     rclpy.init(args=args)
#     image_server = ImageServer()
#     rclpy.spin(image_server)
#     image_server.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

