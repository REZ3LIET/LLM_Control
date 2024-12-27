import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class VLMImageHandler(Node):
    def __init__(self):
        super.__init__("vlm_image_handler")
        self.subscription = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.listener_callback, 
            10
        )
        self.subscription
        self.br = CvBridge()
        self.captured_image = None

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving feed')
    
        # Convert ROS Image message to OpenCV image
        try:
            current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        self.captured_image = current_frame

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    # Create the node
    image_subscriber = VLMImageHandler()
    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
