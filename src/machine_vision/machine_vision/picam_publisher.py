# Import the necessary libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Define the CameraNode class
class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera_frame', 10)
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.bridge = CvBridge()

    def timer_callback(self):
        # Capture frame from camera
        cap = cv2.VideoCapture()
        ret, frame = cap.read()
        cap.release()

        # Convert the frame to a ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        cv2.imshow("current frame",frame)
        # Publish the image
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing an image')

# Define the main function
def main(args=None):
    rclpy.init(args=args)

    # Create an instance of CameraNode
    node = CameraNode()

    # Spin the node so it can send and receive messages
    rclpy.spin(node)

    # Shutdown and cleanup the node
    node.destroy_node()
    rclpy.shutdown()

# Call the main function
if __name__ == '__main__':
    main()
