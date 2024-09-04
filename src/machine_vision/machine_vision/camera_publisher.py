# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import pyrealsense2 as rs
# import numpy as np
# import cv2

# class CameraPublisher(Node):
#     def __init__(self):
#         super().__init__('camera_publisher')
#         self.bridge = CvBridge()
#         self.image_pub = self.create_publisher(Image, "/camera_publisher/image_raw", 10)
#         self.depth_raw_pub = self.create_publisher(Image, "/camera_publisher/depth_raw", 10)
#         self.depth_pub = self.create_publisher(Image, "/camera_publisher/depth_pub", 10)

#         self.pipeline = rs.pipeline()
#         config = rs.config()
#         config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
#         config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 60)
#         self.pipeline.start(config)

#     def publish_images(self):
#         while rclpy.ok():
#             self.get_logger().info('into camera')
            
#             frames = self.pipeline.wait_for_frames()
#             depth_frame = frames.get_depth_frame()
#             depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(np.asanyarray(depth_frame.get_data()), alpha=0.03), cv2.COLORMAP_JET)
#             color_frame = frames.get_color_frame()
#             if not color_frame:
#                 continue
#             color_image = np.asanyarray(color_frame.get_data())
#             depth_image = np.asanyarray(depth_frame.get_data())        
#             image_message = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
#             depthImageMessage = self.bridge.cv2_to_imgmsg(depth_colormap, "bgr8")
#             depthRawMessage = self.bridge.cv2_to_imgmsg(depth_image, "16UC1")  # 16-bit unsigned single-channel image
#             self.image_pub.publish(image_message)
#             self.depth_pub.publish(depthImageMessage)
#             self.depth_raw_pub.publish(depthRawMessage)
#             rclpy.spin_once(self)

# def main(args=None):
#     rclpy.init(args=args)
#     cp = CameraPublisher()
#     cp.publish_images()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, "/camera_publisher/image_raw", 10)
        self.depth_raw_pub = self.create_publisher(Image, "/camera_publisher/depth_raw", 10)
        self.depth_pub = self.create_publisher(Image, "/camera_publisher/depth_pub", 10)

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        # config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.pipeline.start(config)

    def publish_images(self):
        i = 0
        try:
            while rclpy.ok():
                self.get_logger().info('into camera')
                
                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                if not depth_frame :
                    continue
                depth_image = np.asanyarray(depth_frame.get_data())  
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                cv2.imshow("Object Tracking", depth_image)
                
                color_frame = frames.get_color_frame()
                if not color_frame :
                    continue
                color_image = np.asanyarray(color_frame.get_data())
                      
                image_message = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
                depthImageMessage = self.bridge.cv2_to_imgmsg(depth_colormap, "bgr8")
                depthRawMessage = self.bridge.cv2_to_imgmsg(depth_image, "16UC1")  # 16-bit unsigned single-channel image
                self.image_pub.publish(image_message)
                self.depth_pub.publish(depthImageMessage)
                self.depth_raw_pub.publish(depthRawMessage)
                rclpy.spin_once(self)
                i += 1
        except Exception as e:
            self.get_logger().error('exception camera {}'.format(e))
        finally:
            pass

def main(args=None):
    rclpy.init(args=args)
    cp = CameraPublisher()
    cp.publish_images()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
