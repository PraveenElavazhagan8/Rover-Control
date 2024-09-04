#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from collections import defaultdict
from ultralytics import YOLO
import time

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, "/camera_publisher/image_raw", self.image_callback, 10)
        self.depth_pub = self.create_subscription(Image, "/camera_publisher/depth_pub", self.Depth_image_callback, 10)
        self.depth_raw_pub = self.create_subscription(Image, "/camera_publisher/depth_raw", self.Depth_raw_image_callback, 10)
        self.current_image = None
        self.current_Depth_image = None
        self.current_Depth_raw_image = None
        self.Model = YOLO("yolov5n.pt").to('cpu') 

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def Depth_image_callback(self, msg):
        self.current_Depth_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def Depth_raw_image_callback(self, msg):
        self.current_Depth_raw_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")


    def track_objects(self):
        if self.current_image is not None and self.current_Depth_raw_image is not None:
            # Perform object detection
            results = self.Model.track(self.current_image)
            boxes = results[0].boxes.xywh.cpu()

            # labels = results[0].names[results.pred[0][:, -1].int().cpu().numpy()]

            if results[0].boxes.id is None:
                return 0
            track_ids = results[0].boxes.id.int().cpu().tolist()
            annotated_frame = results[0].plot()
            # Show frame
            try:
                cv2.imshow('frame', self.current_Depth_image)
            except:
                pass
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return(0)
            

            for box in boxes:
                x, y, w, h = box
                # Convert color frame coordinates to depth frame coordinates
                depth_x = int((x + w )/ 2) #*  self.current_Depth_raw_image.shape[1] / self.current_image.shape[1])
                depth_y = int((y + h )/ 2) #*  self.current_Depth_raw_image.shape[0] / self.current_image.shape[0])

                # TOdo to check the pixel size is with in bound and then have to update the depth value
                # Todo  check and remove objects which are moved out of the bound so the object list will not have a build up of memeory
                # Get depth at the center of the bounding box
                depth = self.current_Depth_raw_image[depth_y, depth_x]/1000
                #change it into camera space point.
                # track.append((int(x + w // 2), int(y + h // 2), depth))  # x, y center point
            
                # Append the depth value text in the frame
                cv2.putText(self.current_image, f"Depth: {depth}", (depth_x, depth_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("Object Tracking", self.current_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return(0)


def main(args=None):
    rclpy.init(args=args)
    ot = ObjectTracker()
    rclpy.spin(ot)
    ot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
