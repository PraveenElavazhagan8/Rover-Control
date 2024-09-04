import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
from geometry_msgs.msg import Pose

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        self.subscription = self.create_subscription(
            Pose,
            'robot_target',
            self.listener_callback,
            10)
        self.a1 =5
        self.a2 =16
        self.a3 =16
        self.phi_deg_values = [0]#np.arange(-180, 180, self.deg)
        self.publisher = self.create_publisher(Float64MultiArray, 'joint_angles', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        target =  np.array([msg.position.x, msg.position.y, msg.position.z])
        joint_angles = self.calculate_inverse_kinematics(target)
        self.get_logger().info('Calculated joint angles: \"%s\"' % joint_angles)

        # Create a Float64MultiArray message for the joint angles
        joint_angles_msg = Float64MultiArray()
        joint_angles_msg.data = joint_angles

        # Publish the joint angles
        self.publisher.publish(joint_angles_msg)

    def calculate_inverse_kinematics(self, target):
        # Define the lengths of the robot's arms
        

        # Extract the target coordinates
        px, py, pz = target
        angles = []
        theta1 = math.degrees(math.atan2(py, px))
        f = math.sqrt(px**2+(pz-self.a1)**2)

        alpha = math.degrees(math.acos((f**2 + self.a2**2 - self.a3 **2) / (2*f*self.a2)))
        beta = math.degrees(math.atan2(pz-5,px))
        theta2 = beta-alpha

        #theta 3
        c3 = (f**2 - self.a2**2 - self.a3**2)/(2*f*self.a3)
        s3 = math.sqrt(1-c3**2)
        theta3 = math.degrees(math.atan2(s3,c3))

        
        angles= [theta1,theta2,theta3]
        self.get_logger().info('IK angles: {} {} {}'.format(theta1,theta2,theta3))

        # if(q1 > 0):
        #     q1 = q1 + 90
        # elif(q1 < 0):
        #     q1 = q1 + 90
        # else:
        #     q1 = q1

        
        # if(q3 > 0):
        #     q3 = q3 + 90
        # elif(q3<0):
        #     q3 = q3 +90
        # else:
        #     q3 = q3

        # if(q2 < 0 and q2 < -90):
        #     angles = [q1,round(-q2-90,2),q3]
        # elif(q2<0):
        #     angles = [q1,round(q2,2),q3]
        # else:
        #     angles = [q1,round(q2,2),q3]
        # Return the joint angles
        return angles
    
def inverse(self, coordinates, app):
        px, py, pz = coordinates
        self.coord = coordinates
        angles = []
        theta0 = math.atan2(py, px)
        px_projected = px * math.cos(theta0) + py * math.sin(theta0)
        px = px_projected

        for phi in self.phi_deg_values:
            phi = phi * np.pi / 180
            wx = px - (self.a3 * np.cos(phi))
            wz = pz - (self.a3 * np.sin(phi))

            c2 = (wx ** 2 + wz ** 2 - self.a1 ** 2 - self.a2 ** 2) / (2 * self.a1 * self.a2)

            if c2 <= 1:
                s2_1 = np.sqrt(1 - c2 ** 2)
                s2_2 = -np.sqrt(1 - c2 ** 2)
                theta2_1 = np.arctan2(s2_1, c2)
                theta2_2 = np.arctan2(s2_2, c2)

                denom_1 = self.a1 ** 2 + self.a2 ** 2 + 2 * self.a1 * self.a2 * np.cos(theta2_1)
                denom_2 = self.a1 ** 2 + self.a2 ** 2 + 2 * self.a1 * self.a2 * np.cos(theta2_2)
                s1_1 = (wz * (self.a1 + self.a2 * np.cos(theta2_1)) - self.a2 * np.sin(theta2_1) * wx) / denom_1
                s1_2 = (wz * (self.a1 + self.a2 * np.cos(theta2_2)) - self.a2 * np.sin(theta2_2) * wx) / denom_2
                c1_1 = (wx * (self.a1 + self.a2 * np.cos(theta2_1)) + self.a2 * np.sin(theta2_1) * wz) / denom_1
                c1_2 = (wx * (self.a1 + self.a2 * np.cos(theta2_2)) + self.a2 * np.sin(theta2_2) * wz) / denom_2
                theta1_1 = np.arctan2(s1_1, c1_1)
                theta1_2 = np.arctan2(s1_2, c1_2)

                theta3_1 = phi - theta1_1 - theta2_1
                theta3_2 = phi - theta1_2 - theta2_2

                sol_1 = [theta0, theta1_1, -theta2_1, -theta3_1 + (math.pi / 2)]  # Elbow Up
                sol_2 = [theta0, theta1_2, -theta2_2, -theta3_2 + (math.pi / 2)]  # Elbow Down

                angles.append(sol_1)
                angles.append(sol_2)
        return angles
def main(args=None):
    rclpy.init(args=args)

    inverse_kinematics_node = InverseKinematicsNode()

    rclpy.spin(inverse_kinematics_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    inverse_kinematics_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

