import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Transform, Vector3Stamped
import tf2_ros
import numpy as np
from geometry_msgs.msg import Vector3
from tf2_ros import TransformListener, Buffer

class ManipulatorControlNode(Node):
    def __init__(self):
        super().__init__('manipulator_control_node')
        self.publisher_ = self.create_publisher(Vector3, 'manipulator_target', 10)
        self.subscription = self.create_subscription(
            Vector3,
            'camera_vector',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def listener_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform('robot_base', 'camera_base', rclpy.time.Time())
            transformed_vector = self.transform_vector(msg, transform)
            self.publisher_.publish(transformed_vector)
        except Exception as e:
            self.get_logger().warn('Could not transform vector: {0}'.format(e))

    def transform_vector(self, vector, transform):
        # Convert the Transform message to a 4x4 transformation matrix
        transform_matrix = tf2_ros.transform_to_kdl(transform)

        # Create a 4x1 homogeneous vector for the incoming vector
        vector_homogeneous = np.array([vector.x, vector.y, vector.z, 1])

        # Perform the transformation
        transformed_vector_homogeneous = np.dot(transform_matrix, vector_homogeneous)

        # Convert the result back to a Vector3 message
        transformed_vector = Vector3()
        transformed_vector.x = transformed_vector_homogeneous[0]
        transformed_vector.y = transformed_vector_homogeneous[1]
        transformed_vector.z = transformed_vector_homogeneous[2]

        return transformed_vector

def main(args=None):
    rclpy.init(args=args)

    manipulator_control_node = ManipulatorControlNode()

    rclpy.spin(manipulator_control_node)

    manipulator_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
