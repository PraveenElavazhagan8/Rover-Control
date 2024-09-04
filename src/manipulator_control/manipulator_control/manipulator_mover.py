import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from adafruit_servokit import ServoKit
import time

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        try:
            self.kit = ServoKit(channels=16)
           
            self.kit.servo[4].actuation_range =180
            self.kit.servo[5].actuation_range =180
            self.kit.servo[6].actuation_range =180

            self.kit.continuous_servo[0].set_pulse_width_range(500,2500)
            self.kit.continuous_servo[0].fraction = 0.5
            self.kit.continuous_servo[1].set_pulse_width_range(500,2500)
            self.kit.continuous_servo[1].fraction = 0.5 
            self.kit.continuous_servo[2].set_pulse_width_range(500,2500)
            self.kit.continuous_servo[2].fraction = 0.5
            self.kit.continuous_servo[3].set_pulse_width_range(500,2500)
            self.kit.continuous_servo[3].fraction = 0.5
            self.kit.servo[4].set_pulse_width_range(500,2500)
            self.kit.servo[5].set_pulse_width_range(500,2500)
            self.kit.servo[6].set_pulse_width_range(500,2500)

            self.kit.servo[4].angle =95
            self.kit.servo[5].angle =70
            self.kit.servo[6].angle =180

            self.subscription = self.create_subscription(
                Float64MultiArray,
                'servo_mover',
                self.listener_callback,
                10)
            self.subscription
        except Exception as e:
            self.get_logger().error('Error in initialization: {}'.format(e))

    def listener_callback(self, msg):
        try:
            joint_angles = msg.data
            self.get_logger().warn('Servo angles, got {}'.format(joint_angles))
            if len(joint_angles) > 7 and len(joint_angles) <3:
                self.get_logger().warn('Expected 3 or 7 joint angles, got {}'.format(len(joint_angles)))
                return
            for i in range(len(joint_angles)):
                j = i 
                
                
                self.get_logger().info('channel number {}'.format(j))
                self.get_logger().info('setting wheel throttle')
                if joint_angles[j] < 90:
                    self.kit.continuous_servo[j].throttle =  -1 + (joint_angles[j]/90)
                elif joint_angles[j] > 90:
                    self.kit.continuous_servo[j].throttle =  (joint_angles[j] -90)/90
                else:
                    self.kit.continuous_servo[j].throttle = 0
                    
                    self.kit.continuous_servo[j].fraction = 0.5
                
                self.get_logger().info('wheel throttle {}'.format( self.kit.continuous_servo[j].throttle))
                    
                if j > 4:
                                     
                    target_angle = joint_angles[j]
                    if target_angle < 0.0 or target_angle > 180.1:
                        self.get_logger().warn('Invalid target angle: {}. Angle should be between 0 and 180'.format(target_angle))
                        return

                    current_angle = self.kit.servo[j].angle
                    step_size = 1  # change this to control the speed of the servo
                    if current_angle < target_angle:
                        while current_angle < target_angle:
                            current_angle += step_size
                            if current_angle < 0.0 or current_angle > 179.0:
                                # self.get_logger().warn('Invalid current stepper increment angle: {}'.format(current_angle))
                                if(current_angle < 0.0):
                                    current_angle = 0.0
                                else:
                                    current_angle = 179
                            # self.get_logger().info('stepper angle: {}'.format(current_angle))
                            self.kit.servo[j].angle = current_angle
                            time.sleep(0.01)  # delay between steps
                    else:
                        while current_angle > target_angle:
                            current_angle -= step_size
                            if current_angle < 0.0 or current_angle > 179.0:
                                # self.get_logger().warn('Invalid current stepper decrement angle: {}'.format(current_angle))
                                if(current_angle < 0.0):
                                    current_angle = 0.0
                                else:
                                    current_angle = 179
                            # self.get_logger().info('stepper angle: {}'.format(current_angle))
                            self.kit.servo[j].angle = current_angle
                            time.sleep(0.01)  # delay between steps
                    
                    self.get_logger().info('moved angle: {}'.format(self.kit.servo[j].angle))
        except Exception as e:
            self.get_logger().error('An unexpected error occurred: {}'.format(e))
            

def main(args=None):
    try:
        rclpy.init(args=args)
        servo_controller = ServoController()
        rclpy.spin(servo_controller)
        servo_controller.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print('Error in main: {}'.format(e))

if __name__ == '__main__':
    main()
