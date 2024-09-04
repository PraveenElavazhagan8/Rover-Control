import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import math
from std_msgs.msg import String, Float64MultiArray
import numpy as np
import time

class XboxControllerNode(Node):
    def __init__(self):
        super().__init__('xbox360_controller')
        self.buttons = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,]
        self.axis = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.loco_mode = True
        self.Px, self.Py, self.Pz = 27, 0, 22  # Initial end-effector position
        self.RR,self.FR,self.RL,self.FL = [None,None,None,None] # wheels
        self.Theta1, self.Theta2, self.Theta3 = [-5, 27, 9]
        self.slow = False
        self.speed = 5
        self.subscription = self.create_subscription(
            Joy,
            'joystick_publisher',
            self.joy_callback,
            10)
        self.publisher_ = self.create_publisher(Float64MultiArray, 'servo_mover', 10)
        self.timer_period = 0.05  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def joy_callback(self, data):
        self.buttons = data.buttons
        self.axis = data.axes
        self.axis = np.asfarray(self.axis)
        self.buttons = np.asarray(self.buttons)

    def inv_kin(self, Px=27, Py=0, Pz=10):
        l1 = 5
        l2 = 16
        l3 = 17
        # Theta 1
        Theta1 = math.degrees(math.atan2(Py, Px))

        # Theta 2
        f = math.sqrt(Px**2 + (Pz - l1)**2)
        alpha = math.degrees(math.acos((f**2 + l2**2 - l3**2) / (2*f*l2)))
        beta = math.degrees(math.atan2(Pz-5, Px))
        Theta2 = beta - alpha

        # Theta 3
        c3 = (f**2 - l2**2 - l3**2) / (2*l2*l3)
        s3 = math.sqrt(1 - c3**2)
        Theta3 = math.degrees(math.atan2(s3, c3))
        return Theta1, Theta2, Theta3

    def locomotion(self):
        RT = self.axis[4]
        LT = self.axis[5]
        L_stick = self.axis[0]
        up_down_key = self.buttons[9]

        if(up_down_key == 1.0): #up D-pad
            self.speed+=1
            print("speed: " ,self.speed)
            if self.speed  >=9:
                self.speed = 9
            time.sleep(0.5)
        if(up_down_key == -1.0): #down D-pad
            self.speed -=1
            print("speed: " ,self.speed)
            if self.speed <= 2:
                self.speed = 2
            time.sleep(0.5)
        print("RT: " ,RT)
        print("LT: " ,LT)
        if(RT > -0.5):
            print("moving forward")
            self.RL =  90 + (self.speed * 10)
            self.FL =  90 + (self.speed * 10)
            self.FR =  90 - (self.speed * 10)
            self.RR =  90 - (self.speed * 10)
        elif(LT > -0.5):
            print("moving backward")
            self.RL = 90 - (self.speed * 10)
            self.FL = 90 - (self.speed * 10)
            self.FR = 90 + (self.speed * 10)
            self.RR = 90 + (self.speed * 10)
        elif L_stick >=0.8:
            self.RL =  90 - (self.speed * 10)
            self.FL =  90 - (self.speed * 10)
            self.FR =  90 - (self.speed * 10)
            self.RR =  90 - (self.speed * 10)
        elif L_stick <=-0.8:
            self.RL =   90 + (self.speed * 10)
            self.FL =   90 + (self.speed * 10)
            self.FR =   90 + (self.speed * 10)
            self.RR =   90 + (self.speed * 10)
        else:
            self.RL = 90 
            self.FL = 90
            self.FR = 90
            self.RR = 90

    def manipulator(self):
        global Px, Py, Pz
        global  Theta1, Theta2, Theta3
        # print(Theta1, Theta2, Theta3)
        increment_step_size = 0.25
        if self.buttons[6] == 1: #LB
            Pz = Pz - increment_step_size
            print("Reducing Pz:", Pz)
            # time.sleep(0.05)
        elif self.buttons[7] == 1: #RB
            Pz = Pz + increment_step_size
            print("Increasing Pz:", Pz)
            # time.sleep(0.05)
        if self.buttons[0] == 1: #A
            Px = Px - increment_step_size
            print("Reducing Px:", Px)
            # buttons[0] = 0
        elif self.buttons[4] == 1: #Y
            Px = Px + increment_step_size
            print("Increasing Px:", Px)
            # buttons[3] = 0
        if self.buttons[1] == 1: #B
            Py = Py - increment_step_size
            print("Reducing Py:", Py)
            # buttons[2] = 0
        elif self.buttons[3] == 1: #X
            Py = Py + increment_step_size
            print("Increasing Py:", Py)
            # buttons[1] = 0            int( )
        if self.buttons[8] == 1: #Left click
            print("button 8:", self.buttons[8] )
        elif self.buttons[10] == 1: #Right click
            Px, Py, Pz = 27, 0, 22
        #     print("Going Home")
        # if buttons[6] == 1: #Orient Ccw - back
        #     Theta4 = Theta4 - 1
        #     print("Orient CCW: ", Theta4)
        # elif buttons[7] == 1: #Orient Cw - start
        #     Theta4 = Theta4 + 1
        #     print("Orient CW: ", Theta4)
        else:
            pass
        try:
            Theta1, Theta2, Theta3 = self.inv_kin(Px=Px, Py=Py, Pz=Pz)
            # return Theta1, Theta2, Theta3, Theta4
        except:
            pass
            print("No Solution")

    def timer_callback(self):
        power = self.buttons[16]
        if(power==1):
            time.sleep(2)
            if not self.loco_mode:
                print("Switching LocoMode")
                self.loco_mode=True
            else:
                print("Switching Manipulator mode")
                self.loco_mode=False

        if not self.loco_mode:
            self.manipulator()
        else:
            self.locomotion()
        msg = Float64MultiArray()
        scale = 180 / 180
        servo_offset1, servo_offset2, servo_offset3 = 95, 90, 90
        if(self.loco_mode):
            msg.data = [
                self.RL,
                self.FL,
                self.FR,
                self.RR, 
                
                ]
        else:
            msg.data = [
                self.RL,
                self.FL,
                self.FR,
                self.RR, 
                (scale * (self.Theta1 + servo_offset1)),
                (scale * (self.Theta2 + servo_offset2)),
                (scale * (self.Theta3 + servo_offset3)),
                ]
        print(f"servo message: { msg.data}" )
        if msg is not None:
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    xbox_controller_node = XboxControllerNode()
    rclpy.spin(xbox_controller_node)
    xbox_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
