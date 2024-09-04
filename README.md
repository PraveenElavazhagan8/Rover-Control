# Mini Rover with 3DOF Arm and Machine Vision

## Overview

This project features a mini rover equipped with four wheels and a 3DOF robotic arm, all actuated by servo motors. The wheels are 360-degree motors, while the manipulator uses 180-degree servos. The system is powered by a Raspberry Pi 4 (8GB) board with a 16-channel Servo PiHat servo controller. Additional hardware includes a Raspberry Pi Camera, an Intel RealSense 435i depth camera, and two PWM relays. These relays enable a 6V DC motor for a drill bit at the end of the robotic arm and control an LED light for low-light conditions.

## Hardware

- **Raspberry Pi 4 (8GB)**
- **16-Channel Servo PiHat**
- **Raspberry Pi Camera**
- **Intel RealSense 435i Depth Camera**
- **4 x 360-degree Servo Motors (Wheels)**
- **3 x 180-degree Servo Motors (Manipulator)**
- **2 x PWM Relay Modules**
  - 6V DC motor for drill bit (end of the robotic arm)
  - LED light for low-light conditions

## Software

The software is organized into three main packages:

### 1. Machine Vision Package
This package handles all vision-related tasks.

- **Nodes:**
  1. **Pi Camera Reader and Publisher:** Captures and publishes images from the Raspberry Pi Camera.
  2. **Intel RealSense Camera Node:** Publishes 2D images and 3D depth matrices from the RealSense camera.
  3. **Object Detection Node:** Uses YOLOv8 to detect objects in the 2D frame, find the center of the object, and publish this data. (Note: A condition must be added to check for required classes from the detected objects before finding and publishing the center.)

### 2. Manipulator Control Package
This package manages the motion and control of the 3DOF manipulator.

- **Nodes:**
  1. **Manipulator Motion Planner:** Subscribes to the object detection node's target centre message, converts it into robot space, and publishes the result.
  2. **Kinematics Control:** Subscribes to the robot-space target and computes the inverse kinematics (IK) and forward kinematics (FK) of the manipulator.
  3. **Manipulator Mover:** Subscribes to the movement message and moves the servo motors to the desired position using the Servo PiHat controller.

### 3. Rover Control Package
This package is responsible for controlling the rover's movement.

- **Node:**
  - **Teleop Control:** Connects with a remote client node that publishes joystick control messages. It sends the appropriate commands to move the rover and, based on the mode, allows manual or semi-autonomous control using live feed data from the cameras.

### Semi-Autonomous Mode
- Controlled via joystick.
- Allows for both manual movement and semi-autonomous operations using data from the Intel RealSense camera.

### Manual Mode
- Provides direct control over the rover and the manipulator using the joystick.
- Button functions on the joystick can be customized according to user preferences for flexibility and ease of use.

## Installation and Setup

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/yourusername/rover-control.git
   cd rover-src
   ```

2. **Install Dependencies:**
   - Install the required packages and libraries for Raspberry Pi, Servo PiHat, YOLOv8, and Intel RealSense.
   
3. **Launch Files:**
   - The system is configured using a launch file that initiates the control for the manipulator. 
   - Run the launch file to start all the nodes simultaneously:
     ```bash
     roslaunch manipulator_control manipulator_control.launch
     ```

## Usage

- **Joystick Controls:**
  - Customize the joystick button functions according to your preference for manual and semi-autonomous modes.
  
- **Switching Between Modes:**
  - Semi-autonomous and manual modes can be toggled depending on the situation and user preference.

## Contributions

Contributions, issues, and feature requests are welcome. Feel free to check the [issues page](https://github.com/yourusername/mini-rover-project/issues).


---

*Developed with ❤️ by [Praveen Elavazhagan]
