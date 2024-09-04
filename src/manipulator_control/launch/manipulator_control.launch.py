from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='machine_vision',
        #     executable='camera_publisher',
        #     name='camera_publisher'
        # ),
        # Node(
        #     package='machine_vision',
        #     executable='object_detector',
        #     name='object_detector'
        # ),
        Node(
            package='machine_vision',
            executable='picam_publisher',
            name='picam_publisher'
        )
        # Node(
        #     package='manipulator_control',
        #     executable='manipulator_motionplanner',
        #     name='manipulator_motionplanner'
        # ),
        # Node(
        #     package='manipulator_control',
        #     executable='kinematics_control',
        #     name='kinematics_control'
        # ),
        # Node(
        #     package='manipulator_control',
        #     executable='manipulator_mover',
        #     name='manipulator_mover'
        # ),
        
        # Node(
        #     package='rover_control',
        #     executable='TeleOp_Loco_Arm',
        #     name='TeleOp_Loco_Arm'
        # )
        
    ])
