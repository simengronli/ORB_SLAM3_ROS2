from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(package='joy', 
             executable='joy_node', 
             parameters=[{'deadzone': 0.2}],
             output='screen'),

        Node(package='tello_driver', 
             executable='tello_joy_main', 
             output='screen'),

        Node(package='tello_driver', 
             executable='tello_driver_main', 
             output='screen', 
             remappings=[('/image_raw','/tello/camera/image_raw')]),
     # Launch node that plays a bag file
     #    ExecuteProcess(
     #         cmd=['ros2', 
     #              'bag', 
     #              'play', 
     #              '/home/isaac/Documents/ros2bag_tello/rosbag2_2023_03_08-13_52_10']),
            

        TimerAction(period=5.0, actions=[
     #    Node(package='tello_control',
     #         executable='tello_control',
     #         output='screen'),
     #    Node(package='tello_control',
     #         executable='tello_dynamics',
     #         output='screen'),

        Node(package='orbslam3', 
             executable='mono', 
             output='screen',
             arguments=['/home/simen/git/project_autonomous/colcon_ws/src/ORB_SLAM3_ROS2/vocabulary/ORBvoc.txt', \
                        '/home/simen/git/project_autonomous/colcon_ws/src/ORB_SLAM3_ROS2/config/monocular/TELLO.yaml'])])

    ])