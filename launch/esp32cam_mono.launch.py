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
             output='screen'),
        Node(package='time_sync',
            executable='time_sync',
            output='screen'),
        Node(package='aruco_detector',
             executable='aruco_detector',
             output='screen'),
        Node(package='tello_control',
            executable='tello_control',
            output='screen'),
        Node(package='orbslam3', 
             executable='mono', 
             output='screen',
             arguments=['/home/simen/git/project_autonomous/colcon_ws/src/ORB_SLAM3_ROS2/vocabulary/ORBvoc.txt', \
                        '/home/simen/git/project_autonomous/colcon_ws/src/ORB_SLAM3_ROS2/config/monocular/esp32cam.yaml'])
        ])
