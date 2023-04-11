from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(package='time_sync',
            executable='time_sync',
            output='screen'),
        Node(package='orbslam3', 
             executable='mono-inertial', 
             output='screen',
             arguments=['/home/simen/git/project_autonomous/colcon_ws/src/ORB_SLAM3_ROS2/vocabulary/ORBvoc.txt', \
                        '/home/simen/git/project_autonomous/colcon_ws/src/ORB_SLAM3_ROS2/config/monocular-inertial/esp32cam.yaml'])])
