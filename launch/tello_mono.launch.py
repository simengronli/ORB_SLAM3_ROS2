from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
     #    Node(package='joy', 
     #         executable='joy_node', 
     #         output='screen'),

     #    Node(package='tello_driver', 
     #         executable='tello_joy_main', 
     #         output='screen'),

     #    Node(package='tello_driver', 
     #         executable='tello_driver_main', 
     #         output='screen', 
     #         remappings=[('/image_raw','/tello/camera/image_raw')]),
     # Launch node that plays a bag file
        ExecuteProcess(
             cmd=['ros2', 
                  'bag', 
                  'play', 
                  '/home/simen/Documents/ros2bag_tello/april_grid/rosbag2_2023_03_09-16_15_29_0.db3']),

        Node(package='orbslam3', 
             executable='mono', 
             output='screen',
             arguments=['/home/simen/git/project_autonomous/colcon_ws/src/ORB_SLAM3_ROS2/vocabulary/ORBvoc.txt', \
                        '/home/simen/git/project_autonomous/colcon_ws/src/ORB_SLAM3_ROS2/config/monocular/TELLO.yaml'])

    ])