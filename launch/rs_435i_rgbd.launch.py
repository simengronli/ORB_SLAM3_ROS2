from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import FindExecutable

def generate_launch_description():
    return LaunchDescription([
     # Launch node that plays a bag file
     #    ExecuteProcess(
     #         cmd=['ros2', 
     #              'bag', 
     #              'play', 
     #              '/home/isaac/Documents/ros2bag_tello/rosbag2_2023_03_08-13_52_10']),

        # Run the realsense launch file
        # ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=linear_interpolation enable_sync:=true enable_pointcloud:=false enable_fisheye:=false enable_infra1:=true enable_infra2:=true enable_color:=true
   #   ExecuteProcess(
   #   cmd=[["ros2",
   #        'launch',
   #        'realsense2_camera',
   #        'rs_launch.py',
   #        'enable_gyro:=true',
   #        'enable_accel:=true',
   #        'unite_imu_method:=linear_interpolation',
   #        'enable_sync:=true',
   #        'enable_pointcloud:=false',
   #        'enable_fisheye:=false',
   #        'enable_infra1:=true',
   #        'enable_infra2:=true',
   #        'enable_color:=true',]]),

         # Run the realsense node
         Node(package='realsense2_camera',
               executable='realsense2_camera_node',
               output='screen',
               parameters=[{'enable_gyro': True, 
                            'enable_accel': True, 
                            'unite_imu_method': 2, 
                            'enable_sync': True, 
                            'enable_pointcloud': False, 
                            'enable_fisheye': False, 
                            'enable_infra1': True, 
                            'enable_infra2': True, 
                            'enable_color': True,
                            'emitter_enabled': 0}]),


          

     TimerAction(period=2.0, actions=[
        Node(package='orbslam3', 
             executable='rgbd', 
             output='screen',
            #  remappings=[('/infra1/image_rect_raw','/camera/left'),('infra2/image_rect_raw','/camera/right' ), ('/imu','/imu')],
            #  remappings=[('/camera/left', '/camera/infra1/image_rect_raw'), ('/camera/right', '/camera/infra2/image_rect_raw'), ('/imu', '/imu')],
             arguments=['/home/simen/git/project_autonomous/colcon_ws/src/ORB_SLAM3_ROS2/vocabulary/ORBvoc.txt', \
                        '/home/simen/git/project_autonomous/colcon_ws/src/ORB_SLAM3_ROS2/config/rgb-d/RealSense_D435i.yaml'])])
     ])
