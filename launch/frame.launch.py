from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():    # a launcher for the chatbot's input and output
    stretch_core_path = get_package_share_directory('stretch_core')
    
    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'position', 'broadcast_odom_tf': 'True'}.items())

    # launch camera
    d435i_launch = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               stretch_core_path, 'launch'),
               '/stretch_realsense.launch.py'])
          )

    turn_to_frame = Node(
            package='rp_household_tasks',
            executable='turn_to_frame',
            output='screen',
            parameters=[{"frame": 'base_link'}] # can be any frame
        )

    stretch_aruco = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               stretch_core_path, 'launch'),
               '/stretch_aruco.launch.py'])
          )
    
    rviz_config_path = os.path.join(stretch_core_path, 'rviz', 'stretch_simple_test.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        )


    
    return LaunchDescription([
        #say that 5 times in a row
        stretch_driver_launch,
        turn_to_frame,
        # d435i_launch,
        # stretch_aruco,
        # rviz_node
    ])