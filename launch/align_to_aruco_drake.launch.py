import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# launch file
def generate_launch_description():
    stretch_core_path = get_package_share_directory('stretch_core')

    stretch_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(stretch_core_path), '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'trajectory', 'broadcast_odom_tf': 'False', 'fail_out_of_range_goal': 'True'}.items(),
    )

    # launch camera
    d435i_launch = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               stretch_core_path, 'launch'),
               '/stretch_realsense.launch.py'])
          )

    stretch_aruco = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               stretch_core_path, 'launch'),
               '/stretch_aruco.launch.py'])
          )
    
    # <node pkg="tf2_ros" type="static_transform_publisher" name="link_head_to_static_camera" output="screen"
    #         args="0.14 0.1 -0.1 0 0 0 1 link_head static_camera_link"/>

    # tf2_ros = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='link_head_to_static_camera',
    #     arguments=[0.14, 0.1, -0.1, 0, 0, 0, 1, 'link_head', 'static_camera_link'],
    #     output='screen',
    # )

    # code to run 
    align_to_aruco = Node(
        package='camera_track_head',
        executable='align_to_aruco',
        output='screen',
        )

    rviz_config_path = os.path.join(stretch_core_path, 'rviz', 'stretch_simple_test.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        )

    return LaunchDescription([
        stretch_driver,
        d435i_launch,
        stretch_aruco,
        align_to_aruco,
        rviz_node,
        # tf2_ros
        ])