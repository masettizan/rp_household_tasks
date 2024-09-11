from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():    # a launcher for the chatbot's input and output
    return LaunchDescription([
        #say that 5 times in a row
        Node(
            package='rp_household_tasks',
            namespace='rp_household_tasks',
            executable='turn_to_frame',
            name='turn_to_frame',
            output='screen',
            parameters=[{"frame": 'base_link'}] # can be “anthropic” or “openai”
        )
    ])