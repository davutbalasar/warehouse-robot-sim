# trash.launch.py

# cd ~/ros2_trash_ws
# ros2 launch trash_sim trash.launch.py

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trash_sim',
            executable='environment',
            name='environment_node',
            output='screen'
        ),
        Node(
            package='trash_sim',
            executable='robot',
            name='robot_node',
            output='screen'
        ),
        Node(
            package='trash_sim',
            executable='visualizer',
            name='visualizer_node',
            output='screen'
        ),
        Node(
            package='trash_sim',
            executable='rviz_publisher_node',
            name='rviz_publisher_node',
            output='screen'
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', '/home/davut/ros2_trash_ws/src/trash_sim/warehouse_robot.rviz'],
            output='screen'
        ),
    ])