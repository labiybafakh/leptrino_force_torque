from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare the 'com_port' launch argument
    com_port_arg = DeclareLaunchArgument('com_port', default_value='/dev/ttyACM0', description='Serial port to connect to')

    # Use the LaunchConfiguration to get the value of 'com_port'
    com_port_param = LaunchConfiguration('com_port')

    # Create the 'leptrino_node' node
    leptrino_node = Node(
        package='leptrino_force_torque',
        executable='leptrino_force_torque',
        parameters=[{'com_port': com_port_param}],
        output='screen'
    )

    return LaunchDescription([
        com_port_arg,
        leptrino_node
    ])
