from launch import LaunchDescription
from launch_ros.actions import Node
import socket

hostname = socket.gethostname()

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'phoenix',
            namespace = hostname,
            executable = 'stream',
            name = 'phoenix_stream'
        ),
        Node(
            package = 'phoenix',
            namespace = hostname,
            executable = 'battery',
            name = 'phoenix_battery'
        ),
        Node(
            package = 'phoenix',
            namespace = hostname,
            executable = 'command',
            name = 'phoenix_command'
        )
    ])
