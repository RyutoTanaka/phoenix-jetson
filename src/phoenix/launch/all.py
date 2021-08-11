from launch import LaunchDescription
from launch_ros.actions import Node
import socket
import re

namespace = re.sub("[^a-zA-Z0-9]", "_", socket.gethostname()) + "/phoenix"

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'phoenix',
            namespace = namespace,
            executable = 'stream'
        ),
        Node(
            package = 'phoenix',
            namespace = namespace,
            executable = 'battery'
        ),
        Node(
            package = 'phoenix',
            namespace = namespace,
            executable = 'command'
        )
    ])
