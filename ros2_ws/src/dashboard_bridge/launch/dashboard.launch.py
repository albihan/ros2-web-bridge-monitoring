from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directory
    pkg_dir = get_package_share_directory('dashboard_bridge')
    
    return LaunchDescription([
        # 1. Custom Event Bridge Node (file Python kita)
        Node(
            package='dashboard_bridge',
            executable='event_bridge',
            name='event_bridge',
            output='screen',
            parameters=[{
                'ws_port': 9090,
                'debug': True,
            }],
            emulate_tty=True,
        ),
        
        # 2. ROS Bridge WebSocket Server
        ExecuteProcess(
            cmd=['ros2', 'run', 'rosbridge_server', 'rosbridge_websocket',
                 '--port', '9090',
                 '--max_message_size', '10000000'],
            output='screen',
            shell=True,
        ),
        
        # 3. (Optional) Web Video Server jika butuh streaming
        # Node(
        #     package='web_video_server',
        #     executable='web_video_server',
        #     name='web_video_server',
        #     output='screen',
        # ),
    ])