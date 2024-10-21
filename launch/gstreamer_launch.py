from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('ros2_streamer'),
        'config',
        'gstreamer_config.yaml'
    )

    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)

    print(config)

    return LaunchDescription(
        generate_nodes(config)
    )

def generate_nodes(config_file):
    list_of_nodes = []
    ip_host = config_file['ip_host']
    ntp_server = config_file['ntp_server']
    bitrate = config_file['bitrate']
    local_time_frequency = config_file['local_time_frequecy']
    pan_tilt_frequency = config_file['pan_tilt_frequency']

    for camera_key in config_file['cameras']:
        camera = config_file['cameras'][camera_key]
        
        list_of_nodes.append(Node(
            package='ros2_streamer',
            executable='service',
            name='gstreamer_service_' + camera['name'],
            output='screen',
            parameters=[
                {'camera_name': camera['name']},
                {'width': camera['width']},
                {'height': camera['height']},
                {'pan_tilt': camera['pan_tilt']},
                {'rtp_port': camera['port']},
                {'rtp_dest': ip_host},
                {'device': camera['usb_port']},
                {'ntp_server': ntp_server},
                {'bitrate': bitrate},
                {'local_time_frequency': local_time_frequency},
                {'pan_tilt_frequency': pan_tilt_frequency}
            ]
        ))

    return list_of_nodes
