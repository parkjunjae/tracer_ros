from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    # 아무 내용 없는 yaml을 하나 만들어 두고 경로만 지정
    dummy_map = '/home/vertin/ros2_ws/src/nav2_map/dummy_map.yaml'

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'slam': 'True',
                'map': dummy_map,    # dummy 파일이라도 반드시 지정
                'params_file': '/home/vertin/ros2_ws/src/nav2_map/nav2_params.yaml',
                'use_sim_time': 'false'
            }.items()
        )
    ])

