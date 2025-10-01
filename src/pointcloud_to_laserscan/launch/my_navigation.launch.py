from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # pointcloud_to_laserscan 노드
    pcl2scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        parameters=['/home/vertin/pointcloud_to_laserscan_params.yaml'],  # ← yaml 파일 경로!
        remappings=[
            ('cloud_in', '/unilidar/cloud'),  # 3D 라이다 토픽에 맞게 수정
            ('scan', '/scan')
        ],
        output='screen'
    )

    # slam_toolbox 노드 (추가, yaml 파라미터 적용)
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=['/home/vertin/slam_toolbox_params.yaml'],
        output='screen'
    )

    return LaunchDescription([
        pcl2scan_node,
        TimerAction(
            period=5.0,  # 3초 딜레이
            actions=[slam_node]
        )
    ])

