import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 패키지 공유 디렉토리 경로
    share_dir = FindPackageShare('lio_sam').find('lio_sam')

    # 파라미터 YAML 파일 경로 선언
    parameter_file = LaunchConfiguration('params_file')

    # xacro 파일 경로
    xacro_file = PathJoinSubstitution([
        share_dir, 'config', 'robot.urdf.xacro'
    ])

    # robot_description 파라미터 설정
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # RViz 설정 파일 경로
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('lio_sam'),
        'config',
        'rviz2.rviz'
    ])

    # params.yaml 경로를 launch argument로 선언
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'config', 'params.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    return LaunchDescription([
        params_declare,

        # map -> odom static transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
            output='screen'
        ),

        # robot_state_publisher 노드 실행
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # LIO-SAM 핵심 노드 실행
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[parameter_file],
            output='screen'
        ),

        # RViz 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])

