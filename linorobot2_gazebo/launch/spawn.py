import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 시뮬레이션 시간을 사용하도록 설정
    use_sim_time = True

    # URDF 파일 경로 설정
    robot_base = os.getenv('LINOROBOT2_BASE', '2wd') # 기본값을 '2wd'로 설정
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_description"), "urdf/robots", f"{robot_base}.urdf.xacro"]
    )

    # robot_state_publisher 등을 실행하는 description.launch.py 파일 경로
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
    )

    return LaunchDescription([
        # --- Launch Arguments (스폰 위치 등 설정) ---
        DeclareLaunchArgument(
            name='urdf', 
            default_value=urdf_path,
            description='URDF path'
        ),
        DeclareLaunchArgument(
            name='spawn_x', 
            default_value='0.0',
            description='Robot spawn position in X axis'
        ),
        DeclareLaunchArgument(
            name='spawn_y', 
            default_value='0.0',
            description='Robot spawn position in Y axis'
        ),
        DeclareLaunchArgument(
            name='spawn_z', 
            default_value='0.0',
            description='Robot spawn position in Z axis'
        ),
        DeclareLaunchArgument(
            name='spawn_yaw', 
            default_value='0.0',
            description='Robot spawn heading'
        ),
        
        # --- 핵심 노드 ---
        
        # 1. URDF 파일을 파싱하여 /robot_description 토픽으로 발행 (robot_state_publisher 실행)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints': 'false',
                'urdf': LaunchConfiguration('urdf')
            }.items()
        ),

        # 2. /robot_description 토픽을 구독하여 Gazebo에 로봇 모델을 스폰
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=[
                '-topic ', 'robot_description',  # <- 이 부분이 문제의 원인이었습니다.
                '-entity ', 'linorobot2', 
            ]
        ),
    ])