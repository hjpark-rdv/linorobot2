# navigate_mapless.launch.py: LIO-SAM + GPS 기반의 Mapless 자율주행을 위한 메인 Launch 파일

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ===================================================================================
    # --- 1. 기본 경로 및 파라미터 설정 ---
    # ===================================================================================
    
    # 이 Launch 파일이 있는 패키지의 share 디렉토리 경로를 찾습니다.
    # 'YOUR_PACKAGE_NAME'을 실제 패키지 이름으로 꼭 변경해주세요!
    pkg_share = get_package_share_directory('linorobot2_navigation')

    # 이전에 작성한 Nav2 파라미터 파일의 경로를 지정합니다.
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # 이전에 작성한 robot_localization 설정 파일의 경로를 지정합니다.
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # Nav2 실행에 필요한 기본 파라미터를 Launch Argument로 선언합니다.
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')

    # ===================================================================================
    # --- 2. robot_localization 노드 실행 ---
    # EKF를 사용하여 LIO-SAM과 GPS 데이터를 융합합니다.
    # ===================================================================================

    # EKF(Extended Kalman Filter) 노드 실행
    start_robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
        parameters=[
                {'use_sim_time': use_sim_time}, 
                ekf_config_path
            ],
    )

    # NavSat(GPS) Transform 노드 실행
    start_navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[
                {'use_sim_time': use_sim_time}, 
                ekf_config_path
            ],
        # 사용하는 실제 토픽 이름으로 리맵핑(remapping) 해야 합니다.
        remappings=[('imu', '/imu/data'),       # ⚠️ 실제 IMU 토픽으로 변경
                    ('gps/fix', '/gps/fix'),     # ⚠️ 실제 GPS 토픽으로 변경
                    ('odometry/filtered', '/odometry/filtered')]
    )

    # ===================================================================================
    # --- 3. Nav2 메인 스택 실행 ---
    # nav2_bringup 패키지의 navigation_launch.py를 포함하여 실행합니다.
    # ===================================================================================

    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_params_path,
        }.items()
    )

    # ===================================================================================
    # --- 4. LaunchDescription 생성 및 반환 ---
    # 위에서 정의한 모든 노드와 Launch 파일을 실행 리스트에 추가합니다.
    # ===================================================================================
    
    return LaunchDescription([
        # Launch Argument 선언
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'autostart', 
            default_value='true',
            description='Automatically startup Nav2 stack'),

        # robot_localization 노드들
        start_robot_localization_node,
        start_navsat_transform_node,

        # Nav2 메인 스택
        start_nav2_cmd,
    ])