# localization.launch.py: Launches robot_localization nodes

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = True
    # 실행할 패키지의 share 디렉토리 경로를 찾음
    pkg_share = get_package_share_directory('linorobot2_navigation') # 'YOUR_PACKAGE_NAME'을 실제 패키지 이름으로 변경

    # ekf.yaml 파일의 절대 경로 생성
    navsat_config_path = os.path.join(pkg_share, 'config/navsat.yaml')


    # robot_localization의 NavSat Transform 노드 실행
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[
                {'use_sim_time': use_sim_time}, 
                navsat_config_path
            ],
        # navsat_transform_node는 토픽 이름 리맵핑이 필요한 경우가 많음
        remappings=[('imu', '/imu/data'), # imu입력
                    ('gps/fix', '/gps/fix'), # gps입력
                    # ('odometry/filtered', '/odom/wheel')] # odom 입력
                    ('odometry/filtered', '/odom/wheel')] # odom 입력
    )

    return LaunchDescription([
        navsat_transform_node
    ])