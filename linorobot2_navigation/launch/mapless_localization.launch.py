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
    ekf_config_path = os.path.join(pkg_share, 'config/ekf.yaml')

    # robot_localization의 EKF 노드 실행
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[
                {'use_sim_time': use_sim_time}, 
                ekf_config_path
            ],
    )
    return LaunchDescription([
        robot_localization_node,
    ])