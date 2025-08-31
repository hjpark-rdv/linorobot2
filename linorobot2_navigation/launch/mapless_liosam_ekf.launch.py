from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('linorobot2_navigation'), 'config')  # 'your_package_name'을 실제 패키지 이름으로 변경
    params_file = os.path.join(config_dir, 'liosam_ekf_navsat.yaml')

    return LaunchDescription([
        # navsat_transform_node 실행
        # Node(
        #     package='robot_localization',
        #     executable='navsat_transform_node',
        #     name='navsat_transform_node',
        #     output='screen',
        #     parameters=[params_file],
        #     remappings=[
        #         ('/imu/data', '/imu/data'),  # IMU 토픽 리맵 (필요 시 변경)
        #         ('/gps/fix', '/gps/fix'),    # GPS 토픽
        #         ('/odometry/filtered', '/odometry/navsat')  # LIO-SAM odometry 입력 (필요 시 변경)
        #     ]
        # ),

        # ekf_filter_node 실행
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[params_file],
            remappings=[
                # ('/imu', '/imu/data'),  # IMU 토픽 (ROS1의 imu_correct에 해당, 실제 토픽으로 변경)
                # ('/gps/fix', 'gps/fix'),
                ('/odometry/filtered', '/odometry/navsat')  # navsat_transform_node 출력
            ]
        )
    ])