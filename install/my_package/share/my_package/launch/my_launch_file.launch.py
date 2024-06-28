import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_directory = get_package_share_directory('my_package')
    urdf_file = os.path.join(package_share_directory, 'models', 'my_robot', 'robot.urdf')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    return LaunchDescription([
        # Gazebo 서버 및 클라이언트 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'verbose': 'true'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            launch_arguments={'verbose': 'true'}.items(),
        ),
        # 로봇 상태 퍼블리셔 실행
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        # Gazebo에 로봇 스폰
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-entity', 'scarecrow_robot', '-file', urdf_file]
        ),
        # 센서 노드 실행
        Node(
            package='my_package',
            executable='sensor_node',
            name='sensor_node',
            output='screen'
        ),
        # 모니터 노드 실행
        Node(
            package='my_package',
            executable='monitor_node',
            name='monitor_node',
            output='screen'
        ),
        # 제어 노드 실행
        Node(
            package='my_package',
            executable='control_node',
            name='control_node',
            output='screen'
        ),
        # 모터 노드 실행
        Node(
            package='my_package',
            executable='motor_node',
            name='motor_node',
            output='screen'
        ),
        # Flask 노드 실행
        Node(
            package='my_package',
            executable='flask_node',
            name='flask_node',
            output='screen'
        ),
    ])
