from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('lidar_test_bench')
    params_file = os.path.join(share, 'config', 'lidar_test_bench.yaml')

    reporting_share = get_package_share_directory('lidar_reporting')
    reporting_params_file = os.path.join(reporting_share, 'config', 'config.yaml')

    transforms_share = get_package_share_directory('lidar_transforms')
    lidar_frame_params_file = os.path.join(transforms_share, 'config', 'lidar_frame.yaml')

    venv = os.environ.get('VIRTUAL_ENV')
    python_exec = os.path.join(venv, 'bin', 'python3') if venv else 'python3'

    tf_broadcaster = Node(
        package='lidar_transforms',
        executable='lidar_bench_tf_broadcaster',
        name='lidar_bench_tf_broadcaster',
        output='screen',
        parameters=[lidar_frame_params_file],
    )

    baseline_node = Node(
        package='lidar_transforms',
        executable='lidar_baseline_node',
        name='lidar_baseline_node',
        output='screen',
        parameters=[lidar_frame_params_file],
    )

    lidar_controller = ExecuteProcess(
        cmd=[
            python_exec,
            '-m', 'lidar_test_bench.nodes.lidar_controller',
            '--ros-args',
            '-r', '__node:=lidar_controller',
            '--params-file', params_file,
        ],
        output='screen',
    )

    grafana_reporter = Node(
        package='lidar_reporting',
        executable='grafana_reporter_node',
        name='grafana_reporter_node',
        output='screen',
        parameters=[reporting_params_file],
    )

    return LaunchDescription([
        tf_broadcaster,
        baseline_node,
        lidar_controller,
        grafana_reporter,
    ])
