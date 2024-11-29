import os
import yaml
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from pathlib import Path
import subprocess
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # パラメータファイルのパス設定
    config_file_path = os.path.join(
        get_package_share_directory('main_executor'),
        'config',
        'main_params.yaml'
    )

    # Livox MID360の起動ファイルのパス設定
    livox_launch_path = os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360_launch.py')
    # 起動パラメータファイルのロード
    with open(config_file_path, 'r') as file:
        launch_params = yaml.safe_load(file)['launch']['ros__parameters']

    # メイン実行機ノードの作成
    main_exec_node = Node(
        package = 'main_executor',
        executable = 'main_exec',
        parameters = [config_file_path],
        output='screen'
    )
    # トラッキングモジュールのノードの作成
    qwiic_node = Node(
        package = 'qwiic',
        executable = 'odom_node',
        parameters = [config_file_path],
        output='screen'
    )
    # 操縦機ノードの作成
    joy_node = Node(
        package = 'joy',
        executable = 'joy_node',
        output='screen'
    )
    # Livox起動の設定
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_launch_path)
    )
    # 起動エンティティクラスの作成
    launch_discription = LaunchDescription()

    # 起動の追加
    if(launch_params['joy'] is True):
        launch_discription.add_entity(joy_node)
    if(launch_params['livox_mid360'] is True):
        launch_discription.add_entity(livox_launch)
    if(launch_params['odom'] is True):
        launch_discription.add_entity(qwiic_node)

    launch_discription.add_entity(main_exec_node)

    return launch_discription
