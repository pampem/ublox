from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1) 既存の u-blox 複合ノードの Launch を include
    ublox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ublox_gps'),
                'launch',
                'ublox_gps_node-composed-launch.py'
            ])
        ),
        # 必要なら launch_arguments={"frame_id": "gps", ...}.items()
    )

    # 2) 起動時の一発時刻合わせノード（ヘルパー呼び出し方式）
    gps_time_setter = Node(
        package='ublox_gps',
        executable='gps_time_setter',
        name='gps_time_setter',
        output='screen',
        parameters=[{
            'navpvt_topic': 'ublox_gps_node/navpvt',   # NavPVTのトピック
            'min_fix_type': 3,                         # 少なくとも3D Fix
            'tacc_threshold_ns': 50000,                # tAcc ≤ 50 µs
            'once_only': True,                         # 一度だけ合わせて終了
            'helper_path': '/usr/local/sbin/settime_utc'  # 特権ヘルパーの絶対パス
        }],
        # remappings=[('ublox_gps_node/navpvt', 'your/navpvt/topic')]  # 必要なら
    )

    return LaunchDescription([
        ublox_launch,
        gps_time_setter,
    ])
