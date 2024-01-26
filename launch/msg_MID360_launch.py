from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, NotEqualsSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetRemap
from launch.conditions import IfCondition

################### user configure parameters for ros2 start ###################
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

user_config_path_default = PathJoinSubstitution([
                    FindPackageShare('livox_ros_driver2'),
                    "config",
                    "MID360_config.json"
                ])
################### user configure parameters for ros2 end #####################

def generate_launch_description():
    
    user_config_path_arg = DeclareLaunchArgument(
              'user_config_path',
              default_value=user_config_path_default,
              description="Path to json config file."
    )

    xfer_format_arg = DeclareLaunchArgument(
              'xfer_format',
              default_value='0',
              description="0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format"
    )

    lidar_topic_arg = DeclareLaunchArgument(
              'lidar_topic',
              default_value="",
              description="Remap lidar topic to supplied topic."
    )
    lidar_topic = LaunchConfiguration('lidar_topic')

    lidar_remap = SetRemap('/livox/lidar', lidar_topic, condition=IfCondition(NotEqualsSubstitution(lidar_topic,"")))


    imu_topic_arg = DeclareLaunchArgument(
              'imu_topic',
              default_value="",
              description="Remap topic to this."
    )
    imu_topic = LaunchConfiguration('imu_topic')

    imu_remap = SetRemap('/livox/imu', imu_topic, condition=IfCondition(NotEqualsSubstitution(imu_topic,"")))

    livox_ros2_params = [
      {"xfer_format": LaunchConfiguration('xfer_format')},
      {"multi_topic": multi_topic},
      {"data_src": data_src},
      {"publish_freq": publish_freq},
      {"output_data_type": output_type},
      {"frame_id": frame_id},
      {"lvx_file_path": lvx_file_path},
      {"user_config_path": LaunchConfiguration('user_config_path') },
      {"cmdline_input_bd_code": cmdline_bd_code}
    ]

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
        )
    
    return LaunchDescription([user_config_path_arg, xfer_format_arg, lidar_topic_arg, imu_topic_arg, lidar_remap, imu_remap, livox_driver])
