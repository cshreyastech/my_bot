import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
  package_name = 'my_bot'

  rsp = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory(
        package_name), 'launch', 'rsp.launch.py'
    )]), launch_arguments={'use_sim_time': 'true'}.items()
  )

  # joint_state_publisher_gui = Node(
  #           package='joint_state_publisher_gui',
  #           executable='joint_state_publisher_gui',
  #           name='joint_state_publisher_gui',
  #           output='screen'
  #           )
  
  rviz2 = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', 'src/my_bot/config/my_bot_v2.rviz']
  )

  return LaunchDescription([
    rsp,
    # joint_state_publisher_gui,
    rviz2
  ])