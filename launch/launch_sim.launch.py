import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

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

  default_world = os.path.join(
    get_package_share_directory(package_name),
    'worlds',
    'obstacles.sdf'
    )
  
  world = LaunchConfiguration('world')

  world_arg = DeclareLaunchArgument(
    'world',
    default_value=default_world,
    description='World to load'
    )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
  gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

  # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
  spawn_entity = Node(package='ros_gz_sim', executable='create',
    arguments=['-topic', 'robot_description',
    '-name', 'my_bot',
    '-z', '0.1'],
    output='screen')

  diff_drive_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["diff_cont"],
  )

  joint_broad_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_broad"],
  )

  bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
  
  ros_gz_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
      '--ros-args',
      '-p',
      f'config_file:={bridge_params}',
    ]
  )

  return LaunchDescription([
    rsp,
    # joint_state_publisher_gui,
    rviz2,
    world_arg,
    gazebo,
    spawn_entity,
    # diff_drive_spawner,
    # joint_broad_spawner,
    ros_gz_bridge
  ])