#############################################################
# ROS2 and Gazebo launch file of the differential drive robot



#############################################################

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro


def generate_launch_description():
  # this name has to match the robot name in the Xacro file
  robotXacroName = 'my_bot_ddr'

  # this is the name of our package, at the same time this is the name of the
  # folder that will be used to define the paths
  namePackage = 'my_bot'

  # this is a relative path to the xacro file defining the model
  modelFileRelativePath = 'description/robot.urdf.xacro'

  # uncomement this if you want to define your own empty world model
  # however, then you have to create empty_world.world
  # this is a relative path to the Gazebo world file
  # worldFileRelativePath = 'model/empty_world.world'

  # this os tje absolute path to the model
  pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
  
  # uncomment this if you are using your own wold model
  # this is the absolute path to the world model
  # pathWorldFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)

  # get the robot description from the xacro model file
  robotDescription = xacro.process_file(pathModelFile).toxml()


  # this is the launch file from the gazebo_ros package
  gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),
                                                                        'launch', 'gz_sim.launch.py'))
  
  # this ithe launch description

  # this is if you are using your own world model
  # gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r -v -v4', pathWorldFile], 'on_exit_shutdown': 'true'}.tiems())

  # this is if you are using an empty world model
  gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r -v -v4 empty.sdf'], 'on_exit_shutdown': 'true'}.items())

  # Gazebo node
  spawnModelNodeGazebo = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
      '-name', robotXacroName,
      '-topic', 'robot_description'
    ],
    output='screen'
  )

  # Robot State Publisher node
  nodeRobotStatePublisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[{'robot_description': robotDescription,
    'use_sim_time': True}]
  )

  # this is very import so we can control the robot from ROS2
  bridge_params = os.path.join(
    get_package_share_directory(namePackage),
    'parameters',
    'bridge_parameters.yaml'
  )

  start_gazebo_ros_bridge_cmd = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
      '--ros-args',
      '-p',
      f'config_file:={bridge_params}',
    ],
    output='screen',
  )

  # here we create an empty lauch description object
  launchDescriptionObject = LaunchDescription()

  # we add gazeboLaunch
  launchDescriptionObject.add_action(gazeboLaunch)

  # we add the two nodes
  launchDescriptionObject.add_action(spawnModelNodeGazebo)
  launchDescriptionObject.add_action(nodeRobotStatePublisher)
  launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)

  return launchDescriptionObject