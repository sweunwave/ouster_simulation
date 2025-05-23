import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, EmitEvent, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

def generate_launch_description():
  this_directory = get_package_share_directory('ouster_description')
  xacro_path = os.path.join(this_directory, 'urdf', 'os1_32_example.urdf.xacro')
  world = os.path.join(this_directory, 'worlds', 'test.world')
  rviz_config_file = os.path.join(this_directory, 'rviz', 'test.rviz')

  declare_gpu_cmd = DeclareLaunchArgument(
    'gpu',
    default_value='false',
    description='Whether to use Gazebo gpu_ray or ray')
  gpu = LaunchConfiguration('gpu')
  robot_description = Command(['xacro',' ', xacro_path, ' gpu:=', gpu])

  start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{
      'use_sim_time': True,
      'robot_description': robot_description
    }]
  )

  spawn_example_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=[
      '-entity', 'example',
      '-topic', 'robot_description',
    ],
    output='screen',
  )

  start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file],
    output='screen'
  )

  exit_event_handler = RegisterEventHandler(
    event_handler=OnProcessExit(
      target_action=start_rviz_cmd,
      on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))
    )
  )

  declare_gui_cmd = DeclareLaunchArgument(
    'gui',
    default_value='True',
    description='Whether to launch the Gazebo GUI or not (headless)')
  gui = LaunchConfiguration('gui')
  start_gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
    launch_arguments={'world' : world, 'gui' : gui, 'verbose' : 'true'}.items()
  )

  ld = LaunchDescription()

  # Add the actions
  ld.add_action(declare_gpu_cmd)
  ld.add_action(declare_gui_cmd)
  ld.add_action(start_gazebo)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(spawn_example_cmd)
  ld.add_action(start_rviz_cmd)
  ld.add_action(exit_event_handler)

  return ld