import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_helpers import to_urdf

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')

  ## ROBOT MODEL
  # Load XACRO and parse to URDF
  neato_description_pkg = get_package_share_directory('neato_description')
  xacro_model_name = "neato.urdf.xacro"
  xacro_model_path = os.path.join(neato_description_pkg, 'urdf', xacro_model_name)

  # Parse XACRO file to URDF
  urdf_model_path = to_urdf(xacro_model_path)
  urdf_params = {'urdf_model_path': urdf_model_path}

  return LaunchDescription([
      DeclareLaunchArgument(
          'use_sim_time',
          default_value='false',
          description='Use simulation (Gazebo) clock if true'),
      Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          output='screen',
          parameters=[{'use_sim_time': use_sim_time}],
          arguments=[urdf_params]),
  ])