import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import xacro

def to_urdf(xacro_path, urdf_path=None, mappings={}):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * urdf_path -- the path to the urdf file
    """
    # If no URDF path is given, use a temporary file
    if urdf_path is None:
        urdf_path = tempfile.mktemp(prefix='%s_' % os.path.basename(xacro_path))

    # open and process file
    doc = xacro.process_file(xacro_path, mappings=mappings)
    # open the output file
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    robot_desc = Command('xacro %s' % xacro_path)

    return urdf_path, robot_desc  # Return robot description


def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')

  #Load robot model
  # Load XACRO and parse to URDF and then put that on pa
  neato_description_pkg = get_package_share_directory('neato_description')
  xacro_model_name = "neato.urdf.xacro"
  xacro_model_path = os.path.join(neato_description_pkg, 'urdf', xacro_model_name)
  urdf_model_path, robot_description = to_urdf(xacro_model_path)
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