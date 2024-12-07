import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import Shutdown
from launch.actions                    import DeclareLaunchArgument
from launch.actions                    import OpaqueFunction
from launch.substitutions              import LaunchConfiguration
from launch_ros.actions                import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    rvizcfg = os.path.join(pkgdir('final_project'), 'rviz/viewurdfplus.rviz')

    urdf = os.path.join(pkgdir('final_project'), 'urdf/sevenDOF.urdf')
    with open(urdf, 'r') as file:
        robot_description = file.read()

    node_robot_state_publisher = Node(
        name       = 'robot_state_publisher', 
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description}])

    node_rviz = Node(
        name       = 'rviz', 
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown())
    
    node_controller = Node(
        name       = 'controller', 
        package    = 'final_project',
        executable = 'controller',
        output     = 'screen')
    
    ball_viz = Node(
        name       = 'balls', 
        package    = 'final_project',
        executable = 'balls',
        output     = 'screen')
    
    bin_viz = Node(
        name       = 'target', 
        package    = 'final_project',
        executable = 'target',
        output     = 'screen')

    return LaunchDescription([
        node_robot_state_publisher,
        node_rviz,
        node_controller,
        ball_viz,
        bin_viz
    ])
