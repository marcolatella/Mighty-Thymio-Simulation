from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    """
    ROS 2 launch files are special Python files that contain a function named
    generate_launch_description. This function must return a LaunchDescription
    object that represents the sets of actions that ROS should execute to start
    the system
    """
    
    return LaunchDescription([
        # Create a launch argument. It can be used to supply values on the
        # command line, with this syntax:
        #   ros2 launch [...] thymio_name:=thymio1
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='False'
        ),
        DeclareLaunchArgument(
            'thymio_name',
            default_value='thymio0'
        ),
        
        # Start a ROS node, equivalent to ros2 run <package_name> <executable_name>
        Node(
            package='thymio_ld',
            executable='controller_node',
            parameters = [{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            
            # Launch controller_node in the namespace defined by the thymio_name
            # launch argument (default: /thymio0). Combined with the use of
            # relative topic names in controller_node, this provides a nice way
            # to specify which robot this node should control.
            namespace=LaunchConfiguration('thymio_name'),

            
            # Nodes launched from a launch file print their output to a log file
            # by default. Print to the console to help with debugging.
            output='screen'
        )
    ])

