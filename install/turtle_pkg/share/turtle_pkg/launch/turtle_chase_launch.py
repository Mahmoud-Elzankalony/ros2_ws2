from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld=LaunchDescription()
    
    turtlesim_node=Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
        )
    turtle_chase_node=Node(
            package='turtle_pkg',
            executable='turtle_chase',
            name='turtle',
        )
    
    ld.add_action(turtlesim_node)
    ld.add_action(turtle_chase_node)
    return ld