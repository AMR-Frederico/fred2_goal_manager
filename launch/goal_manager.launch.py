import launch_ros
import launch_ros.descriptions

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    goal_provider_node = launch_ros.actions.Node(

        package = 'fred2_goal_manager',
        executable = 'goal_provider.py',
        name = 'goal_provider',
        output = 'screen',
    )


    goal_reached_node = launch_ros.actions.Node(
       
      package = 'fred2_goal_manager',
      executable = 'goal_reached.py',
      name = 'goal_reached',
      output = 'screen',

    )



    return LaunchDescription([

        goal_provider_node, 
        goal_reached_node

    ])
