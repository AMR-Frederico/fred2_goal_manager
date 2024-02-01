import launch_ros
import launch_ros.descriptions

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    

    declare_robot_localization_odom = DeclareLaunchArgument(
        '--use-robot-localization',
        default_value='true',  
        description='Use de odometry from robot localization'
    )


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
        arguments=[
            '--use-robot-localization',
            LaunchConfiguration('--use-robot-localization'),  
        ],


    )



    return LaunchDescription([

        declare_robot_localization_odom,
        goal_provider_node, 
        goal_reached_node

    ])
