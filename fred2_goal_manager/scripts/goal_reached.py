#!/usr/bin/env python3

import rclpy 

from rclpy.node import Node

import os
import sys
import yaml
import rclpy
import threading

from rclpy.node import Node
from typing import List, Optional
from rclpy.context import Context 
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from math import hypot

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool

# Parameters file (yaml)
node_path = '~/ros2_ws/src/fred2_goal_manager/fred2_goal_manager/conf/goal_manager.yaml'
node_group = 'goal_reached'


debug_mode = '--debug' in sys.argv


class goal_reached(Node): 
    
    robot = Pose()
    goal = Pose()

    robot_in_goal = Bool()
    robot_in_goal.data = False


    def __init__(self, 
                 node_name: str, 
                 *, # keyword-only argument
                 cli_args: List[str] = None, 
                 namespace: str = None, 
                 use_global_arguments: bool = True, 
                 enable_rosout: bool = True, 
                 start_parameter_services: bool = True, 
                 parameter_overrides: List[Parameter] | None = None) -> None:
        
        super().__init__(node_name, 
                         cli_args=cli_args, 
                         namespace=namespace, 
                         use_global_arguments=use_global_arguments, 
                         enable_rosout=enable_rosout, 
                         start_parameter_services=start_parameter_services, 
                         parameter_overrides=parameter_overrides)
        
        
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.create_subscription(PoseStamped, 'goal/current', self.goalCurrent_callback, 10)

        self.goalReached_pub = self.create_publisher(Bool, 'goal/reached', 10)


        self.load_params(node_path, node_group)
        self.get_params()


        self.add_on_set_parameters_callback(self.parameters_callback)



    def parameters_callback(self, params):
        
        for param in params:
            self.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")



        if param.name == 'robot_in_goal_tolerence':
            self.ROBOT_IN_GOAL_TOLERANCE = param.value
    


        return SetParametersResult(successful=True)





    def load_params(self, path, group): 
        param_path = os.path.expanduser(path)

        with open(param_path, 'r') as params_list: 
            params = yaml.safe_load(params_list)
        
        # Get the params inside the specified group
        params = params.get(group, {})

        # Declare parameters with values from the YAML file
        for param_name, param_value in params.items():
            # Adjust parameter name to lowercase
            param_name_lower = param_name.lower()
            self.declare_parameter(param_name_lower, param_value)
            self.get_logger().info(f'{param_name_lower}: {param_value}')



    def get_params(self):
        
        self.ROBOT_IN_GOAL_TOLERANCE = self.get_parameter('robot_in_goal_tolerence').value



    def odom_callback(self, odom_msg): 

        self.robot.position.x = odom_msg.pose.pose.position.x 
        self.robot.position.y = odom_msg.pose.pose.position.y 
        self.robot.orientation.z = odom_msg.pose.pose.orientation.z 
    


    def goalCurrent_callback(self, goal_msg): 
        
        self.goal.position.x = goal_msg.pose.position.x 
        self.goal.position.y = goal_msg.pose.position.y 


    def main(self): 
        
        dx = self.goal.position.x - self.robot.position.x 
        dy = self.goal.position.y - self.robot.position.y 

        linear_error = hypot(dx, dy)

        self.robot_in_goal.data = linear_error < self.ROBOT_IN_GOAL_TOLERANCE

        self.goalReached_pub.publish(self.robot_in_goal)

        if debug_mode: 
            self.get_logger().info(f'Delta X: {dx} | Delta Y: {dy}')
            self.get_logger().info(f'Linear error: {linear_error} | Tolerance: {self.ROBOT_IN_GOAL_TOLERANCE} | Robo in goal: {self.robot_in_goal.data}\n')


if __name__ == '__main__': 
    rclpy.init()
    node = goal_reached('goal_reached', namespace='goal_manager', cli_args=['--debug'] )

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(10)

    try: 
        while rclpy.ok(): 
            node.main()
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
