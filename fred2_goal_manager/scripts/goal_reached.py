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
from rcl_interfaces.srv import GetParameters

from math import hypot

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool, Int16

# Parameters file (yaml)
node_path = '~/ros2_ws/src/fred2_goal_manager/conf/goal_manager.yaml'
node_group = 'goal_reached'


debug_mode = '--debug' in sys.argv
use_robot_localization = '--use-robot-localization' in sys.argv



class goal_reached(Node): 
    
    robot = Pose()
    goal = Pose()

    robot_in_goal = Bool()
    robot_in_goal.data = False

    robot_state = 100



    # starts with randon value 
    ROBOT_MANUAL = 1000
    ROBOT_AUTONOMOUS = 1000
    ROBOT_IN_GOAL = 1000
    ROBOT_MISSION_COMPLETED = 1000
    ROBOT_EMERGENCY = 1000



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
        


        self.create_subscription(PoseStamped, 
                                 'goal/current', 
                                 self.goalCurrent_callback, 
                                 10)
        

        self.create_subscription(Int16,
                                 '/machine_states/robot_state',
                                 self.robot_state_callback, 
                                 5 )
        

        if use_robot_localization: 
            
            self.get_logger().info('Using ROBOT LOCALIZATION odometry')
            
            self.create_subscription(Odometry,
                                    '/odometry/filtered', 
                                    self.odom_callback, 
                                    10)
        

        else: 
            
            self.get_logger().info('Using MOVE BASE odometry')
            
            self.create_subscription(Odometry,
                                    '/odom', 
                                    self.odom_callback, 
                                    10)


        self.goalReached_pub = self.create_publisher(Bool, 
                                                     'goal/reached', 
                                                     10)


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



        # Get global params 

        self.client = self.create_client(GetParameters, '/machine_states/main_robot/get_parameters')
        self.client.wait_for_service()

        request = GetParameters.Request()
        request.names = ['manual', 'autonomous', 'in_goal', 'mission_completed', 'emergency']

        future = self.client.call_async(request)
        future.add_done_callback(self.callback_global_param)


    
    def callback_global_param(self, future):


        try:

            result = future.result()

            self.ROBOT_MANUAL = result.values[0].integer_value
            self.ROBOT_AUTONOMOUS = result.values[1].integer_value
            self.ROBOT_IN_GOAL = result.values[2].integer_value
            self.ROBOT_MISSION_COMPLETED = result.values[3].integer_value
            self.ROBOT_EMERGENCY = result.values[4].integer_value


            self.get_logger().info(f"Got global param ROBOT_MANUAL -> {self.ROBOT_MANUAL}")
            self.get_logger().info(f"Got global param ROBOT_AUTONOMOUS -> {self.ROBOT_AUTONOMOUS}")
            self.get_logger().info(f"Got global param ROBOT_IN GOAL -> {self.ROBOT_IN_GOAL}")
            self.get_logger().info(f"Got global param ROBOT_MISSION_COMPLETED: {self.ROBOT_MISSION_COMPLETED}")
            self.get_logger().info(f"Got global param ROBOT_EMERGENCY: {self.ROBOT_EMERGENCY}\n")



        except Exception as e:

            self.get_logger().warn("Service call failed %r" % (e,))



    def odom_callback(self, odom_msg): 

        self.robot.position.x = odom_msg.pose.pose.position.x 
        self.robot.position.y = odom_msg.pose.pose.position.y 
        self.robot.orientation.z = odom_msg.pose.pose.orientation.z 
    


    def goalCurrent_callback(self, goal_msg): 
        
        self.goal.position.x = goal_msg.pose.position.x 
        self.goal.position.y = goal_msg.pose.position.y 


    def robot_state_callback(self, robot_state): 
        
        self.robot_state = robot_state.data


    def main(self): 
        
        dx = self.goal.position.x - self.robot.position.x 
        dy = self.goal.position.y - self.robot.position.y 

        linear_error = hypot(dx, dy)

        if (linear_error < self.ROBOT_IN_GOAL_TOLERANCE) and self.robot_state == self.ROBOT_AUTONOMOUS: 
            
            self.robot_in_goal.data = True

            self.get_logger('Goal Reached!!!!')
        
        else: 
            
            self.robot_in_goal.data = False


        self.goalReached_pub.publish(self.robot_in_goal)

        if debug_mode: 
            self.get_logger().info(f'Delta X: {dx} | Delta Y: {dy}')
            self.get_logger().info(f'Linear error: {linear_error} | Tolerance: {self.ROBOT_IN_GOAL_TOLERANCE} | Robo in goal: {self.robot_in_goal.data}\n')



if __name__ == '__main__': 
    rclpy.init()
    node = goal_reached(
        node_name='goal_reached', 
        namespace='goal_manager', 
        cli_args=['--debug', '--use-robot-localization'] )

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(1)

    try: 
        while rclpy.ok(): 
            node.main()
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
