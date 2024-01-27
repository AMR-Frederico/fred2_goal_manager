#!/usr/bin/env python3

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

from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import Bool


# Parameters file (yaml)
node_path = '~/ros2_ws/src/fred2_goal_manager/fred2_goal_manager/conf/goal_manager.yaml'
node_group = 'goal_provider'


debug_mode = '--debug' in sys.argv


class goal_provider(Node): 
    
    goal_reached = False
    
    goals_array = []
    current_index = 0

    current_goal = PoseStamped()

    mission_completed = Bool()

    last_goal_reached = False

    goal_reached = False

    def __init__(self, 
                 node_name: str, 
                 *, # keyword-only argument
                 cli_args: List[str] = None, 
                 namespace: str = None, 
                 use_global_arguments: bool = True, 
                 enable_rosout: bool = True, 
                 start_parameter_services: bool = True, 
                 parameter_overrides: List[Parameter] | None = None, 
                 allow_undeclared_parameters) -> None:
        
        super().__init__(node_name, 
                         cli_args=cli_args, 
                         namespace=namespace, 
                         use_global_arguments=use_global_arguments, 
                         enable_rosout=enable_rosout, 
                         start_parameter_services=start_parameter_services, 
                         parameter_overrides=parameter_overrides, 
                         allow_undeclared_parameters=allow_undeclared_parameters)
        
        # Load parameters
        self.load_params(node_path, node_group)

        self.create_subscription(Bool, 'goal/reached', self.goalReached_callback, 1)
        
        self.create_subscription(Bool, 'goal/reset', self.reset_callback, 10)

        self.goals_pub = self.create_publisher(PoseArray, 'goals', 10)

        self.goalCurrent_pub = self.create_publisher(PoseStamped, 'goal/current', 10)

        self.missionCompleted_pub = self.create_publisher(Bool, 'goal/mission_completed', 10)


        self.add_on_set_parameters_callback(self.parameters_callback)

        
    

    def parameters_callback(self, params):


        for param in params:

            self.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")

            if param.name == 'goal_provider.goals':
                # Assuming the value is a dictionary where keys are goal names and values are lists [x, y, theta]
                goals_dict = param.value
                # Convert the dictionary to a list of goal values
                self.goals_array = list(goals_dict.values())
                self.get_logger().info(f"Updated goals_array: {self.goals_array}")

            elif param.name == 'goal_provider.frame_id':

                self.FRAME_ID = param.value
                self.get_logger().info(f"Updated FRAME_ID: {self.FRAME_ID}")

        return SetParametersResult(successful=True)




    def main(self): 
        
        goals_pose = PoseArray()
        
        goals_pose.header.stamp = self.get_clock().now().to_msg()
        goals_pose.header.frame_id = self.FRAME_ID

        for goal_values in self.goals_array:
            
            x, y, theta = goal_values
            
            pose_msg = Pose()

            pose_msg.position.x = x 
            pose_msg.position.y = y
            
            pose_msg.orientation.z = theta
            pose_msg.orientation.w = 1.0

            goals_pose.poses.append(pose_msg)

        self.goals_pub.publish(goals_pose)

        if debug_mode: 

           self.get_logger().info(f'Current goal: {self.current_goal.pose.position.x}, {self.current_goal.pose.position.y}, {self.current_goal.pose.position.z}| Index: {self.current_index} | Number of goals: {len(self.goals_array)} ')
           self.get_logger().info(f'Goal reached: {self.goal_reached} | Last status: {self.last_goal_reached} | Update goal: {self.goal_reached > self.last_goal_reached} \n')


    def goalReached_callback(self, current_status):
        
        if current_status.data > self.last_goal_reached:

            if self.current_index < (len(self.goals_array) - 1):
                
                self.current_goal.header.stamp = self.get_clock().now().to_msg()
                self.current_goal.header.frame_id = self.FRAME_ID

                pose_msg = Pose()
                pose_msg.position.x, pose_msg.position.y, pose_msg.orientation.z = self.goals_array[self.current_index]

                self.current_goal.pose = pose_msg

                self.mission_completed.data = False
                self.missionCompleted_pub.publish(self.mission_completed)

                self.goalCurrent_pub.publish(self.current_goal)

                # Increment the index only when an update happens
                self.current_index += 1

            if self.current_index == len(self.goals_array) - 1:
                self.mission_completed.data = True
                self.missionCompleted_pub.publish(self.mission_completed)
        

        self.last_goal_reached = current_status.data
        self.goal_reached = current_status.data



    
    def reset_callback(self, reset_msg):
        
        reset_goals = reset_msg.data

        if reset_goals: 
            
            self.current_index = 0
    



    def load_params(self, path, group): 
        param_path = os.path.expanduser(path)

        with open(param_path, 'r') as params_list: 
            params = yaml.safe_load(params_list)

        # Get the params inside the specified group
        params = params.get(group, {})

        # Get the 'goals' parameter
        goals_param = params.get('goals', [])

        # Initialize goals_array
        self.goals_array = []

        # Iterate over goal parameters
        for goal_name, goal_values in goals_param.items():
            # Append goal values to goals_array
            self.goals_array.append(goal_values)

        # Get the 'frame_id' parameter
        self.FRAME_ID = params.get('frame_id', 'odom')

        # Declare the 'frame_id' parameter
        self.declare_parameter('frame_id', self.FRAME_ID)

        # Print loaded parameters
        self.get_logger().info(f'Loaded parameters: goals_array={self.goals_array}, frame_id={self.FRAME_ID}')




if __name__ == '__main__': 
    
    rclpy.init()
    
    node = goal_provider('goal_provider', 
                         namespace='goal_manager', 
                         cli_args=['--debug'], 
                         start_parameter_services= True, 
                         allow_undeclared_parameters= True)
    
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
