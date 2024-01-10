#!/usr/bin/env python3

import os
import yaml
import rclpy
import threading

from rclpy.node import Node
from typing import List, Optional
from rclpy.context import Context 
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseArray, Pose

# Parameters file (yaml)
node_path = '~/ros2_ws/src/fred2_goal_manager/fred2_goal_manager/conf/goal_manager.yaml'
node_group = 'goal_provider'

LED = 1.0
GHOST = 0.0

class goal_provider(Node): 
    
    goals_array = [[1.0, 2.0, LED], 
                   [2.0, 3.0, GHOST]]


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
        
        self.goals_pub = self.create_publisher(PoseArray, '/goal_manager/goals', 10)
        

        # self.load_params(node_path, node_group)
        # self.get_params()
    
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

    # def get_params(self):
    #     print('oi')

    
    
    def publish_goals(self): 
        
        goals_pose = PoseArray()
        
        goals_pose.header.stamp = self.get_clock().now().to_msg()
        goals_pose.header.frame_id = 'odom'
        
        for goal_data in self.goals_array:
            x, y, theta = goal_data

            pose_msg = Pose()
            pose_msg.position.x = x 
            pose_msg.position.y = y
            pose_msg.orientation.z = theta
            pose_msg.orientation.w = 1.0

            goals_pose.poses.append(pose_msg)

        self.goals_pub.publish(goals_pose)



if __name__ == '__main__': 
    rclpy.init()
    node = goal_provider('goal_provider', namespace='goal_manager', cli_args=['--debug'] )

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(10)

    try: 
        while rclpy.ok(): 
            node.publish_goals()
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
