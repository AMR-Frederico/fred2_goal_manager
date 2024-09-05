#!/usr/bin/env python3

import sys
import rclpy
import threading

import fred2_goal_manager.scripts.debug as debug 
import fred2_goal_manager.scripts.parameters as params 
import fred2_goal_manager.scripts.publishers as publishers 
import fred2_goal_manager.scripts.qos as qos 
import fred2_goal_manager.scripts.subscribers as subscribers

from typing import List

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import Bool


debug_mode = '--debug' in sys.argv


class goal_provider(Node): 
    
    goal_reached = False                # Indicates if the goal has been reached
    
    goals = []
    current_index = 0                   # Index of the current goal in the list of goals

    current_goal = PoseStamped()        # PoseStamped message representing the current goal

    in_last_goal = Bool()               # Boolean indicating whether the robot is in the last goal

    last_goal_reached = False           # Indicates if the last goal has been reached

    goal_reached = False                # Indicates if a goal has been reached

    reset = False                 # Indicates if goals need to be reset

    robot_state = -1                     # Current state of the robot

    # starts with randon value 
    ROBOT_MANUAL = 1000
    ROBOT_AUTONOMOUS = 1000
    ROBOT_INIT = 1000
    ROBOT_EMERGENCY = 1000



    def __init__(self, 
                 node_name: str, 
                 *, # keyword-only argument
                 cli_args: List[str] = None, 
                 namespace: str = None, 
                 use_global_arguments: bool = True, 
                 enable_rosout: bool = True, 
                 start_parameter_services: bool = True, 
                 parameter_overrides: List[Parameter] | None = None, 
                 allow_undeclared_parameters: True) -> None:
        
        super().__init__(node_name, 
                         cli_args=cli_args, 
                         namespace=namespace, 
                         use_global_arguments=use_global_arguments, 
                         enable_rosout=enable_rosout, 
                         start_parameter_services=start_parameter_services, 
                         parameter_overrides=parameter_overrides, 
                         allow_undeclared_parameters=allow_undeclared_parameters)
        
        subscribers.goal_provider_config(self)
        publishers.goal_provider(self)
        params.goal_provider_config(self)

        self.add_on_set_parameters_callback(params.provider_params_callback)



    # Main function to publish goals and current goal
    def main(self): 
        
        #### Publish goals array 
        goals_pose = PoseArray()
        
        goals_pose.header.stamp = self.get_clock().now().to_msg()
        goals_pose.header.frame_id = self.FRAME_ID

         # Add each goal to the pose array
        for goal_values in self.goals:
            
            x, y, theta = goal_values
            
            pose_msg = Pose()

            pose_msg.position.x = x 
            pose_msg.position.y = y
            pose_msg.position.z = theta

            pose_msg.orientation.z = 0.0
            pose_msg.orientation.w = 1.0

            goals_pose.poses.append(pose_msg)

    

        # Publish goals array and current goal
        self.goals_pub.publish(goals_pose)
        self.goalCurrent_pub.publish(self.current_goal)

        self.current_goal.header.stamp = self.get_clock().now().to_msg()
        self.current_goal.header.frame_id = self.FRAME_ID

        pose_msg = Pose()
        pose_msg.position.x, pose_msg.position.y, pose_msg.orientation.z = self.goals[self.current_index]

        self.current_goal.pose = pose_msg
        

        if debug_mode or self.DEBUG: 

            debug.goal_provider(self)


    def index_check(self):

        # Check if a new goal needs to be set       
        if (self.goal_reached > self.last_goal_reached) and self.robot_state == self.ROBOT_AUTONOMOUS:
            
            # Check if the current goal index is less than the total number of goals
            if self.current_index < (len(self.goals) - 1):
                
                self.in_last_goal.data = False

                self.current_index += 1

                self.get_logger().info(f'Goal index changed: {self.current_index}')

        # Check if the current goal is the last goal
        if self.current_index == len(self.goals) - 1:
            
            self.in_last_goal.data = True                       # Set the robot to be in the last goal
            self.get_logger().warn('Heading to last goal!!!')

        # Publish the status of whether the robot is in the last goal
        self.inLastGoal_pub.publish(self.in_last_goal)
        
        # Update the last goal reached status
        self.last_goal_reached = self.goal_reached


    
    def reset_goals(self):

        # Check if goals need to be reset
        if self.reset: 
            
            self.current_index = 0                               # Reset the current goal index to the first goal
            self.in_last_goal.data = False                       # Set the robot to not be in the last goal
            self.inLastGoal_pub.publish(self.in_last_goal)       # Publish the updated status of whether the robot is in the last goal
    


if __name__ == '__main__': 
    
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    
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

        node.get_logger().warn(' ------------------------------------ DEACTIVATING NODE --------------------------------------')
        pass

    rclpy.shutdown()
    thread.join()
