#!/usr/bin/env python3

import rclpy 

import sys
import rclpy
import threading

import fred2_goal_manager.scripts.debug as debug 
import fred2_goal_manager.scripts.parameters as param 
import fred2_goal_manager.scripts.publishers as publishers 
import fred2_goal_manager.scripts.subscribers as subscribers 

from typing import List

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

from math import hypot

from geometry_msgs.msg import Pose
from std_msgs.msg import Bool


debug_mode = '--debug' in sys.argv


class goal_reached(Node): 
    
    robot = Pose()                          # Variables to hold robot and goal positions
    goal = Pose()

    robot_in_goal = Bool()                  # Boolean to indicate if the robot is in goal
    robot_in_goal.data = False

    robot_state = 100                       # Variable to hold robot state, starts in a random value 

    last_goal = False                       # Flag to indicate if the last goal has been reached

    mission_completed = Bool()              # Boolean to indicate if the mission is completed
    mission_completed.data = False

    sinalization_msg = Bool()               # Message for LED signalization

    waypoint_goal = False                   # Flag to indicate if the goal is a waypoint
    
    tolerance = 0.3                    # Accuracy level for reaching the goal


    # starts with randon value 
    ROBOT_MANUAL = 1000
    ROBOT_AUTONOMOUS = 1000
    ROBOT_INIT = 1000
    ROBOT_EMERGENCY = 1000

    # Ghost goal tolerance 
    HIGH_TOLERANCE = 3.0 
    NORMAL_TOLERANCE = 2.0 
    LOW_TOLERANCE = 1.0



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
        
        subscribers.goal_reached_config(self)
        publishers.goal_reached(self)
        param.goal_reached_config(self)

        self.add_on_set_parameters_callback(param.reached_parameters_callback)


    # Main function responsible for checking if the robot has reached the goal or completed the mission.
    def main(self): 
        
        # Calculate linear error between robot and goal positions
        self.dx = self.goal.position.x - self.robot.position.x 
        self.dy = self.goal.position.y - self.robot.position.y 

        self.linear_error = hypot(self.dx, self.dy)

        # Set default values
        self.robot_in_goal.data = False
        self.mission_completed.data = False
        self.sinalization_msg.data = False
        self.waypoint_goal = (self.goal.orientation.z == 0.0)

        # Check if the robot is in autonomous mode
        if self.robot_state == self.ROBOT_AUTONOMOUS: 
            
            # Check if the goal is a waypoint goal
            if self.waypoint_goal: 
                
                # Check if the robot has reached the waypoint goal
                if (self.linear_error < self.WAYPOINT_GOAL_TOLERANCE) and not self.last_goal: 
                    
                    self.get_logger().info('WAYPOINT goal reachead !!!!!!!! ')
                    
                    self.robot_in_goal.data = True

                    self.sinalization_msg.data = True
                    self.led_on.publish(self.sinalization_msg)
                

                # Check if the robot has reached the final waypoint goal
                elif (self.linear_error < self.WAYPOINT_GOAL_TOLERANCE) and self.last_goal: 
                    
                    self.mission_completed.data = True 

                    self.get_logger().info('Mission Completed !!!!')
                    

            else:   # for ghost goal 

                # Disable the LED sinalization
                self.sinalization_msg.data = False 
                self.led_on.publish(self.sinalization_msg)

                goal_accurancy = self.goal.orientation.z
                self.get_logger().info(f'{goal_accurancy}')

                if goal_accurancy == self.HIGH_TOLERANCE: 
                        
                    self.tolerance = self.HIGH_ACCURACY_TOLERANCE
                
                elif goal_accurancy == self.NORMAL_TOLERANCE: 
                    
                    self.tolerance = self.NORMAL_ACCURACY_TOLERANCE 
                
                elif goal_accurancy == self.LOW_TOLERANCE: 
                        
                    self.tolerance = self.LOW_ACCURACY_TOLERANCE


                # Check if the goal is a ghost goal
                if (self.linear_error < self.tolerance): 
                    
                    self.robot_in_goal.data = True

                    self.get_logger().info('GHOST goal reached !!!!!')


        # Publish status messages
        self.missionCompleted_pub.publish(self.mission_completed)
        self.goalReached_pub.publish(self.robot_in_goal)
        self.led_on.publish(self.sinalization_msg)



        if debug_mode or self.DEBUG: 
            debug.goal_reached(self)



if __name__ == '__main__': 

    rclpy.init(args = None, signal_handler_options = SignalHandlerOptions.NO)

    node = goal_reached(
        node_name='goal_reached', 
        namespace='goal_manager', 
        cli_args=['--debug'] )

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
