#!/usr/bin/env python3

import rclpy 

from rclpy.node import Node

import os
import sys
import yaml
import rclpy
import threading

from typing import List, Optional

from rclpy.node import Node
from rclpy.context import Context 
from rclpy.parameter import Parameter
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters

from math import hypot

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Int16

# Parameters file (yaml)
node_path = '/home/ubuntu/ros2_ws/src/fred2_goal_manager/conf/goal_manager.yaml'
node_group = 'goal_reached'


debug_mode = '--debug' in sys.argv




class goal_reached(Node): 
    
    robot = Pose()
    goal = Pose()

    robot_in_goal = Bool()
    robot_in_goal.data = False

    robot_state = 100

    last_goal = False

    mission_completed = Bool()
    mission_completed.data = False


    sinalization_msg = Bool()

    # niveis de precisão dos ghost goals
    HIGH_ACCURACY = 0.15 
    NORMAL_ACCURACY = 0.30
    LOW_ACCURACY = 0.60

    waypoint_goal = False
    accuracy_level = NORMAL_ACCURACY


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
        
        
        # quality protocol -> the node must not lose any message 
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, 
            durability= QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, 
            depth=10, 
            liveliness=QoSLivelinessPolicy.AUTOMATIC
            
        )


        self.create_subscription(PoseStamped, 
                                 'goal/current', 
                                 self.goalCurrent_callback, 
                                 qos_profile)
        

        self.create_subscription(Int16,
                                 '/machine_states/robot_state',
                                 self.robot_state_callback, 
                                 qos_profile)
        
        self.create_subscription(Bool, 
                                 'robot/in_last_goal', 
                                 self.lastGoal_callback, 
                                 qos_profile)
        
                    
        self.create_subscription(Odometry,
                                '/odom', 
                                self.odom_callback, 
                                qos_profile)


        self.goalReached_pub = self.create_publisher(Bool, 
                                                     'goal/reached', 
                                                     qos_profile)
        

        self.missionCompleted_pub = self.create_publisher(Bool, 
                                                          'goal/mission_completed', 
                                                          qos_profile)

        self.led_on = self.create_publisher(Bool, 
                                            'goal/sinalization', 
                                            qos_profile)



        self.load_params(node_path, node_group)
        self.get_params()


        self.add_on_set_parameters_callback(self.parameters_callback)



    def parameters_callback(self, params):
        
        for param in params:
            self.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")



        if param.name == 'robot_in_goal_tolerence':
            self.ROBOT_IN_GOAL_TOLERANCE = param.value

        if param.name == 'debug': 
            self.DEBUG = param.value


    


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
        self.DEBUG = self.get_parameter('debug').value



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

        if goal_msg.pose.orientation.z == 1.0: 

            self.accuracy_level = self.LOW_ACCURACY
            self.waypoint_goal = False

        
        elif goal_msg.pose.orientation.z == 2.0: 

            self.accuracy_level = self.NORMAL_ACCURACY
            self.waypoint_goal = False

        elif goal_msg.pose.orientation.z == 3.0: 

            self.accuracy_level = self.HIGH_ACCURACY
            self.waypoint_goal = False

        else: 

            self.waypoint_goal = True


    def robot_state_callback(self, robot_state): 
        
        self.robot_state = robot_state.data



    def lastGoal_callback(self, status): 

        self.last_goal = status.data



    def main(self): 
        
        dx = self.goal.position.x - self.robot.position.x 
        dy = self.goal.position.y - self.robot.position.y 

        linear_error = hypot(dx, dy)

        if self.waypoint_goal: 


            if (linear_error < self.ROBOT_IN_GOAL_TOLERANCE) and self.robot_state == self.ROBOT_AUTONOMOUS and not self.last_goal: 
                
                self.robot_in_goal.data = True

                self.get_logger().warn('Goal Reached!!!!')

                self.sinalization_msg.data = True
                self.led_on.publish(self.sinalization_msg)


            elif (linear_error < self.ROBOT_IN_GOAL_TOLERANCE) and self.robot_state == self.ROBOT_AUTONOMOUS and self.last_goal: 

                
                
                self.mission_completed.data = True

                self.get_logger().info('Mission Completed !!!!')
                

            else: 

                
                self.robot_in_goal.data = False

                self.mission_completed.data = False

                self.sinalization_msg.data = False
        
        
        else: 

            self.sinalization_msg.data = False

            if (linear_error < self.accuracy_level) and self.robot_state == self.ROBOT_AUTONOMOUS and not self.last_goal: 
                
                self.robot_in_goal.data = True

                self.get_logger().info('Goal Reached!!!!')



            else: 
                
                self.robot_in_goal.data = False

                self.mission_completed.data = False




        self.missionCompleted_pub.publish(self.mission_completed)

        self.goalReached_pub.publish(self.robot_in_goal)

        self.led_on.publish(self.sinalization_msg)



        if debug_mode or self.DEBUG: 
            self.get_logger().info(f'Ghost goal: {not self.waypoint_goal} | Accurancy level: {self.accuracy_level}')
            self.get_logger().info(f'Goal X: {self.goal.position.x} | Goal Y: {self.goal.position.y} ')
            self.get_logger().info(f'Delta X: {dx} | Delta Y: {dy}')
            self.get_logger().info(f'Linear error: {linear_error} | Tolerance: {self.ROBOT_IN_GOAL_TOLERANCE} | Robo in goal: {self.robot_in_goal.data}\n')



if __name__ == '__main__': 
    rclpy.init()
    node = goal_reached(
        node_name='goal_reached', 
        namespace='goal_manager', 
        cli_args=['--debug'] )

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(7)

    try: 
        while rclpy.ok(): 
            node.main()
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
