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
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters

from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import Bool, Int16


# Parameters file (yaml)
node_path = '~/ros2_ws/src/fred2_goal_manager/conf/goal_manager.yaml'
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

    reset_goals = False

    robot_state = 0


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


        # quality protocol -> the node can't lose any message 
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, 
            durability= QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, 
            depth=10, 
            liveliness=QoSLivelinessPolicy.AUTOMATIC
            
        )


        self.create_subscription(Bool, 
                                 'goal/reached', 
                                 self.goalReached_callback, 
                                 qos_profile)
        

        self.create_subscription(Bool, 
                                 '/odom/reset', 
                                 self.reset_callback, 
                                 qos_profile)
        

        self.create_subscription(Int16, 
                                 '/machine_states/robot_state',
                                 self.robotState_callback, 
                                 5)


        self.goals_pub = self.create_publisher(PoseArray, 
                                               'goals', 
                                               5)


        self.goalCurrent_pub = self.create_publisher(PoseStamped, 
                                                     'goal/current', 
                                                     5)


        self.missionCompleted_pub = self.create_publisher(Bool, 
                                                          'goal/mission_completed', 
                                                          5)


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



    def robotState_callback(self, msg): 

        self.robot_state = msg.data




    def main(self): 
        
        #### Publish goals array 
        
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

    

        ##### Publish current goal 

        self.goals_pub.publish(goals_pose)
        self.goalCurrent_pub.publish(self.current_goal)

        print(self.goals_array[self.current_index])

        self.current_goal.header.stamp = self.get_clock().now().to_msg()
        self.current_goal.header.frame_id = self.FRAME_ID

        pose_msg = Pose()
        pose_msg.position.x, pose_msg.position.y, pose_msg.orientation.z = self.goals_array[self.current_index]

        self.current_goal.pose = pose_msg
        

        if debug_mode: 

            self.get_logger().info(f'Current goal: {self.current_goal.pose.position.x}, {self.current_goal.pose.position.y}, {self.current_goal.pose.orientation.z}| Index: {self.current_index} | Number of goals: {len(self.goals_array)} ')
            self.get_logger().info(f'Goal reached: {self.goal_reached} | Last status: {self.last_goal_reached} | Update goal: {self.goal_reached > self.last_goal_reached} \n')
            self.get_logger().info(f'Goal reset: {self.reset_goals}')



    def goalReached_callback(self, current_status):
        

        self.goal_reached = current_status.data


        if (self.goal_reached > self.last_goal_reached) and self.robot_state == self.ROBOT_AUTONOMOUS:

            
            if self.current_index < (len(self.goals_array) - 1):
                
                self.mission_completed.data = False
                self.missionCompleted_pub.publish(self.mission_completed)

                self.current_index += 1

                self.get_logger().info(f'Goal index changed: {self.current_index}')

                
            if self.current_index == len(self.goals_array) - 1:
                
                self.mission_completed.data = True
                self.missionCompleted_pub.publish(self.mission_completed)
                self.get_logger().warn('Mission Completed!!!')


        self.last_goal_reached = self.goal_reached




    
    def reset_callback(self, reset_msg):
        
        self.reset_goals = reset_msg.data

        if self.reset_goals: 
            
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




if __name__ == '__main__': 
    
    rclpy.init()
    
    node = goal_provider('goal_provider', 
                         namespace='goal_manager', 
                         cli_args=['--debug'], 
                         start_parameter_services= True, 
                         allow_undeclared_parameters= True)
    
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
