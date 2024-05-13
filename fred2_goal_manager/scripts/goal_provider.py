#!/usr/bin/env python3

import sys
import json
import rclpy
import threading

from typing import List, Optional
from rclpy.context import Context 
from rclpy.node import Node, ParameterDescriptor
from rclpy.parameter import Parameter, ParameterType
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters

from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import Bool, Int16


debug_mode = '--debug' in sys.argv


class goal_provider(Node): 
    
    goal_reached = False                # Indicates if the goal has been reached
    
    goals = []
    current_index = 0                   # Index of the current goal in the list of goals

    current_goal = PoseStamped()        # PoseStamped message representing the current goal

    in_last_goal = Bool()               # Boolean indicating whether the robot is in the last goal

    last_goal_reached = False           # Indicates if the last goal has been reached

    goal_reached = False                # Indicates if a goal has been reached

    reset_goals = False                 # Indicates if goals need to be reset

    robot_state = -1                     # Current state of the robot

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
        self.load_params()
        self.get_params()
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.quality_protocol()
        self.setup_publishers()
        self.setup_subscribers()



    def quality_protocol(self):

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Set the reliability policy to RELIABLE, ensuring reliable message delivery
            durability= QoSDurabilityPolicy.VOLATILE,   # Set the durability policy to VOLATILE, indicating messages are not stored persistently
            history=QoSHistoryPolicy.KEEP_LAST,         # Set the history policy to KEEP_LAST, storing a limited number of past messages
            depth=10,                                   # Set the depth of the history buffer to 10, specifying the number of stored past messages
            liveliness=QoSLivelinessPolicy.AUTOMATIC    # Set the liveliness policy to AUTOMATIC, allowing automatic management of liveliness
            
        )



    def setup_subscribers(self):

        # ------ Checks if the robot reached the goal
        self.create_subscription(Bool, 
                                 'goal/reached', 
                                 self.goalReached_callback, 
                                 self.qos_profile)
        
        # ------- Request for reset odometry 
        self.create_subscription(Bool, 
                                 '/odom/reset', 
                                 self.reset_callback, 
                                 self.qos_profile)
        
        # ------- Current robot state 
        self.create_subscription(Int16, 
                                 '/machine_states/robot_state',
                                 self.robotState_callback, 
                                 self.qos_profile)

    def setup_publishers(self):

        # ------- For visualization, publishes all the goals 
        self.goals_pub = self.create_publisher(PoseArray, 
                                               'goals', 
                                               self.qos_profile)

        # ------- Indicates the current goal 
        self.goalCurrent_pub = self.create_publisher(PoseStamped, 
                                                     'goal/current', 
                                                     self.qos_profile)

        # ------- Indicates when the robot reaches the laste goal 
        self.inLastGoal_pub = self.create_publisher(Bool, 
                                                    'robot/in_last_goal', 
                                                    self.qos_profile)


    

    # updates the parameters when they are changed by the command line
    def parameters_callback(self, params):  
        
        for param in params:
            self.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")


        if param.name == 'frame_id':
            self.FRAME_ID = param.value


        if param.name == 'goals': 
            self.goals = json.loads(param.value)
    
        
        if param.name == 'debug': 
            self.DEBUG = param.value
        

        if param.name == 'unit_test': 
            self.UNIT_TEST = param.value

        
        if param.name == 'frequency': 
            self.FREQUENCY = param.value 


        return SetParametersResult(successful=True)



    # Get robot current state 
    def robotState_callback(self, msg): 

        self.robot_state = msg.data


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
            
            pose_msg.orientation.z = theta
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

            self.get_logger().info(f'Current goal: {self.current_goal.pose.position.x}, {self.current_goal.pose.position.y}, {self.current_goal.pose.orientation.z}| Index: {self.current_index} | Number of goals: {len(self.goals)} ')
            self.get_logger().info(f'Goal reached: {self.goal_reached} | Last status: {self.last_goal_reached} | Update goal: {self.goal_reached > self.last_goal_reached} \n')
            self.get_logger().info(f'Goal reset: {self.reset_goals}')



    def goalReached_callback(self, current_status):
        
        # Update the status of whether a goal has been reached
        self.goal_reached = current_status.data


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


    
    def reset_callback(self, reset_msg):
        
        self.reset_goals = reset_msg.data

        # Check if goals need to be reset
        if self.reset_goals: 
            
            self.current_index = 0                               # Reset the current goal index to the first goal
            self.in_last_goal.data = False                       # Set the robot to not be in the last goal
            self.inLastGoal_pub.publish(self.in_last_goal)       # Publish the updated status of whether the robot is in the last goal
    

    def load_params(self):

        # Declare parameters
        self.declare_parameters(
            namespace='',
            
            parameters=[
                ('frame_id', None, 
                    ParameterDescriptor(
                        description='The frame of reference for the goals',
                        type=ParameterType.PARAMETER_STRING)),

                ('goals', None, 
                    ParameterDescriptor(
                        description='List of goals, each as [x, y, theta]',
                        type=ParameterType.PARAMETER_STRING)),

                ('debug', None, 
                    ParameterDescriptor(
                        description='Enable debug prints for troubleshooting',
                        type=ParameterType.PARAMETER_BOOL)),
                        
                ('unit_test', None, 
                    ParameterDescriptor(
                        description='Allow the node to run isolated for unit testing',
                        type=ParameterType.PARAMETER_BOOL)),
                
                ('frequency', None, 
                    ParameterDescriptor(
                        description='Node frequency', 
                        type=ParameterType.PARAMETER_INTEGER)),
            ]
        )


    def get_params(self):
        
        # Fetch the parameters
        self.FRAME_ID = self.get_parameter('frame_id').value
        self.DEBUG = self.get_parameter('debug').value
        self.UNIT_TEST = self.get_parameter('unit_test').value
        self.FREQUENCY = self.get_parameter('frequency').value

        # Get goals as a string 
        goals_str = self.get_parameter('goals').value
        
        # Decode the JSON string to get the actual list structure for goals
        self.goals = json.loads(goals_str)

        
        # if the unit test is active, it disabled the global param from machine states 
        if self.UNIT_TEST: 
            
            self.robot_state = 2
            self.get_logger().info('In UNIT TEST mode')  

        
        else: 

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

    rate = node.create_rate(node.FREQUENCY)

    try: 
        while rclpy.ok(): 
            node.main()
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
