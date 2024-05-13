#!/usr/bin/env python3

import rclpy 

from rclpy.node import Node

import sys
import rclpy
import threading

from typing import List, Optional

from rclpy.context import Context 
from rclpy.node import Node, ParameterDescriptor
from rclpy.parameter import Parameter, ParameterType
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters

from math import hypot

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Int16


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
    
    ghost_goal_tolerance = 0.0                    # Accuracy level for reaching the goal


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
        
        self.quality_protocol()
        self.setup_publishers()
        self.setup_subscribers()

        self.load_params()
        self.get_params()

        self.add_on_set_parameters_callback(self.parameters_callback)



        
    def quality_protocol(self):

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Set the reliability policy to RELIABLE, ensuring reliable message delivery
            durability= QoSDurabilityPolicy.VOLATILE,   # Set the durability policy to VOLATILE, indicating messages are not stored persistently
            history=QoSHistoryPolicy.KEEP_LAST,         # Set the history policy to KEEP_LAST, storing a limited number of past messages
            depth=10,                                   # Set the depth of the history buffer to 10, specifying the number of stored past messages
            liveliness=QoSLivelinessPolicy.AUTOMATIC    # Set the liveliness policy to AUTOMATIC, allowing automatic management of liveliness
            
        )



    def setup_subscribers(self):
        
        # ------- Receives the current goal 
        self.create_subscription(PoseStamped, 
                                 'goal/current', 
                                 self.goalCurrent_callback, 
                                 self.qos_profile)
        
        # ------- Current robot state 
        self.create_subscription(Int16,
                                 '/machine_states/robot_state',
                                 self.robot_state_callback, 
                                 self.qos_profile)
        
        # ------- Flag to indicate if the robot is heading to the last goal 
        self.create_subscription(Bool, 
                                 'robot/in_last_goal', 
                                 self.lastGoal_callback, 
                                 self.qos_profile)
        
        # ------- Current robot pose according to odom        
        self.create_subscription(Odometry,
                                '/odom', 
                                self.odom_callback, 
                                self.qos_profile)



    def setup_publishers(self):
        
        # ------ Flag to indicate if the robot has reached the goal 
        self.goalReached_pub = self.create_publisher(Bool, 
                                                     'goal/reached', 
                                                     self.qos_profile)
        
        # ------ Indicates if the robot has reached the last goal 
        self.missionCompleted_pub = self.create_publisher(Bool, 
                                                          'goal/mission_completed', 
                                                          self.qos_profile)
        
        # ------ Request for enable the led strip for goal sinzalization 
        self.led_on = self.create_publisher(Bool, 
                                            'goal/sinalization', 
                                            self.qos_profile)



    # updates the parameters when they are changed by the command line
    def parameters_callback(self, params):
        
        for param in params:
            self.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")


        if param.name == 'debug': 
            self.DEBUG = param.value


        if param.name == 'unit_test': 
            self.UNIT_TEST = param.value


        if param.name == 'waypoint_goal_tolerance': 
            self.WAYPOINT_GOAL_TOLERANCE = param.value 

        
        if param.name == 'high_accuracy': 
            self.HIGH_ACCURACY_TOLERANCE = param.value
    
        
        if param.name == 'normal_accuracy': 
            self.NORMAL_ACCURACY_TOLERANCE = param.value


        if param.name == 'low_accuracy': 
            self.LOW_ACCURACY_TOLERANCE = param.value
        

        if param.name == 'frequency': 
            self.FREQUENCY = param.value 


        return SetParametersResult(successful=True)



    def load_params(self):
        # Declare parameters related to goal-reaching tolerances and debugging/testing
        self.declare_parameters(
            namespace='',
            parameters=[

                ('high_accuracy', None, 
                    ParameterDescriptor(
                        description='High level of accuracy for determining if the robot has reached its goal, in meters',
                        type=ParameterType.PARAMETER_DOUBLE)),

                ('normal_accuracy', None, 
                    ParameterDescriptor(
                        description='Normal level of accuracy for determining if the robot has reached its goal, in meters',
                        type=ParameterType.PARAMETER_DOUBLE)),

                ('low_accuracy', None, 
                    ParameterDescriptor(
                        description='Low level of accuracy for determining if the robot has reached its goal, in meters',
                        type=ParameterType.PARAMETER_DOUBLE)),
                        
                ('waypoint_goal_tolerance', None, 
                    ParameterDescriptor(
                        description='Tolerance for determining if the robot has reached its waypoint goal, in meters',
                        type=ParameterType.PARAMETER_DOUBLE)),

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


    # Get parameters value 
    def get_params(self):
        
        self.DEBUG = self.get_parameter('debug').value
        self.UNIT_TEST = self.get_parameter('unit_test').value
        self.FREQUENCY = self.get_parameter('frequency').value

        self.HIGH_ACCURACY_TOLERANCE = self.get_parameter('high_accuracy').value
        self.NORMAL_ACCURACY_TOLERANCE = self.get_parameter('normal_accuracy').value
        self.LOW_ACCURACY_TOLERANCE = self.get_parameter('low_accuracy').value 

        self.WAYPOINT_GOAL_TOLERANCE = self.get_parameter('waypoint_goal_tolerance').value


        if self.UNIT_TEST: 

            self.robot_state = self.ROBOT_AUTONOMOUS
        
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


    # Get robot current position 
    def odom_callback(self, odom_msg): 

        self.robot.position.x = odom_msg.pose.pose.position.x 
        self.robot.position.y = odom_msg.pose.pose.position.y 
        self.robot.orientation.z = odom_msg.pose.pose.orientation.z 



    def goalCurrent_callback(self, goal_msg): 
        
        # Update the goal position based on the received message
        self.goal.position.x = goal_msg.pose.position.x 
        self.goal.position.y = goal_msg.pose.position.y


        # Determine the accuracy level and whether the goal is a waypoint based on the orientation
        if goal_msg.pose.orientation.z == 1.0: 

            self.ghost_goal_tolerance = self.LOW_ACCURACY_TOLERANCE
            self.waypoint_goal = False

        
        elif goal_msg.pose.orientation.z == 2.0: 

            self.ghost_goal_tolerance = self.NORMAL_ACCURACY_TOLERANCE
            self.waypoint_goal = False

        elif goal_msg.pose.orientation.z == 3.0: 

            self.ghost_goal_tolerance = self.HIGH_ACCURACY_TOLERANCE
            self.waypoint_goal = False

        else: 
            print('no else')
            self.waypoint_goal = True


    # Get current robot state 
    def robot_state_callback(self, robot_state): 
        
        self.robot_state = robot_state.data


    # Flag to indicate if the robot is heading to the last goal 
    def lastGoal_callback(self, status): 

        self.last_goal = status.data


    # Main function responsible for checking if the robot has reached the goal or completed the mission.
    def main(self): 
        
        # Calculate linear error between robot and goal positions
        dx = self.goal.position.x - self.robot.position.x 
        dy = self.goal.position.y - self.robot.position.y 

        linear_error = hypot(dx, dy)

        # Set default values
        self.robot_in_goal.data = False
        self.mission_completed.data = False
        self.sinalization_msg.data = False


        # Check if the robot is in autonomous mode
        if self.robot_state == self.ROBOT_AUTONOMOUS: 
            
            # Check if the goal is a waypoint goal
            if self.waypoint_goal: 
                
                # Check if the robot has reached the waypoint goal
                if (linear_error < self.WAYPOINT_GOAL_TOLERANCE) and not self.last_goal: 
                    
                    self.get_logger().info('WAYPOINT goal reachead !!!!!!!! ')
                    
                    self.robot_in_goal.data = True

                    self.sinalization_msg.data = True
                    self.led_on.publish(self.sinalization_msg)
                

                # Check if the robot has reached the final waypoint goal
                elif (linear_error < self.WAYPOINT_GOAL_TOLERANCE) and self.last_goal: 
                    
                    self.mission_completed.data = True 

                    self.get_logger().info('Mission Completed !!!!')
                    

            else:   # for ghost goal 
                
                # Disable the LED sinalization
                self.sinalization_msg.data = False 
                self.led_on.publish(self.sinalization_msg)

                # Check if the goal is a ghost goal
                if (linear_error < self.ghost_goal_tolerance): 
                    
                    self.robot_in_goal.data = True

                    self.get_logger().info('GHOST goal reached !!!!!')


        # Publish status messages
        self.missionCompleted_pub.publish(self.mission_completed)
        self.goalReached_pub.publish(self.robot_in_goal)
        self.led_on.publish(self.sinalization_msg)



        if debug_mode or self.DEBUG: 

            self.get_logger().info(f'Goal X: {self.goal.position.x} | Goal Y: {self.goal.position.y} | Robo in goal: {self.robot_in_goal.data}')
            self.get_logger().info(f'Delta X: {dx} | Delta Y: {dy} | Linear error: {linear_error}')
            self.get_logger().info(f'Ghost goal: {not self.waypoint_goal} | Tolerance: {self.ghost_goal_tolerance}')
            self.get_logger().info(f'Waypoint goal: {self.waypoint_goal} | Tolerance: {self.WAYPOINT_GOAL_TOLERANCE}\n')



if __name__ == '__main__': 
    rclpy.init()
    node = goal_reached(
        node_name='goal_reached', 
        namespace='goal_manager', 
        cli_args=['--debug'] )

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
