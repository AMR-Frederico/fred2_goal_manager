#!/usr/bin/env python3

import rclpy 
import json 

from rclpy.node import Node, ParameterDescriptor
from rclpy.parameter import Parameter, ParameterType

from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters

########################################################################################
###### GOAL PROVIDER 
########################################################################################

# updates the parameters when they are changed by the command line
def provider_params_callback(node: Node, params):  
    
    for param in params:
        node.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")


    if param.name == 'frame_id':
        node.FRAME_ID = param.value


    if param.name == 'goals': 
        node.goals = json.loads(param.value)

    
    if param.name == 'debug': 
        node.DEBUG = param.value
    

    if param.name == 'unit_test': 
        node.UNIT_TEST = param.value

    
    if param.name == 'frequency': 
        node.FREQUENCY = param.value 


    return SetParametersResult(successful=True)


def goal_provider_config(node: Node):

    # Declare parameters
    node.declare_parameters(
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
                    
            ('use_global_param', None, 
                ParameterDescriptor(
                    description='Allow the node to run isolated for unit testing',
                    type=ParameterType.PARAMETER_BOOL)),
            
            ('frequency', None, 
                ParameterDescriptor(
                    description='Node frequency', 
                    type=ParameterType.PARAMETER_INTEGER)),
        ]
    )
        
    # Fetch the parameters
    node.FRAME_ID = node.get_parameter('frame_id').value
    node.DEBUG = node.get_parameter('debug').value
    node.GLOBAL_PARAM = node.get_parameter('use_global_param').value
    node.FREQUENCY = node.get_parameter('frequency').value

    # Get goals as a string 
    goals_str = node.get_parameter('goals').value
    
    # Decode the JSON string to get the actual list structure for goals
    node.goals = json.loads(goals_str)

    if node.GLOBAL_PARAM: 

        global_params(node)


########################################################################################
###### GOAL REACHED 
########################################################################################



# updates the parameters when they are changed by the command line
def reached_parameters_callback(node: Node, params):
    
    for param in params:
        node.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")


    if param.name == 'debug': 
        node.DEBUG = param.value


    if param.name == 'use_global_param': 
        node.GLOBAL_PARAM = param.value


    if param.name == 'waypoint_goal_tolerance': 
        node.WAYPOINT_GOAL_TOLERANCE = param.value 

    
    if param.name == 'high_accuracy': 
        node.HIGH_ACCURACY_TOLERANCE = param.value

    
    if param.name == 'normal_accuracy': 
        node.NORMAL_ACCURACY_TOLERANCE = param.value


    if param.name == 'low_accuracy': 
        node.LOW_ACCURACY_TOLERANCE = param.value
    

    if param.name == 'frequency': 
        node.FREQUENCY = param.value 


    return SetParametersResult(successful=True)



def goal_reached_config(node: Node):

    # Declare parameters related to goal-reaching tolerances and debugging/testing
    node.declare_parameters(
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
            
            ('use_global_param', None, 
                ParameterDescriptor(
                    description='Allow the node to run isolated for unit testing',
                    type=ParameterType.PARAMETER_BOOL)),
            
            ('frequency', None, 
                ParameterDescriptor(
                    description='Node frequency', 
                    type=ParameterType.PARAMETER_INTEGER)),
        ]
    )


    node.DEBUG = node.get_parameter('debug').value
    node.GLOBAL_PARAM = node.get_parameter('use_global_param').value
    node.FREQUENCY = node.get_parameter('frequency').value

    node.HIGH_ACCURACY_TOLERANCE = node.get_parameter('high_accuracy').value
    node.NORMAL_ACCURACY_TOLERANCE = node.get_parameter('normal_accuracy').value
    node.LOW_ACCURACY_TOLERANCE = node.get_parameter('low_accuracy').value 

    node.WAYPOINT_GOAL_TOLERANCE = node.get_parameter('waypoint_goal_tolerance').value


    if node.GLOBAL_PARAM: 

        global_params(node)


########################################################################################
###### GLOBAL PARAMS 
########################################################################################

def global_params(node: Node): 

    # Get global params 
    node.client = node.create_client(GetParameters, '/main_robot/operation_modes/get_parameters')
    node.client.wait_for_service()

    request = GetParameters.Request()
    request.names = ['init', 'manual', 'autonomous', 'emergency']

    future = node.client.call_async(request)
    future.add_done_callback(lambda future: callback_global_param(node, future))


    node.get_logger().info('Global params are deactivated')  
        


# get the global values from the machine states params 
def callback_global_param(node: Node, future):


    try:

        result = future.result()

        node.ROBOT_INIT = result.values[0].integer_value
        node.ROBOT_MANUAL = result.values[1].integer_value
        node.ROBOT_AUTONOMOUS = result.values[2].integer_value
        node.ROBOT_EMERGENCY = result.values[3].integer_value

        node.get_logger().info(f"Got global param ROBOT_EMERGENCY: {node.ROBOT_EMERGENCY}\n")
        node.get_logger().info(f"Got global param ROBOT_INIT -> {node.ROBOT_INIT}")
        node.get_logger().info(f"Got global param ROBOT_MANUAL -> {node.ROBOT_MANUAL}")
        node.get_logger().info(f"Got global param ROBOT_AUTONOMOUS -> {node.ROBOT_AUTONOMOUS}")



    except Exception as e:

        node.get_logger().warn("Service call failed %r" % (e,))


