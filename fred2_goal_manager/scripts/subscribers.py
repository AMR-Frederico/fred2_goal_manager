#!/usr/bin/env python3

import rclpy
import fred2_goal_manager.scripts.qos as qos 
from fred2_goal_manager.scripts.goal_provider import goal_provider 
from rclpy.node import Node 

from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

########################################################
# --------------- Set subscribers 
########################################################

commum_qos = qos.general_config()

def goal_provider_config(node: Node): 
    
    global commum_qos

    # ----- Indicate if the robot has reached the goal 
    node.create_subscription(Bool, 
                                 'goal/reached', 
                                 lambda msg: goalReached_callback(node, msg), 
                                 commum_qos)
    
    # ----- Commant for reset odometry, and goal index 
    node.create_subscription(Bool, 
                                 '/odom/reset', 
                                 lambda msg: reset_callback(node, msg), 
                                 commum_qos)
        
    # ----- Get current robot state 
    node.create_subscription(Int16, 
                                '/machine_states/robot_state',
                                lambda msg: robotState_callback(node, msg), 
                                commum_qos)


def goal_reached_config(node: Node):

    global commum_qos
    
    # ------- Receives the current goal 
    node.create_subscription(PoseStamped, 
                                'goal/current', 
                                lambda msg: goalCurrent_callback(node, msg), 
                                commum_qos)
    
    # ------- Current robot state 
    node.create_subscription(Int16,
                                '/machine_states/robot_state',
                                lambda msg: robotState_callback(node, msg), 
                                commum_qos)
    
    # ------- Flag to indicate if the robot is heading to the last goal 
    node.create_subscription(Bool, 
                                'robot/in_last_goal', 
                                lambda msg: lastGoal_callback(node, msg), 
                                commum_qos)
    
    # ------- Current robot pose according to odom        
    node.create_subscription(Odometry,
                            '/odom', 
                            lambda msg: odom_callback(node, msg), 
                            commum_qos)


    
########################################################
# ------------- Get callbacks 
########################################################


# Get current robot state
def robotState_callback(node, msg): 

    node.robot_state = msg.data


# Goal reached status 
def goalReached_callback(node, current_status):    

    node.goal_reached = current_status.data
    goal_provider.index_check(node)


# Reset goals 
def reset_callback(node, reset_msg):
    
    node.reset = reset_msg.data
    goal_provider.reset_goals(node)


# Get robot current position 
def odom_callback(node: Node, odom_msg): 

    node.robot.position.x = odom_msg.pose.pose.position.x 
    node.robot.position.y = odom_msg.pose.pose.position.y 
    node.robot.orientation.z = odom_msg.pose.pose.orientation.z 



def goalCurrent_callback(node:Node, goal_msg): 
    
    # Update the goal position based on the received message
    node.goal.position.x = goal_msg.pose.position.x 
    node.goal.position.y = goal_msg.pose.position.y


    # Determine the accuracy level and whether the goal is a waypoint based on the orientation
    if goal_msg.pose.orientation.z == 1.0: 

        node.ghost_goal_tolerance = node.LOW_ACCURACY_TOLERANCE
        node.waypoint_goal = False

    
    elif goal_msg.pose.orientation.z == 2.0: 

        node.ghost_goal_tolerance = node.NORMAL_ACCURACY_TOLERANCE
        node.waypoint_goal = False

    elif goal_msg.pose.orientation.z == 3.0: 

        node.ghost_goal_tolerance = node.HIGH_ACCURACY_TOLERANCE
        node.waypoint_goal = False



# Flag to indicate if the robot is heading to the last goal 
def lastGoal_callback(node: Node, status): 

    node.last_goal = status.data