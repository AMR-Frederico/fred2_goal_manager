import rclpy

import fred2_goal_manager.scripts.qos as qos

from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseArray


commum_qos = qos.general_config()


########################################################
# --------------- GOAL PROVIDER
########################################################

def goal_provider(node: Node): 

    global commum_qos 
    
    # ----- Publish goals array 
    node.goals_pub = node.create_publisher(PoseArray, 'goals', commum_qos)

    # ----- Publish current goal 
    node.goalCurrent_pub = node.create_publisher(PoseStamped, 'goal/current', commum_qos)

    # ----- Publish flag to indicate last goal 
    node.inLastGoal_pub = node.create_publisher(Bool, 'robot/in_last_goal', commum_qos)


########################################################
# --------------- GOAL REACHED
########################################################

def goal_reached(node: Node):

    global commum_qos
    
    # ------ Flag to indicate if the robot has reached the goal 
    node.goalReached_pub = node.create_publisher(Bool, 'goal/reached', commum_qos)

    # ------ Indicates if the robot has reached the last goal 
    node.missionCompleted_pub = node.create_publisher(Bool, 'goal/mission_completed', commum_qos)
    
    # ------ Request for enable the led strip for goal sinzalization 
    node.led_on = node.create_publisher(Bool, 'goal/sinalization', commum_qos)