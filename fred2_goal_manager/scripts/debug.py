import rclpy 

from rclpy.node import Node

def goal_provider(node: Node): 
            
    node.get_logger().info(
        
        f'Current goal -> X: {node.current_goal.pose.position.x}  Y: {node.current_goal.pose.position.y}, Theta: {node.current_goal.pose.orientation.z}| Index: {node.current_index} | Number of goals: {len(node.goals)} | Goal reached: {node.goal_reached} | Last status: {node.last_goal_reached} | Update goal: {node.goal_reached > node.last_goal_reached} | Goal reset: {node.reset}'
    )



def goal_reached(node: Node): 

    node.get_logger().info(
        
        f'Goal ->  X: {node.goal.position.x}  Y: {node.goal.position.y} | Robo in goal: {node.robot_in_goal.data} | Delta ->  X: {node.dx}  Y: {node.dy} | Linear error: {node.linear_error} | Ghost goal: {not node.waypoint_goal} Tolerance: {node.tolerance} | Waypoint goal: {node.waypoint_goal} Tolerance: {node.WAYPOINT_GOAL_TOLERANCE}'
    )
