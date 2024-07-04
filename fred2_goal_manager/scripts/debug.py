import rclpy 

from rclpy.node import Node

def goal_provider(node: Node): 
            
    node.get_logger().info(f'Current goal: {node.current_goal.pose.position.x}, {node.current_goal.pose.position.y}, {node.current_goal.pose.orientation.z}| Index: {node.current_index} | Number of goals: {len(node.goals)} ')
    node.get_logger().info(f'Goal reached: {node.goal_reached} | Last status: {node.last_goal_reached} | Update goal: {node.goal_reached > node.last_goal_reached} \n')
    node.get_logger().info(f'Goal reset: {node.reset}')



def goal_reached(node: Node): 

    node.get_logger().info(f'Goal X: {node.goal.position.x} | Goal Y: {node.goal.position.y} | Robo in goal: {node.robot_in_goal.data}')
    node.get_logger().info(f'Delta X: {node.dx} | Delta Y: {node.dy} | Linear error: {node.linear_error}')
    node.get_logger().info(f'Ghost goal: {not node.waypoint_goal} | Tolerance: {node.tolerance}')
    node.get_logger().info(f'Waypoint goal: {node.waypoint_goal} | Tolerance: {node.WAYPOINT_GOAL_TOLERANCE}\n')
