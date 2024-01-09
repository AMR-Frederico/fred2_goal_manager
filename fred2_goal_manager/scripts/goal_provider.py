import os
import yaml
import rclpy
import threading

from rclpy.node import Node
from typing import List, Optional
from rclpy.context import Context 
from rclpy.parameter import Parameter


class goal_provider(Node): 

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



        
def main(): 
    print('oi')

if __name__ == '__main__': 
    
    rclpy.init()

    node = goal_provider('goal_provider',
                         namespace='goal_manager', 
                         cli_args=['--debug'] )

    thread = threading.Thread(target = rclpy.spin, args = (node,), daemon = True)
    thread.start()

    rate = node.create_rate(10)

    try: 
        while rclpy.ok(): 

            main()
            
            rate.sleep()
    except KeyboardInterrupt:

        pass

    rclpy.shutdown()
    thread.join()