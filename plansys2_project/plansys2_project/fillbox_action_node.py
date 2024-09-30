#!/usr/bin/python3

# Copyright 2023 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.exceptions import ParameterNotDeclaredException
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from std_srvs.srv import Trigger 
from lifecycle_msgs.srv import GetState
import threading
from plansys2_project.pickupbox_action_node import PickupboxAction
from plansys2_project.loadcarrier_action_node import LoadCarrierAction
from plansys2_project.move_action_node import MoveAction
from plansys2_project.deliver_action_node import DeliverAction
from plansys2_project.emptybox_action_node import EmptyboxAction
from plansys2_project.return_action_node import ReturnAction
import time


class FillboxAction(ActionExecutorClient):

    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_fillbox_action_node', 0.3)
        self.progress_ = 0.0
        # Declare parameter to store robot name
        self.declare_parameter('robot_name', robot_name)
        # Retrieve the parameter
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        '''self.declare_parameter('robot_name', '')
        if self.has_parameter('robot_name'):
            self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        else:
            self.get_logger().error("Parameter 'robot_name' is missing!")'''
        # Here you can initialize additional attributes if needed
        self.current_arguments = []  # Initialize current_arguments_ if you need to use it

    def set_arguments(self, arguments):
        """Set the current arguments for the action."""
        self.current_arguments = arguments

    def do_work(self):
        """Perform the action work."""
        print(f'Running FillboxAction for robot: {self.robot_name}')
        
        # Execute the action
        self.send_feedback(1.0, 'Filling box completed')  # Send final feedback since there's only one update
        self.finish(True, 1.0, 'Fillbox completed')  # Finish the action
        self.progress_ = 0.0  # Reset progress for future actions
        print('running3')
        self.get_logger().info(f'Filling box completed')
        print('running10')




def main(args=None):
    rclpy.init(args=args)

    try:
        # List of robots
        #robots = ['r1', 'r2', 'r3']

        # Iterate over each robot
        #for robot_name in robots:
        # Create a temporary node to retrieve parameters
        temp_node = Node('temp_node')
        
        # Declare the parameter robot_name
        temp_node.declare_parameter('robot_name', '')
        
        # Retrieve the parameter value
        robot_name = temp_node.get_parameter('robot_name').get_parameter_value().string_value
        
        # Log the retrieved robot name
        temp_node.get_logger().info(f'Retrieved robot name: {robot_name}')
        temp_node.destroy_node()
        action_nodes = []
        if robot_name:
            # Initialize each action node with the robot's name
            fillbox_node = FillboxAction(robot_name)
            #if FillboxAction.robot_name:
            print(f"Starting actions...")
            fillbox_node.set_parameters([Parameter(name='action_name', value='fillbox')])
            fillbox_node.set_arguments([robot_name, 'fillbox'])
            
            print('running4')
            action_nodes.append(fillbox_node)
            print('running5')

            pickupbox_node = PickupboxAction(robot_name)
            pickupbox_node.set_parameters([Parameter(name='action_name', value='pickupbox')])
            pickupbox_node.set_arguments([robot_name, 'pickupbox'])      
            action_nodes.append(pickupbox_node)

            loadcarrier_node = LoadCarrierAction(robot_name)
            loadcarrier_node.set_parameters([Parameter(name='action_name', value='loadcarrier')])
            loadcarrier_node.set_arguments([robot_name, 'loadcarrier'])
            action_nodes.append(loadcarrier_node)

            move_node = MoveAction(robot_name)
            move_node.set_parameters([Parameter(name='action_name', value='move')])
            move_node.set_arguments([robot_name, 'move'])
            action_nodes.append(move_node)

            deliver_node = DeliverAction(robot_name)
            deliver_node.set_parameters([Parameter(name='action_name', value='deliver')])
            deliver_node.set_arguments([robot_name, 'deliver'])
            action_nodes.append(deliver_node)

            empty_node = EmptyboxAction(robot_name)
            empty_node.set_parameters([Parameter(name='action_name', value='emptybox')])
            empty_node.set_arguments([robot_name, 'emptybox'])
            action_nodes.append(empty_node)

            return_node = ReturnAction(robot_name)
            return_node.set_parameters([Parameter(name='action_name', value='return')])
            return_node.set_arguments([robot_name, 'return'])
            action_nodes.append(return_node)

            # Configure and run each action node for the robot
            for node in action_nodes:
                node.trigger_configure()  # Configure the node
                rclpy.spin_once(node)  # Spin the node once to process the configuration

                # Execute the action until it completes
                #while node.progress_ < 1.0:
                node.do_work()  # Execute the work of the node
                rclpy.spin_once(node)
            print(f"Completed actions\n")

    except rclpy.exceptions.ROSInterruptException as e:
        print(f"ROS error: {e}")
    except Exception as e:
        print(f"Exception occurred: {e}")
    finally:
        # Shutdown all action nodes gracefully
        for node in action_nodes:
           node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()