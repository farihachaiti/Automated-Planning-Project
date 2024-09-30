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
from rclpy.parameter import Parameter
from rclpy.exceptions import ParameterNotDeclaredException
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from std_srvs.srv import Trigger 


class ReturnAction(ActionExecutorClient):

    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_return_action_node', 0.3)
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

        self.current_arguments = []  # Initialize current_arguments_ if you need to use it

    def set_arguments(self, arguments):
        """Set the current arguments for the action."""
        self.current_arguments = arguments

    def do_work(self):
        """Perform the action work."""
        print(f'Running ReturnAction for robot: {self.robot_name}')
        
        # Execute the action
        #self.send_feedback(1.0, 'Returning robot completed')  # Send final feedback since there's only one update
        self.finish(True, 1.0, 'ReturnAction completed')
        self.progress_ = 0.0  # Reset progress for future actions
        print('running3')
        self.get_logger().info(f'Returning robot completed')
        print('running10')








