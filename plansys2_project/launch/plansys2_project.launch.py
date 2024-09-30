import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_project')

    # List of robots
    robots = ['r1', 'r2', 'r3']
    
    ld = LaunchDescription()

    # Central PlanSys2 bringup (if only one instance is needed)
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('plansys2_bringup'), 'launch', 'plansys2_bringup_launch_distributed.py')
        ),
        launch_arguments={
            'model_file': example_dir + '/pddl/model.pddl',
            'problem_file': example_dir + '/pddl/problem.pddl',
        }.items()
    )
    ld.add_action(plansys2_cmd)

    # Define the actions for each robot namespace
    for robot_name in robots:
        # Group action to set the namespace for each robot
        robot_group = GroupAction([
            # Push namespace for the current robot
            PushRosNamespace(robot_name),

            # Action nodes for each robot
            Node(
                package='plansys2_project',
                executable='fillbox_action_node.py',
                name=f'{robot_name}_fillbox_action_node',
                output='screen',
                parameters=[{'robot_name': robot_name}]
            ),
            Node(
                package='plansys2_project',
                executable='pickupbox_action_node.py',
                name=f'{robot_name}_pickupbox_action_node',
                output='screen',
                parameters=[{'robot_name': robot_name}]
            ),
            Node(
                package='plansys2_project',
                executable='loadcarrier_action_node.py',
                name=f'{robot_name}_loadcarrier_action_node',
                output='screen',
                parameters=[{'robot_name': robot_name}]
            ),
            Node(
                package='plansys2_project',
                executable='move_action_node.py',
                name=f'{robot_name}_move_action_node',
                output='screen',
                parameters=[{'robot_name': robot_name}]
            ),
            Node(
                package='plansys2_project',
                executable='deliver_action_node.py',
                name=f'{robot_name}_deliver_action_node',
                output='screen',
                parameters=[{'robot_name': robot_name}]
            ),
            Node(
                package='plansys2_project',
                executable='emptybox_action_node.py',
                name=f'{robot_name}_emptybox_action_node',
                output='screen',
                parameters=[{'robot_name': robot_name}]
            ),
            Node(
                package='plansys2_project',
                executable='return_action_node.py',
                name=f'{robot_name}_return_action_node',
                output='screen',
                parameters=[{'robot_name': robot_name}]
            )
        ])
        ld.add_action(robot_group)

    return ld