#!/usr/bin/env python3
"""
MoveIt Demo Launch File

Control the robot with Interactive Markers in RViz.

Usage:
    ros2 launch launch/moveit_demo.launch.py robot_config:=configs/robot/so100_config.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import sys
from pathlib import Path
import yaml

# Add src to path for RobotBridge
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


def load_robot_config(config_path):
    """Load robot configuration file"""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def flatten_dict(d, parent_key='', sep='.'):
    """Flatten nested dictionary for ROS2 parameters"""
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)


def generate_launch_description_from_config(context, *args, **kwargs):
    """Generate launch description from configuration file"""

    # Launch arguments
    robot_config_path = LaunchConfiguration('robot_config').perform(context)
    urdf_path = LaunchConfiguration('urdf').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    use_real_robot = LaunchConfiguration('use_real_robot').perform(context) == 'true'

    # Load robot configuration
    robot_config = load_robot_config(robot_config_path)
    robot_name = robot_config.get('robot_type', 'robot')

    # Load URDF
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(
            f"URDF file not found: {urdf_path}\n"
            f"Generate it first:\n"
            f"  python scripts/generate_urdf.py --config {robot_config_path} --output {urdf_path}"
        )

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # Fix mesh paths: replace relative 'assets/' with absolute path
    urdf_dir = str(Path(urdf_path).parent.absolute())
    robot_description = robot_description.replace('filename="assets/', f'filename="file://{urdf_dir}/assets/')

    # SRDF path
    srdf_path = str(Path('config/moveit') / f'{robot_name}.srdf')
    if not os.path.exists(srdf_path):
        # Generate basic SRDF
        num_joints = len(robot_config.get('motors', {}))
        srdf_content = generate_basic_srdf(robot_name, num_joints)
        os.makedirs('config/moveit', exist_ok=True)
        with open(srdf_path, 'w') as f:
            f.write(srdf_content)
        print(f"Generated SRDF: {srdf_path}")

    with open(srdf_path, 'r') as f:
        robot_description_semantic = f.read()

    # Kinematics configuration
    kinematics_yaml_path = 'config/moveit/kinematics.yaml'
    if not os.path.exists(kinematics_yaml_path):
        kinematics_yaml = generate_kinematics_yaml(robot_name)
        with open(kinematics_yaml_path, 'w') as f:
            f.write(kinematics_yaml)
        print(f"Generated kinematics config: {kinematics_yaml_path}")

    with open(kinematics_yaml_path, 'r') as f:
        kinematics_yaml = yaml.safe_load(f)

    # Controller configuration - DIRECT Python dict (no YAML to avoid parsing issues)
    # MoveIt simple_controller_manager expects these exact parameter names
    controllers_params = {
        'moveit_simple_controller_manager.controller_names': ['robot_controller'],
        'moveit_simple_controller_manager.robot_controller.type': 'FollowJointTrajectory',
        'moveit_simple_controller_manager.robot_controller.action_ns': 'follow_joint_trajectory',
        'moveit_simple_controller_manager.robot_controller.default': True,
        'moveit_simple_controller_manager.robot_controller.joints': [
            'shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'
        ],
    }

    print("=" * 80)
    print("CONTROLLER PARAMETERS BEING SET:")
    for key, value in controllers_params.items():
        print(f"  {key}: {value}")
    print("=" * 80)

    # Joint limits
    joint_limits_path = 'config/moveit/joint_limits.yaml'
    if not os.path.exists(joint_limits_path):
        joint_limits = generate_joint_limits(robot_config)
        with open(joint_limits_path, 'w') as f:
            f.write(joint_limits)
        print(f"Generated joint limits: {joint_limits_path}")

    with open(joint_limits_path, 'r') as f:
        joint_limits_yaml = yaml.safe_load(f)

    # Controllers configuration - inline for debugging
    # controllers_yaml_path = 'config/moveit/controllers.yaml'
    # with open(controllers_yaml_path, 'r') as f:
    #     controllers_yaml = yaml.safe_load(f)

    # MoveIt parameters - Jazzy format
    moveit_params = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': kinematics_yaml,
        'robot_description_planning': joint_limits_yaml,
        'use_sim_time': use_sim_time,
    }

    # Planning pipelines configuration - Jazzy format
    # Key change in Jazzy: planning_plugins is an ARRAY, not a string
    moveit_params['planning_pipelines'] = {
        'pipeline_names': ['ompl'],
    }
    moveit_params['default_planning_pipeline'] = 'ompl'

    # OMPL pipeline configuration - Jazzy format
    moveit_params['ompl'] = {
        'planning_plugins': ['ompl_interface/OMPLPlanner'],  # Array in Jazzy!
        'request_adapters': [
            'default_planning_request_adapters/ResolveConstraintFrames',
            'default_planning_request_adapters/ValidateWorkspaceBounds',
            'default_planning_request_adapters/CheckStartStateBounds',
            'default_planning_request_adapters/CheckStartStateCollision',
        ],
        'response_adapters': [
            'default_planning_response_adapters/AddTimeOptimalParameterization',
            'default_planning_response_adapters/ValidateSolution',
            'default_planning_response_adapters/DisplayMotionPath',
        ],
        'start_state_max_bounds_error': 10.0,  # Relaxed for calibration mismatch
    }

    # Add move group capabilities (removed duplicate/invalid ExecuteTrajectoryAction)
    moveit_params['capabilities'] = (
        'move_group/MoveGroupCartesianPathService '
        'move_group/MoveGroupExecuteTrajectoryAction '
        'move_group/MoveGroupKinematicsService '
        'move_group/MoveGroupMoveAction '
        'move_group/MoveGroupPlanService '
        'move_group/MoveGroupQueryPlannersService '
        'move_group/MoveGroupStateValidationService'
    )

    # Enable trajectory execution with custom action server
    moveit_params['allow_trajectory_execution'] = True
    moveit_params['moveit_manage_controllers'] = False  # We manage controllers externally

    # Specify which controller manager plugin to use
    moveit_params['moveit_controller_manager'] = 'moveit_simple_controller_manager/MoveItSimpleControllerManager'

    moveit_params['trajectory_execution'] = {
        'execution_duration_monitoring': False,  # Disable monitoring for custom controller
        'allowed_execution_duration_scaling': 2.0,
        'allowed_goal_duration_margin': 1.0,
        'allowed_start_tolerance': 0.5,  # Relaxed for real robot tolerance
    }

    # Merge controller configuration from YAML
    moveit_params.update(controllers_params)

    # Nodes
    nodes = [
        # Static transform: world -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
            output='screen',
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
    ]

    # Add Joint State Publisher GUI only if NOT using real robot
    if not use_real_robot:
        nodes.append(
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                }],
            )
        )
        print("Using Joint State Publisher GUI for manual control")
    else:
        print("\n" + "="*80)
        print("⚠️  REAL ROBOT MODE")
        print("="*80)
        print("Please run the robot bridge in a separate terminal:")
        print(f"  python scripts/run_ros2_bridge.py --config {robot_config_path}")
        print("="*80 + "\n")

    # Continue with other nodes
    nodes.extend([

        # MoveIt Move Group
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[moveit_params],
            remappings=[('/joint_states', 'joint_states')],
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
            parameters=[{
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_yaml,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])

    return nodes


def generate_basic_srdf(robot_name, num_joints):
    """Generate basic SRDF - using SO-101 actual joint names"""
    # SO-101 actual joint names
    joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
    joints_xml = '\n'.join(f'    <joint name="{name}"/>' for name in joint_names)
    group_state = '\n'.join(f'    <joint name="{name}" value="0"/>' for name in joint_names)

    return f"""<?xml version="1.0"?>
<robot name="{robot_name}">
  <!-- Planning group for arm (excluding gripper) -->
  <group name="arm">
{joints_xml}
  </group>

  <!-- End effector -->
  <end_effector name="gripper" parent_link="gripper_link" group="arm"/>

  <!-- Disable collision checking for adjacent links -->
  <disable_collisions link1="base_link" link2="shoulder_link" reason="Adjacent"/>
  <disable_collisions link1="shoulder_link" link2="upper_arm_link" reason="Adjacent"/>
  <disable_collisions link1="upper_arm_link" link2="lower_arm_link" reason="Adjacent"/>
  <disable_collisions link1="lower_arm_link" link2="wrist_link" reason="Adjacent"/>
  <disable_collisions link1="wrist_link" link2="gripper_link" reason="Adjacent"/>

  <!-- Disable collision checking for gripper links -->
  <disable_collisions link1="gripper_link" link2="gripper_frame_link" reason="Adjacent"/>
  <disable_collisions link1="gripper_link" link2="moving_jaw_so101_v1_link" reason="Default"/>
  <disable_collisions link1="gripper_link" link2="fixed_jaw_so101_v1_link" reason="Default"/>
  <disable_collisions link1="gripper_frame_link" link2="moving_jaw_so101_v1_link" reason="Adjacent"/>
  <disable_collisions link1="gripper_frame_link" link2="fixed_jaw_so101_v1_link" reason="Adjacent"/>
  <disable_collisions link1="moving_jaw_so101_v1_link" link2="fixed_jaw_so101_v1_link" reason="Adjacent"/>

  <!-- Virtual joint for fixed base -->
  <virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="base_link"/>

  <!-- Group states (predefined poses) -->
  <group_state name="home" group="arm">
{group_state}
  </group_state>
</robot>
"""


def generate_kinematics_yaml(robot_name):
    """Generate kinematics solver configuration"""
    return f"""arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 3
"""


def generate_joint_limits(robot_config):
    """Generate joint limits configuration - using SO-101 actual joint names"""
    # SO-101 actual joint names
    joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
    limits = ["joint_limits:"]

    for joint_name in joint_names:
        limits.append(f"  {joint_name}:")
        limits.append(f"    has_velocity_limits: true")
        limits.append(f"    max_velocity: 2.0")
        limits.append(f"    has_acceleration_limits: true")
        limits.append(f"    max_acceleration: 3.0")

    return '\n'.join(limits)


def generate_launch_description():
    """Generate launch description"""
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'robot_config',
            default_value='configs/robot/so100_config.yaml',
            description='Robot configuration file path'
        ),
        DeclareLaunchArgument(
            'urdf',
            default_value='robot.urdf',
            description='URDF file path'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='config/rviz/moveit.rviz',
            description='RViz configuration file path'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'use_real_robot',
            default_value='false',
            description='Use real robot (true: real robot, false: GUI)'
        ),

        # Opaque function to load config and generate nodes
        OpaqueFunction(function=generate_launch_description_from_config),
    ])
