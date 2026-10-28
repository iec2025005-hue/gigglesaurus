#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    urdf_file = os.path.join(pkg_share, 'urdf', 'rover.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'arena.sdf')

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # Resource paths so Gazebo Sim finds all meshes, textures, and models
    resource_paths = [
        os.path.join(pkg_share, 'models'),
        os.path.join(pkg_share, '..'),
        '/home/diya/rover_urdf_ws/src/rover_description/models',
        '/home/diya/rover_urdf_ws/src/rover_description'
    ]
    if 'GZ_SIM_RESOURCE_PATH' in os.environ and os.environ['GZ_SIM_RESOURCE_PATH']:
        resource_paths.append(os.environ['GZ_SIM_RESOURCE_PATH'])
    combined_resource_path = ':'.join(resource_paths)

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=combined_resource_path
    )
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=combined_resource_path
    )

    # Launch Gazebo Sim with Arena World (contains rover model)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # Robot State Publisher for ROS 2 TF transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Bridge arguments for full 4WIS/4WID Swerve, Sensors, and Telescopic Chassis
    bridge_arguments = [
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        # 4 Independent Steering Angles
        '/rover/steer_fl/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/rover/steer_fr/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/rover/steer_bl/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/rover/steer_br/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        # 4 Independent Wheel Drive Velocities
        '/rover/wheel_fl/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        '/rover/wheel_fr/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        '/rover/wheel_bl/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        '/rover/wheel_br/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        # Drive Motor Shafts
        '/rover/motor_fl/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        '/rover/motor_fr/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        '/rover/motor_bl/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        '/rover/motor_br/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        # Camera Pan / Tilt
        '/rover/camera/pan/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/rover/camera/tilt/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
    ]
    # 12 Telescopic Chassis Stages
    for idx in range(1, 7):
        bridge_arguments.append(f'/rover/tele_l{idx}/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double')
        bridge_arguments.append(f'/rover/tele_r{idx}/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double')

    # ROS <-> Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=bridge_arguments,
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gz_resource_path,
        ign_resource_path,
        gz_sim,
        robot_state_publisher,
        bridge
    ])
