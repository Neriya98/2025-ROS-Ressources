from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

ARGUMENTS = [
    DeclareLaunchArgument('x_init',
            default_value='0.25',
            description='robot initial x'),
    DeclareLaunchArgument('y_init',
            default_value='0.0',
            description='robot initial y'),
    DeclareLaunchArgument('z_init',
            default_value='0.8',
            description='robot initial z'),
    DeclareLaunchArgument('yaw_init',
            default_value='-1.57',
            description='robot initial yaw')
]

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
 
    pkg_share = get_package_share_directory('sim_trc')

    gazebo_model_path = os.path.join(pkg_share, "models")

    gz_model = SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=gazebo_model_path
        )
    
    world_file = PathJoinSubstitution(
        [FindPackageShare("sim_trc"),
        "worlds",
        "trc2k25_arena.world"],
    )

    gazebo_sim = ExecuteProcess(
            cmd=['gazebo',  '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
            output='screen')
    

    ################### Add ROSMASTER X3 #######################
    x3_description_path = PathJoinSubstitution(
        [FindPackageShare("x3_description"),
        "launch",
        "x3_description.launch.py"])
    x3_description_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([x3_description_path]))

    teleop_node = Node(
        package= "x3_control",
        executable="yahboom_joy_X3.py",
        name="joy_X3",
        parameters=[{'use_sim_time': use_sim_time}]
    )

    joy_node = Node(
            package='joy',
            executable='joy_node',
    )

    spawn_x_val = LaunchConfiguration('x_init')
    spawn_y_val = LaunchConfiguration('y_init')
    spawn_z_val = LaunchConfiguration('z_init')
    spawn_yaw_val = LaunchConfiguration('yaw_init')

    # Launch the robot
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', "rosmaster_X3", 
                    '-topic', 'robot_description',
                        '-x', spawn_x_val,
                        '-y', spawn_y_val,
                        '-z', spawn_z_val,
                        '-Y', spawn_yaw_val],
                        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])
    

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_model)
    ld.add_action(gazebo_sim)
    ld.add_action(x3_description_launch)
    ld.add_action(teleop_node)
    ld.add_action(joy_node)
    ld.add_action(spawn_entity_cmd)
    return ld
