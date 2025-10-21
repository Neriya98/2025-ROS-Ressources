from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    x3_urdf = PathJoinSubstitution(
        [FindPackageShare("x3_description"),
        "urdf/robots",
        "rosmaster_x3.urdf.xacro"]
    )

    robot_description = ParameterValue(Command(['xacro ', x3_urdf]),
                                       value_type=str)

    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}]),
        
        Node(
            package='x3_description',
            executable='velocity_scaler.py',
            name='velocity_scaler',
            parameters=[{'use_sim_time': use_sim_time , 'angular_scale': 10.0}])
    ])
