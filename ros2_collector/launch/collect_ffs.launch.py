from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    from launch_ros.substitutions import FindPackageShare

    package_share = FindPackageShare("franka_7link_dynamics_id_ros2")
    controller_config = PathJoinSubstitution(
        [package_share, "config", "ros_controllers.yaml"]
    )
    default_output = PathJoinSubstitution(
        [
            EnvironmentVariable("HOME"),
            "franka_7link_dynamics_id_runs",
            "acquisitions",
        ]
    )

    arguments = [
        DeclareLaunchArgument(
            "summary_file",
            description="FFS summary JSON produced by the Python project.",
        ),
        DeclareLaunchArgument("output_base_dir", default_value=default_output),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("controller_manager", default_value="controller_manager"),
        DeclareLaunchArgument("controller_manager_timeout", default_value="15"),
        DeclareLaunchArgument("trajectory_split", default_value="train"),
        DeclareLaunchArgument("trajectory_index", default_value="1"),
        DeclareLaunchArgument("settle_time", default_value="3.0"),
        DeclareLaunchArgument("return_to_start_timeout", default_value="20.0"),
        DeclareLaunchArgument("collection_rate_hz", default_value="1000.0"),
        DeclareLaunchArgument("joint_states_topic", default_value="joint_states"),
        DeclareLaunchArgument("start_position_tolerance", default_value="0.01"),
        DeclareLaunchArgument("start_velocity_tolerance", default_value="0.03"),
    ]
    namespace = LaunchConfiguration("namespace")
    controller_manager = LaunchConfiguration("controller_manager")
    timeout = LaunchConfiguration("controller_manager_timeout")

    ffs_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "ffs_trajectory_controller",
            "--controller-manager",
            controller_manager,
            "--controller-type",
            "franka_7link_dynamics_id_ros2/FFSTrajectoryController",
            "--param-file",
            controller_config,
            "--controller-manager-timeout",
            timeout,
            "--inactive",
        ],
        output="screen",
    )
    collector_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "ffs_data_collection_controller",
            "--controller-manager",
            controller_manager,
            "--controller-type",
            "franka_7link_dynamics_id_ros2/FFSDataCollectionController",
            "--param-file",
            controller_config,
            "--controller-manager-timeout",
            timeout,
            "--inactive",
        ],
        output="screen",
    )
    return_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "velocity_based_joint_position_controller",
            "--controller-manager",
            controller_manager,
            "--controller-type",
            "franka_7link_dynamics_id_ros2/VelocityBasedJointPositionController",
            "--param-file",
            controller_config,
            "--controller-manager-timeout",
            timeout,
        ],
        output="screen",
    )
    orchestrator = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="franka_7link_dynamics_id_ros2",
                executable="ffs_orchestrator.py",
                name="ffs_orchestrator",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "summary_file": LaunchConfiguration("summary_file"),
                        "output_base_dir": LaunchConfiguration("output_base_dir"),
                        "controller_manager": controller_manager,
                        "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                        **{
                            key: ParameterValue(LaunchConfiguration(key), value_type=float)
                            for key in (
                                "settle_time",
                                "return_to_start_timeout",
                                "collection_rate_hz",
                                "start_position_tolerance",
                                "start_velocity_tolerance",
                            )
                        },
                        "trajectory_split": LaunchConfiguration("trajectory_split"),
                        "trajectory_index": ParameterValue(
                            LaunchConfiguration("trajectory_index"), value_type=int
                        ),
                    }
                ],
            )
        ],
    )
    return LaunchDescription(
        arguments + [ffs_spawner, collector_spawner, return_spawner, orchestrator]
    )
