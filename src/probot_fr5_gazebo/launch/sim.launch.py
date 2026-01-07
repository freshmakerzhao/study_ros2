from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description():
    # --- Launch args ---
    use_sim = LaunchConfiguration("use_sim")
    xacro_file = LaunchConfiguration("xacro_file")
    robot_name = LaunchConfiguration("robot_name")

    declare_use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="true",
        description="Set true to enable gazebo_ros2_control in robot_description",
    )

    declare_robot_name = DeclareLaunchArgument(
        "robot_name",
        default_value="fairino5_v6_robot",
        description="Entity name in Gazebo",
    )

    declare_xacro_file = DeclareLaunchArgument(
        "xacro_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("probot_fr5_moveit_config"),
            "config",
            "fairino5_v6_robot.urdf.xacro",
        ]),
        description="Path to top-level xacro",
    )

    # --- robot_description from xacro ---
    # IMPORTANT: your top-level xacro must accept use_sim:=true and inject
    # <gazebo><plugin filename="libgazebo_ros2_control.so">...</plugin></gazebo>
    robot_description = {
        "robot_description": Command([
            "xacro ",
            xacro_file,
            " use_sim:=",
            use_sim,
        ])
    }

    # --- Gazebo (classic) ---
    gazebo = ExecuteProcess(
        cmd=[
            "gazebo", "--verbose",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
    )


    # --- Robot State Publisher ---
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # --- Spawn entity into Gazebo ---
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", robot_name,
            "-topic", "robot_description",
        ],
    )

    # --- Spawn controllers (controller_manager is provided by gazebo_ros2_control plugin) ---
    # joint_state_broadcaster
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
    )

    # arm_controller
    spawn_arm_ctrl = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
        ],
    )

    return LaunchDescription([
        declare_use_sim,
        declare_robot_name,
        declare_xacro_file,

        gazebo,
        rsp,
        spawn_entity,
        spawn_jsb,
        spawn_arm_ctrl,
    ])
