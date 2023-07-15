from multiprocessing import Condition
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():

    # Path to the package

    pkg_share = FindPackageShare(
        package='skuid_description').find('skuid_description')

    pkg_floxglove = FindPackageShare(
        package='foxglove_bridge').find('foxglove_bridge')

    # Path to rviz configuration

    default_rviz_config_path = os.path.join(
        pkg_share, 'rviz/rviz_skuid_settings.rviz')

    # Path to the urdf file

    default_urdf_model_path = os.path.join(pkg_share, 'urdf/skuid.urdf')

    # They are used to store values of launch arguments in the above variables and to pass them to required actions.

    gui = LaunchConfiguration('gui')
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='absolute path to skuid urdf'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to rviz config file'
    )

    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='True',
        description='Flag to enable joint_state_publisher_gui'
    )

    declare_use_robot_state_pub = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Wheather to start the robot state publisher'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Wheather to start rviz or not'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use gazebo sim clock if true'
    )

    # Specify the actions

    # Publish the joint state values fro the non-fixed joints in the URDF

    start_joint_state_publisher = Node(
        condition=UnlessCondition(gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Subscribe to the joint states of the robot and publish 3d pose of each link

    start_robot_state_publisher = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_model])}
        ],
        arguments=[default_urdf_model_path]
    )

    # start launch of rviz

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # set_fastrtps_environment = SetEnvironmentVariable(name='FASTRTPS_DEFAULT_PROFILES_FILE', value='/usr/workspaces/galactic/src/skuid_description/fastdds-conf/FASTDDS_PROFILES_FILE.xml')

    ignition_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/levels/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/model/skuid/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/skuid/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/world/levels/model/skuid/link/scan/sensor/scan/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/world/levels/model/skuid/link/scan/sensor/scan/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/model/skuid/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/model/skuid/pose@geometry_msgs/msg/TransformStamped[ignition.msgs.Pose'],
        output='screen',
        remappings=[
            ('/world/levels/clock', 'clock'),
            ('/world/levels/model/skuid/link/scan/sensor/scan/scan', 'scan'),
            ('/model/skuid/cmd_vel', 'cmd_vel'),
            ('/model/skuid/odometry', 'odom'),
            ('/world/levels/model/skuid/link/scan/sensor/scan/scan/points', 'scan/points'),
            ('/model/skuid/pose', '/skuid/pose'),
            ('/model/skuid/tf', '/skuid/tf')
        ]
    )

    # Node that subcribes to the skuid/pose to send wheels transforms but also the odom->base_link
    skuid_description_container = ComposableNodeContainer(
        name='skuid_description_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='skuid_description',
                plugin='skuid::ign_localization_node',
                name='ign_localization'
            ),
        ],
        output='screen',
    )

    # static transform from lidar link to the lidar sensor (identity)
    static_tf_lidar_to_gpu_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name="static_tf_lidar_to_gpu_lidar",
        arguments=["0", "0", "0", "0", "0", "0",
                   # transfor between URDF - robotname/linkname/sensorname
                   "scan", "skuid/scan/scan"]
    )

    floxglove = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            [os.path.join(pkg_floxglove, 'launch', 'foxglove_bridge_launch.xml')]),
    )

    # Create the launch description and populate

    ld = LaunchDescription()

    # Declare launch options
    # ld.add_action(set_fastrtps_environment)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_use_robot_state_pub)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # add any actions
    ld.add_action(start_joint_state_publisher)
    ld.add_action(start_robot_state_publisher)
    ld.add_action(static_tf_lidar_to_gpu_lidar)
    ld.add_action(skuid_description_container)
    ld.add_action(ignition_bridge)
    ld.add_action(floxglove)
    ld.add_action(start_rviz_cmd)

    return ld
