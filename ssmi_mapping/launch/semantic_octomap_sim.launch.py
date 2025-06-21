from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, TextSubstitution 
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ssmi_mapping')
    declare_models_path = DeclareLaunchArgument(
        'models_path', default_value=pkg_share+'/sdf',
        description='SDF model path'
    )
    declare_world = DeclareLaunchArgument(
        'world', default_value=PathJoinSubstitution([LaunchConfiguration('models_path'), 'world.sdf']),
        description='World file name'
    )

    set_ign_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[LaunchConfiguration('models_path'), ':', EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value='')]
    )  

    declare_agent_name = DeclareLaunchArgument(
        'agent_name', default_value='husky_1',
        description='Name of the robot to spawn'
    )
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world')]}.items()
    )  #

    bridge_topics = [
        # '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        '/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/model/husky_seg_cam/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose',
        '/model/husky_seg_cam/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        # '/world/semantic_segmentation_world/model/husky_seg_cam/joint_state[sensor_msgs/msg/JointState@ignition.msgs.Model',
        # '/model/husky_seg_cam/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        '/semantic/colored_map@sensor_msgs/msg/Image[ignition.msgs.Image',
        # '/semantic/labels_map@sensor_msgs/msg/Image[ignition.msgs.Image',
    ]
    # robot_name = ['', LaunchConfiguration('agent_name')]
    robot_name = ['']
    topic_remap = [
        # ('/rgbd_camera/camera_info', PathJoinSubstitution([*robot_name, 'camera_info'])),
        ('/rgbd_camera/image', PathJoinSubstitution([*robot_name, 'camera/color/image_raw'])),
        ('/rgbd_camera/depth_image', PathJoinSubstitution([*robot_name, 'camera/depth/image_raw'])),
        ('/model/husky_seg_cam/pose', PathJoinSubstitution([*robot_name, 'pose'])),
        ('/model/husky_seg_cam/cmd_vel', PathJoinSubstitution([*robot_name, 'cmd_vel'])),
        ('/semantic/colored_map', PathJoinSubstitution([*robot_name, 'camera/semantic/image_raw'])),
    ] 
    bridge_node = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge', output='screen',
        arguments=bridge_topics,
        remappings=topic_remap
    )  

    param_file = PathJoinSubstitution([pkg_share, 'config', 'ssmi_cfg.yaml'])
    mapping_node = Node(
        package='ssmi_mapping', executable='octomap_generator_ros', name='octomap_generator', output='screen',
        parameters=[
            param_file
        ],
    )
    tf_node = Node(
        package='ssmi_mapping', executable='dynamic_tf.py', name='dynamic_tf_broadcaster', output='screen',
        parameters=[{
            'agent_name': ''
        }],
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz', output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'config.rviz'])]
    )

    sensor_node = Node(
        package='ssmi_mapping', executable='semantic_sensor_node.py', name='semantic_sensor_node', output='screen',
    )

    # teleop_node = Node(
    #     package='teleop_twist_keyboard', executable='teleop_twist_keyboard', name='husky_keyboard', output='screen',
    #     remappings=[('cmd_vel', '/model/husky_seg_cam/cmd_vel')]
    # )

    static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_camera_optic',
            output='screen',
            arguments=[
                '0.0', '0.0', '0.5',
                '-1.5708', '0.0', '-1.5708',
                'base_link', 'camera_optic'
            ]
        )
    return LaunchDescription([
        declare_models_path,
        declare_world,
        declare_agent_name,
        set_ign_resource,
        gz_sim_launch,
        bridge_node,
        tf_node,
        rviz_node,
        sensor_node,
        static_tf,
        mapping_node,
    ])
