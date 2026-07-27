## launch/pipeline.launch.py
## Full baseline pipeline: gz sim world -> bridge -> perception ->
## prediction -> validity/replanning.
##
## Usage (Linux box):
##   ros2 launch spite_d pipeline.launch.py \
##       world:=depth_single_cross \
##       roadmap_graph:=/path/to/roadmap_graph.txt \
##       roadmap_geoms:=/path/to/roadmap_geoms.txt
##
## The roadmap files come from the offline tool:
##   build_roadmap --out DIR --grow 2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('spite_d')
    gz_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    world_arg = DeclareLaunchArgument(
        'world', default_value=TextSubstitution(text='depth_single_cross'),
        description='World name under gazebo/worlds (without .sdf)')
    graph_arg = DeclareLaunchArgument('roadmap_graph')
    geoms_arg = DeclareLaunchArgument('roadmap_geoms')
    start_arg = DeclareLaunchArgument('start_vid', default_value='0')
    goal_arg = DeclareLaunchArgument('goal_vid', default_value='1')
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true',
        description='Run gz server-only with headless rendering (no GUI); '
                    'set false to open the Gazebo window')

    world_path = [
        TextSubstitution(text=os.path.join(pkg_share, 'gazebo', 'worlds', '')),
        LaunchConfiguration('world'),
        TextSubstitution(text='.sdf'),
    ]

    # '-r' = run immediately; headless adds server-only + offscreen rendering
    # (required on a machine with no display, e.g. driving gz over ssh).
    gz_flags = PythonExpression([
        '"-r -s --headless-rendering " if "', LaunchConfiguration('headless'),
        '" == "true" else "-r "'])

    return LaunchDescription([
        world_arg, graph_arg, geoms_arg, start_arg, goal_arg, headless_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': [gz_flags, *world_path]}.items()),

        Node(package='ros_gz_bridge', executable='parameter_bridge',
             output='screen',
             arguments=[
                 '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                 '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                 '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                 '/world/default/dynamic_pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V',
             ]),

        # Camera pose parameters must match the world's rgbd_camera model.
        Node(package='spite_d', executable='perception_node', output='screen',
             parameters=[{
                 'camera.x': 0.2, 'camera.y': 5.0, 'camera.z': 1.0,
                 'camera.yaw': 0.0,
             }]),

        Node(package='spite_d', executable='prediction_node', output='screen',
             parameters=[{'horizon': 3.0, 'dt': 0.25,
                          'std_growth_rate': 0.1}]),

        Node(package='spite_d', executable='validity_node', output='screen',
             parameters=[{
                 'roadmap_graph': LaunchConfiguration('roadmap_graph'),
                 'roadmap_geoms': LaunchConfiguration('roadmap_geoms'),
                 # Launch substitutions resolve to strings; the node declares
                 # these as ints, so the type must be stated explicitly or
                 # the node throws InvalidParameterTypeException at startup.
                 'start_vid': ParameterValue(LaunchConfiguration('start_vid'),
                                             value_type=int),
                 'goal_vid': ParameterValue(LaunchConfiguration('goal_vid'),
                                            value_type=int),
                 'sigma_gain': 1.0,
             }]),
    ])
