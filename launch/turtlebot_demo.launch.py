## launch/turtlebot_demo.launch.py
##
## Runs the certification pipeline against a REAL robot's depth camera.
## The robot does not move: it sits still and watches. A person walking
## through the workspace should show up as a tracked box, turn roadmap
## edges red, and make the planned path reroute -- all visible in RViz.
##
## This is the whole package except execution. There is deliberately no
## cmd_vel here: nothing yet connects the planned path to the wheels, and
## the roadmap is a free-flying box robot rather than a differential
## drive, so its paths are not directly drivable anyway.
##
## Geometry assumed by the defaults:
##
##        y
##        ^      . . . . . . . .      roadmap: 4m x 4m slab just above
##        |      . . . . . . . .      the floor, x,y in [-2, 2]
##   cam  @----> . . . . . . . .
##  (-2,0)   x   . . . . . . . .      camera at one edge looking down +x
##
## Usage (a matching roadmap ships with the package):
##   ros2 launch spite_d turtlebot_demo.launch.py \
##       depth_topic:=/camera/depth/image_raw
##
## Spans are on by default. For the baseline they are measured against,
## add use_spans:=false and repeat the same walk.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('spite_d')

    # A roadmap for this 4m x 4m setup ships with the package, so there is
    # nothing to generate and OMPL is not needed to run the demo. Override
    # these only if you built your own.
    default_roadmap = os.path.join(pkg_share, 'demo_roadmap')

    args = [
        DeclareLaunchArgument(
            'roadmap_graph',
            default_value=os.path.join(default_roadmap, 'roadmap_graph.txt')),
        DeclareLaunchArgument(
            'roadmap_geoms',
            default_value=os.path.join(default_roadmap, 'roadmap_geoms.txt')),

        # Depth topics differ by camera and driver. On a Waffle with a
        # RealSense these are often /camera/depth/image_rect_raw and
        # /camera/depth/camera_info -- check `ros2 topic list` first.
        DeclareLaunchArgument('depth_topic',
                              default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/camera/depth/camera_info'),

        # Where the camera sits in the roadmap's frame. Defaults place it
        # at the edge of the slab looking across it. Measure and adjust;
        # if obstacles appear in the wrong place, this is the first thing
        # to check.
        DeclareLaunchArgument('camera_x', default_value='-2.0'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.17'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),

        # 1000.0 for 16UC1 millimetre depth (RealSense). The node also
        # accepts 32FC1 metres and converts automatically.
        DeclareLaunchArgument('depth_scale', default_value='1000.0'),

        DeclareLaunchArgument('start_vid', default_value='0'),
        DeclareLaunchArgument('goal_vid', default_value='1'),
        # The contribution switch. Run it once each way, same room, same
        # walk: `use_spans:=false` is the every-frame baseline.
        DeclareLaunchArgument('use_spans', default_value='true'),
        DeclareLaunchArgument('slices', default_value='1'),
        DeclareLaunchArgument('report_period_s', default_value='5.0'),

        DeclareLaunchArgument('marker_frame', default_value='map'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    perception = Node(
        package='spite_d', executable='perception_node', output='screen',
        parameters=[{
            'camera.x': ParameterValue(LaunchConfiguration('camera_x'),
                                       value_type=float),
            'camera.y': ParameterValue(LaunchConfiguration('camera_y'),
                                       value_type=float),
            'camera.z': ParameterValue(LaunchConfiguration('camera_z'),
                                       value_type=float),
            'camera.yaw': ParameterValue(LaunchConfiguration('camera_yaw'),
                                         value_type=float),
            'depth_scale': ParameterValue(LaunchConfiguration('depth_scale'),
                                          value_type=float),
        }],
        remappings=[
            ('/camera/depth_image', LaunchConfiguration('depth_topic')),
            ('/camera/camera_info', LaunchConfiguration('camera_info_topic')),
        ])

    # validity_node predicts internally now -- a span IS a frozen
    # prediction, so it cannot take ready-made ones. This node stays only
    # so /spite_d_prediction/predictions exists to look at and to record.
    prediction = Node(
        package='spite_d', executable='prediction_node', output='screen',
        parameters=[{'horizon': 3.0, 'dt': 0.25, 'std_growth_rate': 0.1}])

    validity = Node(
        package='spite_d', executable='validity_node', output='screen',
        parameters=[{
            'roadmap_graph': LaunchConfiguration('roadmap_graph'),
            'roadmap_geoms': LaunchConfiguration('roadmap_geoms'),
            'start_vid': ParameterValue(LaunchConfiguration('start_vid'),
                                        value_type=int),
            'goal_vid': ParameterValue(LaunchConfiguration('goal_vid'),
                                       value_type=int),
            'sigma_gain': 1.0,
            'use_spans': ParameterValue(LaunchConfiguration('use_spans'),
                                        value_type=bool),
            'slices': ParameterValue(LaunchConfiguration('slices'),
                                     value_type=int),
            'report_period_s': ParameterValue(
                LaunchConfiguration('report_period_s'), value_type=float),
            # Same lookahead as the prediction node above, so the baseline
            # and span runs are comparable.
            'horizon': 3.0, 'dt': 0.25, 'std_growth_rate': 0.1,
            'marker_frame': LaunchConfiguration('marker_frame'),
            'max_drawn_edges': 4000,
        }])

    # The markers are published in `marker_frame`; publish a static
    # identity transform so RViz has somewhere to put them even with no
    # localisation running.
    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='log')

    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'demo.rviz')])

    return LaunchDescription(args + [perception, prediction, validity,
                                     static_tf, rviz])
