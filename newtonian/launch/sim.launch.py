
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def _node(context: LaunchContext) -> list:
    """
    """
    config = LaunchConfiguration("config").perform(context=context)
    if not os.path.isabs(config):
        config = os.path.join(get_package_share_directory("newtonian"), "config", f"{config}.yaml")
    return [
        Node(
            package="newtonian",
            executable="sim",
            name="newtonian",
            output="screen",
            parameters=[config],
        ),
    ]


def generate_launch_description():
    """
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="config",
                default_value="racecar",
                description="config name or parameter file path",
            ),
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="true",
                description="use /clock time in the nodes of this launch (the simulator always steps on wall time)",
            ),
            SetParameter(
                name="use_sim_time",
                value=LaunchConfiguration("use_sim_time"),
            ),
            OpaqueFunction(
                function=_node
            ),
        ]
    )
