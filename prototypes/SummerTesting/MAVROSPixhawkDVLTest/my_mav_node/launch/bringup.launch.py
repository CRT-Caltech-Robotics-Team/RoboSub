# Bring up the full stack: MAVROS (TELEM2), the DVL external-vision node
# (which also broadcasts the ENU->NED / FLU->FRD static transforms the MAVROS
# odometry plugin needs), and the offboard control node.
#
#   ros2 launch my_mav_node bringup.launch.py
#
# Useful arguments:
#   fcu_url:=/dev/ttyTHS1:115200   serial port / baud for the Pixhawk
#   dvl_port:=/dev/ttyUSB0         DVL serial port
#   auto_arm:=false                stream + OFFBOARD but do not arm
#   auto_offboard:=false           only stream setpoints
#
# Omit MAVROS from this launch (start_mavros:=false) if you already run it
# separately.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fcu_url = LaunchConfiguration("fcu_url")
    dvl_port = LaunchConfiguration("dvl_port")
    auto_arm = LaunchConfiguration("auto_arm")
    auto_offboard = LaunchConfiguration("auto_offboard")
    start_mavros = LaunchConfiguration("start_mavros")

    return LaunchDescription([
        DeclareLaunchArgument("fcu_url", default_value="/dev/ttyTHS1:115200"),
        DeclareLaunchArgument("dvl_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("auto_arm", default_value="true"),
        DeclareLaunchArgument("auto_offboard", default_value="true"),
        DeclareLaunchArgument("start_mavros", default_value="true"),

        GroupAction(
            condition=IfCondition(start_mavros),
            actions=[
                Node(
                    package="mavros",
                    executable="mavros_node",
                    output="screen",
                    parameters=[{"fcu_url": fcu_url}],
                ),
            ],
        ),

        Node(
            package="my_mav_node",
            executable="myprogram",
            name="dvl_node",
            output="screen",
            parameters=[{"serial_port": dvl_port}],
        ),

        Node(
            package="my_mav_node",
            executable="offboard_node",
            name="offboard_node",
            output="screen",
            parameters=[{
                "auto_arm": auto_arm,
                "auto_offboard": auto_offboard,
            }],
        ),
    ])
