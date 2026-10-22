"""Everything that runs on the VEHICLE: drivers -> estimator -> controller -> mixer.

    ros2 launch robosub_sim vehicle.launch.py

pool.launch.py includes this file and adds Gazebo plus the sensor stand-ins,
so the sim exercises exactly the nodes the Jetson runs. Nothing in here may
depend on Gazebo.

DRIVERS. Each one replaces a sim stand-in and must speak the same topic:

    IMU driver        -> /imu/data_raw   sensor_msgs/Imu   body frame x fwd,
                                          y port, z up; rad/s; m/s^2 with
                                          gravity INCLUDED (+9.81 on z at rest)
    pressure driver   -> /depth          std_msgs/Float64  metres, +down
    Pathfinder DVL    -> /dvl/velocity   geometry_msgs/TwistWithCovarianceStamped
                                          body-frame m/s; publish NOTHING
                                          without bottom lock
                      -> /dvl/altitude   std_msgs/Float64  metres above floor
    thruster driver   <- /sub/thrust/t1..t6  std_msgs/Float64  newtons; must
                                          zero all outputs if commands stop

Add them as Node(...) entries in `drivers` below once written. Until then
this launch runs headless: the estimator waits for sensors and the
controller stays idle.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    drivers = [
        # Node(package='robosub_drivers', executable='imu_driver', output='screen'),
        # Node(package='robosub_drivers', executable='pressure_driver', output='screen'),
        # Node(package='robosub_drivers', executable='pathfinder_dvl', output='screen'),
        # Node(package='robosub_drivers', executable='thruster_driver', output='screen'),
    ]

    # /sub/cmd_vel -> six thruster forces. Only this node knows the layout.
    mixer = Node(package='robosub_sim', executable='thruster_mixer', output='screen')

    # /imu/data_raw + /depth + /dvl/velocity -> /sub/state.
    # dvl_offset: DVL position relative to the IMU / centre of rotation, in
    # the body frame. Measure it on the real sub and set it here.
    estimator = Node(
        package='robosub_sim', executable='state_estimator', output='screen',
        parameters=[{'dvl_offset': [-0.15, 0.0, -0.12]}],
    )

    # Depth / heading / roll / position hold. Idle until a setpoint arrives.
    controller = Node(package='robosub_sim', executable='depth_heading_controller', output='screen')

    return LaunchDescription(drivers + [mixer, estimator, controller])
