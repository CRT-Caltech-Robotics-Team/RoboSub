#!/usr/bin/env python3
r"""Control allocation: turn a desired motion into individual thruster forces.

This is THE core idea in AUV control. Higher layers (teleop, a PID depth
hold, a path follower) all speak in "what should the sub do" -- surge
forward, strafe right, yaw left, rise. This node is the only place that
knows how many thrusters exist and where they are bolted on.

Swap the sub's frame, and only this file changes.

    Twist.linear.x  -> surge (forward)
    Twist.linear.y  -> sway  (port)
    Twist.linear.z  -> heave (up)
    Twist.angular.x -> roll
    Twist.angular.z -> yaw

FRAME: x forward, y port, z up.

LAYOUT (six thrusters, geometry from my_robot_description/robot.urdf):

                     bow (+x)
              thr_3  /        \  thr_4
                    /          \
        (+y port)  |   5    6   |  (-y starboard)
                    \          /
              thr_1  \        /  thr_2
                     stern (-x)

    1-4 are vectored horizontals at the hull corners, angled +-45 deg.
    5-6 are verticals amidships, port and starboard.

WHY THE MIXING SIGNS BELOW ARE WHAT THEY ARE

Each horizontal thruster pushes along a unit vector d = (cos t, sin t):

    thr_1  pos (-0.294, +0.279)  t = -135 deg  d = (-0.71, -0.71)
    thr_2  pos (-0.294, -0.279)  t = +135 deg  d = (-0.71, +0.71)
    thr_3  pos (+0.294, +0.279)  t =  -45 deg  d = (+0.71, -0.71)
    thr_4  pos (+0.294, -0.279)  t =  +45 deg  d = (+0.71, +0.71)

(Positions are the propeller duct axes from the CAD mesh, not the URDF
joint origins, which sit on the mount brackets 5-8 cm inboard.)

Yaw moment per unit thrust is (r_x*d_y - r_y*d_x), which works out to
+0.41, -0.41, -0.41, +0.41 for thrusters 1..4. Combining those
with the force directions gives three mutually orthogonal patterns --
each of surge, sway and yaw produces zero net force in the other two
axes. That decoupling is the whole point of a vectored layout, and it is
why the mixing below is just a table of signs rather than a matrix solve.

NO PITCH AUTHORITY. Thrusters 5 and 6 both sit at x = +0.012, so their
pitch moment arm is about a centimetre -- effectively nothing. What they
do have is a 0.59 m lateral separation, so they give roll instead. The
sub relies on passive pitch stability (centre of buoyancy above centre of
gravity) exactly as the real vehicle does. Twist.angular.y is ignored.

ROLL GAIN. Roll effort 1.0 puts MAX_THRUST on thrusters 5/6 in opposite
directions (~35 N.m). That is deliberately the whole thruster: the sub's
righting moment peaks at ~20 N.m when it is on its side, so anything
less than full thrust cannot complete a 360 deg roll. Roll and heave share
those two thrusters, so a hard roll leaves nothing for depth.

UNITS: commands are thrust in newtons. The Gazebo Thruster plugin's
<velocity_control> affects how it drives the propeller joint internally,
not what the cmd_thrust topic means.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Newtons of thrust per unit of commanded input.
GAIN_SURGE = 30.0
GAIN_SWAY  = 30.0
GAIN_YAW   = 20.0
GAIN_HEAVE = 30.0
GAIN_ROLL  = 60.0   # full thrust: a barrel roll needs every newton
MAX_THRUST = 60.0

THRUSTERS = ('t1', 't2', 't3', 't4', 't5', 't6')


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class ThrusterMixer(Node):
    def __init__(self):
        super().__init__('thruster_mixer')

        self.pubs = {
            name: self.create_publisher(Float64, f'/sub/thrust/{name}', 10)
            for name in THRUSTERS
        }
        self.create_subscription(Twist, '/sub/cmd_vel', self.on_cmd, 10)

        # Safety: if commands stop arriving, coast to zero rather than
        # running away. Real vehicles must always have this.
        self.last_cmd = self.get_clock().now()
        self.create_timer(0.1, self.watchdog)

        self.get_logger().info(
            'thruster_mixer up (6 thrusters) — listening on /sub/cmd_vel')

    def on_cmd(self, msg: Twist):
        self.last_cmd = self.get_clock().now()

        surge = msg.linear.x  * GAIN_SURGE
        sway  = msg.linear.y  * GAIN_SWAY
        yaw   = msg.angular.z * GAIN_YAW
        heave = msg.linear.z  * GAIN_HEAVE
        roll  = msg.angular.x * GAIN_ROLL

        self.publish({
            't1': -surge - sway + yaw,
            't2': -surge + sway - yaw,
            't3':  surge - sway - yaw,
            't4':  surge + sway + yaw,
            't5':  heave + roll,
            't6':  heave - roll,
        })

    def publish(self, forces):
        for name, f in forces.items():
            self.pubs[name].publish(
                Float64(data=clamp(f, -MAX_THRUST, MAX_THRUST)))

    def watchdog(self):
        age = (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9
        if age > 0.5:
            self.publish(dict.fromkeys(self.pubs, 0.0))


def main():
    rclpy.init()
    node = ThrusterMixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():          # a real error; a SIGTERM mid-spin is not
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():          # a SIGTERM already shut the context down
            rclpy.shutdown()


if __name__ == '__main__':
    main()
