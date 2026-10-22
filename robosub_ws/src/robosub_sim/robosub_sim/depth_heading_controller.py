#!/usr/bin/env python3
"""Hold a depth, a heading, a roll, and (with a DVL) a position. PIDs on /sub/state.

This is the first closed loop on the sub. Everything the vehicle does at
competition (dive to 1.5 m, turn to face the gate, drive through it) is
built from "hold this depth, hold this heading, and meanwhile go forward".

    inputs
        /sub/state             nav_msgs/Odometry   from state_estimator
        /sub/setpoint/depth    std_msgs/Float64    metres, +down
        /sub/setpoint/heading  std_msgs/Float64    yaw, rad, CCW +, 0 = where
                                                   the estimator started
        /sub/setpoint/roll     std_msgs/Float64    roll, rad, + = port side
                                                   down (left roll)
        /sub/setpoint/roll_turns std_msgs/Float64  do this many full rolls
                                                   (1.0 = one 360, - = other
                                                   way), then level out
        /sub/setpoint/position geometry_msgs/Point  x, y in the estimator's
                                                   frame (metres from where
                                                   it started; z ignored)
        /sub/setpoint/move     std_msgs/Float64    "go this many metres
                                                   forward" along the
                                                   heading setpoint (or the
                                                   current heading if none):
                                                   the mission-level move
        /sub/cmd_vel_manual    geometry_msgs/Twist optional: linear.x / y
                                                   (surge / sway) pass
                                                   straight through so you
                                                   can drive while it holds.
                                                   A non-zero manual command
                                                   CANCELS position mode.
    output
        /sub/cmd_vel           geometry_msgs/Twist -> thruster_mixer

LIFE CYCLE. The node starts IDLE and publishes nothing, so the mixer's
watchdog keeps the thrusters at zero and you can still drive by hand on
/sub/cmd_vel exactly as before. The first setpoint on either topic arms
that loop and the node starts owning /sub/cmd_vel at `rate` Hz. From then
on, drive surge/sway through /sub/cmd_vel_manual, not /sub/cmd_vel (two
publishers on one topic just fight).

If /sub/state goes stale for more than 0.5 s the node stops publishing:
no estimate, no thrust. Never remove that.

POSITION MODE needs the DVL. The estimator flags pose.covariance[0] = -1
when the DVL is silent; the controller then commands zero surge/sway
rather than pushing blind on a frozen position. The world-frame error is
rotated into the body frame by yaw, so the sub drives toward the point
without having to face it (it is holonomic in the plane).

ROLL is different from the other loops: the sub is self-righting (centre
of buoyancy above centre of gravity), so holding any non-zero roll means
pushing against a righting torque of about  20 N.m * sin(roll)  forever.
Two things handle that. A feedforward term  roll_ff * sin(roll)  cancels
the known part at every angle (roll_ff = righting moment / thrust torque
per unit ~ 20 / 35), and ki mops up what is left. Full thrust on the two
verticals is ~35 N.m, so any angle is reachable, but a hard roll leaves
nothing for depth (same two thrusters).

A 360 (barrel roll) cannot be a setpoint, because angles wrap. Instead
/sub/setpoint/roll_turns ramps an UNWRAPPED setpoint at roll_spin_rate
through the full turn, tracking the unwrapped roll, then hands back to the
ordinary roll hold at the angle it started from. While the sub is well off
level, the heave and yaw commands are scaled by cos(roll): upside down,
"up" on the body's thrusters is down in the world, and the depth loop must
not fight itself (or wind up) during the manoeuvre.

SIGNS. Depth is +down; the mixer's heave is +up. So a positive depth error
("too shallow, go deeper") becomes a NEGATIVE linear.z. The heading error
is wrapped to [-pi, pi] so the sub always turns the short way round.

TUNING. Gains are in "mixer units per metre / per radian"; 1.0 on the
output saturates the thrusters (see GAIN_* in thruster_mixer.py).
    depth:   kp 1.0 is 60 N per metre of error. The water damps heave hard
             (zW = -120 N per m/s) so little D is needed. Ki is what beats
             the ~5 N of positive buoyancy: without it the sub parks a few
             centimetres shallow of the setpoint forever.
    heading: kp 1.0 is ~32 N.m per radian. Yaw damping (nR = -25) is
             strong, so kd stays small. No steady yaw disturbance in the
             pool, so ki defaults to 0.
    roll:    unit output is now full thrust (~35 N.m), so gains are small:
             kp 0.75 (~26 N.m/rad), ki 0.5, kd 0.08. roll_ff 0.56 carries
             the righting moment. roll_spin_rate 0.6 rad/s: at that rate
             water drag (~9 N.m) + righting (~20 N.m) still fits in 35.
    position: kp 0.5 is ~42 N per metre of error; with kd 0.3 and the
             water's own drag that is close to critically damped. Output is
             capped at pos_limit so a far-away point means "cruise", not
             "full throttle".
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from robosub_sim.state_estimator import q_to_rpy


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_pi(a):
    return (a + math.pi) % (2*math.pi) - math.pi


class PID:
    """u = kp*e + ki*int(e) - kd*rate.  Derivative acts on the MEASURED rate
    (which the estimator already gives us), not on d(error)/dt, so a step
    in the setpoint does not produce a spike."""

    def __init__(self, kp, ki, kd, i_limit, out_limit=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.out_limit = i_limit, out_limit
        self.integral = 0.0

    def step(self, error, rate, dt, integrate=True):
        if integrate:
            self.integral = clamp(self.integral + error*dt,
                                  -self.i_limit, self.i_limit)
        u = self.kp*error + self.ki*self.integral - self.kd*rate
        return clamp(u, -self.out_limit, self.out_limit)

    def reset(self):
        self.integral = 0.0


class DepthHeadingController(Node):
    def __init__(self):
        super().__init__('depth_heading_controller')
        d = self.declare_parameter
        d('rate', 20.0)
        d('depth_kp', 1.0);   d('depth_ki', 0.2);   d('depth_kd', 0.3)
        d('heading_kp', 1.0); d('heading_ki', 0.0); d('heading_kd', 0.1)
        d('pos_kp', 0.5);     d('pos_kd', 0.3);     d('pos_limit', 0.5)
        d('roll_kp', 0.75);   d('roll_ki', 0.5);    d('roll_kd', 0.08)
        d('roll_ff', 0.56);   d('roll_spin_rate', 0.6)
        d('i_limit', 1.0)
        d('state_timeout', 0.5)
        g = lambda n: float(self.get_parameter(n).value)

        self.rate = g('rate')
        self.state_timeout = g('state_timeout')
        self.depth_pid = PID(g('depth_kp'), g('depth_ki'), g('depth_kd'), g('i_limit'))
        self.heading_pid = PID(g('heading_kp'), g('heading_ki'), g('heading_kd'), g('i_limit'))
        self.roll_pid = PID(g('roll_kp'), g('roll_ki'), g('roll_kd'), g('i_limit'))
        self.roll_ff = g('roll_ff')
        self.spin_rate = g('roll_spin_rate')
        self.surge_pid = PID(g('pos_kp'), 0.0, g('pos_kd'), 0.0, g('pos_limit'))
        self.sway_pid = PID(g('pos_kp'), 0.0, g('pos_kd'), 0.0, g('pos_limit'))

        self.depth_sp = None
        self.heading_sp = None
        self.pos_sp = None          # (x, y) or None
        self.roll_sp = None
        self.prev_roll = None
        self.roll_unwrapped = 0.0   # continuous roll angle, for multi-turn moves
        self.spin_sp = None         # ramping unwrapped setpoint during a roll_turns
        self.spin_end = None        # where the ramp stops (unwrapped)
        self.state = None
        self.state_rx = None        # wall time we last heard from the estimator
        self.manual = Twist()
        self.manual_rx = None

        self.pub = self.create_publisher(Twist, '/sub/cmd_vel', 10)
        self.create_subscription(Odometry, '/sub/state', self.on_state, 10)
        self.create_subscription(Float64, '/sub/setpoint/depth', self.on_depth_sp, 10)
        self.create_subscription(Float64, '/sub/setpoint/heading', self.on_heading_sp, 10)
        self.create_subscription(Float64, '/sub/setpoint/roll', self.on_roll_sp, 10)
        self.create_subscription(Float64, '/sub/setpoint/roll_turns', self.on_roll_turns, 10)
        self.create_subscription(Point, '/sub/setpoint/position', self.on_pos_sp, 10)
        self.create_subscription(Float64, '/sub/setpoint/move', self.on_move, 10)
        self.create_subscription(Twist, '/sub/cmd_vel_manual', self.on_manual, 10)
        self.create_timer(1.0/self.rate, self.control)

        self.get_logger().info(
            'depth_heading_controller up, IDLE until a setpoint arrives on '
            '/sub/setpoint/depth or /sub/setpoint/heading')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg):
        self.state = msg
        self.state_rx = self.now()

    def on_depth_sp(self, msg):
        if self.depth_sp is None:
            self.get_logger().info('depth loop armed')
        self.depth_sp = msg.data

    def on_heading_sp(self, msg):
        if self.heading_sp is None:
            self.get_logger().info('heading loop armed')
        self.heading_sp = msg.data

    def on_roll_sp(self, msg):
        if self.roll_sp is None:
            self.get_logger().info('roll loop armed')
        self.roll_sp = msg.data
        self.spin_end = None            # a plain setpoint cancels a spin

    def on_roll_turns(self, msg):
        if self.state is None or self.prev_roll is None:
            self.get_logger().warn('roll_turns ignored: no /sub/state yet')
            return
        if self.roll_sp is None:
            self.get_logger().info('roll loop armed')
            self.roll_sp = 0.0
        self.spin_sp = self.roll_unwrapped
        self.spin_end = self.roll_unwrapped + 2*math.pi*msg.data
        self.roll_pid.reset()
        self.get_logger().info(f'rolling {msg.data:+.2f} turns at {self.spin_rate} rad/s')

    def on_pos_sp(self, msg):
        if self.pos_sp is None:
            self.get_logger().info('position loop armed')
        self.pos_sp = (msg.x, msg.y)
        self.get_logger().info(f'go to ({msg.x:.2f}, {msg.y:.2f})')

    def on_move(self, msg):
        """Forward `data` metres along the heading setpoint (or current yaw)."""
        if self.state is None:
            self.get_logger().warn('move ignored: no /sub/state yet')
            return
        q = self.state.pose.pose.orientation
        _, _, yaw = q_to_rpy((q.w, q.x, q.y, q.z))
        h = self.heading_sp if self.heading_sp is not None else yaw
        x0 = self.state.pose.pose.position.x
        y0 = self.state.pose.pose.position.y
        target = Point(x=x0 + msg.data*math.cos(h), y=y0 + msg.data*math.sin(h))
        self.on_pos_sp(target)

    def on_manual(self, msg):
        self.manual = msg
        self.manual_rx = self.now()
        if self.pos_sp is not None and (msg.linear.x != 0.0 or msg.linear.y != 0.0):
            self.get_logger().info('manual surge/sway received: position mode cancelled')
            self.pos_sp = None

    def control(self):
        if (self.depth_sp is None and self.heading_sp is None
                and self.pos_sp is None and self.roll_sp is None):
            return                              # idle: leave /sub/cmd_vel alone
        if self.state is None or self.now() - self.state_rx > self.state_timeout:
            self.get_logger().warn('no fresh /sub/state; not commanding',
                                   throttle_duration_sec=2.0)
            self.depth_pid.reset()
            self.heading_pid.reset()
            self.roll_pid.reset()
            return

        dt = 1.0/self.rate
        cmd = Twist()

        depth = -self.state.pose.pose.position.z
        heave_up = self.state.twist.twist.linear.z
        q = self.state.pose.pose.orientation
        roll, _, yaw = q_to_rpy((q.w, q.x, q.y, q.z))
        yaw_rate = self.state.twist.twist.angular.z
        roll_rate = self.state.twist.twist.angular.x

        # Keep a continuous roll angle so a 360 is not a no-op.
        if self.prev_roll is not None:
            self.roll_unwrapped += wrap_pi(roll - self.prev_roll)
        self.prev_roll = roll
        upright = math.cos(roll)             # +1 level, 0 on its side, -1 inverted

        if self.spin_end is not None:
            # Ramp the unwrapped setpoint toward spin_end at spin_rate.
            direction = 1.0 if self.spin_end >= self.spin_sp else -1.0
            self.spin_sp += direction * self.spin_rate * dt
            done_ramp = (direction > 0 and self.spin_sp >= self.spin_end) or \
                        (direction < 0 and self.spin_sp <= self.spin_end)
            if done_ramp:
                self.spin_sp = self.spin_end
            rate_sp = 0.0 if done_ramp else direction * self.spin_rate
            err = self.spin_sp - self.roll_unwrapped
            u = self.roll_pid.step(err, roll_rate - rate_sp, dt)
            cmd.angular.x = clamp(u + self.roll_ff * math.sin(roll), -1.0, 1.0)
            if done_ramp and abs(err) < 0.15:
                self.get_logger().info('roll manoeuvre complete')
                self.roll_sp = wrap_pi(self.spin_end)
                self.spin_end = None
                self.roll_pid.reset()
        elif self.roll_sp is not None:
            err = wrap_pi(self.roll_sp - roll)
            u = self.roll_pid.step(err, roll_rate, dt)
            cmd.angular.x = clamp(u + self.roll_ff * math.sin(roll), -1.0, 1.0)

        if self.pos_sp is not None:
            if self.state.pose.covariance[0] < 0.0:
                self.get_logger().warn('position mode but no DVL: holding still',
                                       throttle_duration_sec=2.0)
            else:
                ex = self.pos_sp[0] - self.state.pose.pose.position.x
                ey = self.pos_sp[1] - self.state.pose.pose.position.y
                c, s_ = math.cos(yaw), math.sin(yaw)
                ex_b = c*ex + s_*ey            # error in the body frame
                ey_b = -s_*ex + c*ey
                vx_b = self.state.twist.twist.linear.x
                vy_b = self.state.twist.twist.linear.y
                cmd.linear.x = self.surge_pid.step(ex_b, vx_b, dt)
                cmd.linear.y = self.sway_pid.step(ey_b, vy_b, dt)
        elif self.manual_rx is not None and self.now() - self.manual_rx < 0.5:
            # Surge / sway from the operator, if recent.
            cmd.linear.x = self.manual.linear.x
            cmd.linear.y = self.manual.linear.y

        if self.depth_sp is not None:
            err_down = self.depth_sp - depth            # +: too shallow
            u_down = self.depth_pid.step(err_down, -heave_up, dt,
                                         integrate=abs(upright) > 0.7)
            # Body "up" is only  cos(roll)  of world up: scale so an inverted
            # sub pushes the right way and one on its side does not fight.
            cmd.linear.z = -u_down * upright           # mixer heave is +up

        if self.heading_sp is not None:
            err = wrap_pi(self.heading_sp - yaw)
            cmd.angular.z = self.heading_pid.step(err, yaw_rate, dt) * upright

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = DepthHeadingController()
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
