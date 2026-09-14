# Startup Guide — Sub Bring-up

How to bring the sub up from power-on to a test, and shut it down safely.
Companion to [SESSION_NOTES.md](SESSION_NOTES.md).

Every new terminal needs these two lines first:

```
source /opt/ros/humble/setup.bash
source ~/Robosub/prototypes/SummerTesting/MAVROSPixhawkDVLTest/install/setup.bash
```

> Always source the `MAVROSPixhawkDVLTest` workspace, not the older
> `~/Robosub/install` one, or you will run the unfixed DVL node.

## 1. Before power

- Pixhawk on TELEM2 (`/dev/ttyTHS1`), DVL on `/dev/ttyUSB0`, Teensy driving the
  thrusters, Jetson powered.
- Thrusters out of water are for brief checks only.

## 2. Start the core stack (separate terminals)

```
# Terminal 1 — link to Pixhawk. Wait for "Got HEARTBEAT, connected".
ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyTHS1:115200

# Terminal 2 — DVL + static transforms
ros2 run my_mav_node myprogram

# Terminal 3 — live status
ros2 run my_mav_node status_node
```

One-shot alternative (starts MAVROS + DVL + offboard and **auto-arms**, so only
in water):

```
ros2 launch my_mav_node bringup.launch.py
```

## 3. Verify before doing anything

In the `status_node` line, confirm:

- Not `NO LINK` — MAVROS is talking to the Pixhawk.
- `vel [ok]` with small numbers — the DVL has bottom lock and is being fused.
  `[no]` or `--` means do not attempt autonomous modes.

## 4. Pick one operation

**A. Dry bench thruster check** (brief, out of water):
```
ros2 run my_mav_node thruster_check --ros-args -p throttle:=1.0 -p hold_s:=2.0
```
Arms in MANUAL, pushes forward, disarms. Stand clear. A forward push should show
outputs 1-4 rising above 1500 on `/mavros/rc/out`.

**B. Autonomous arm and hold** (in water):
```
ros2 run my_mav_node offboard_node
```

**C. Manual flying** (in water): switch to STABILIZED and use the joystick. Do
not run `offboard_node` at the same time.

Run only one of A, B, or C at a time.

## 5. Shut down safely

1. Ctrl-C the operation node.
2. Confirm it disarmed; if not:
   ```
   ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
   ```
3. Ctrl-C the status, DVL, and MAVROS terminals.
4. Power off the vehicle.

## Key cautions

- **Never leave it armed unattended.** Ctrl-C stops a node but does not disarm
  the Pixhawk.
- **Offboard needs continuous DVL velocity.** On a dry bench it arms then
  bounces into a failsafe. Expected, not a fault.
- **Thruster gains** were raised from PX4 defaults: `UUV_MGM_THRTL = 0.5`,
  `UUV_THRUST_SAT = 0.3`. Lower them if manual control feels too strong.

## Useful monitoring commands

```
ros2 topic echo /mavros/state              # armed / mode / connection
ros2 topic echo /mavros/statustext/recv    # FCU messages in plain English
ros2 topic echo /mavros/estimator_status   # EKF health flags
ros2 topic echo /mavros/rc/out             # servo outputs the Teensy reads
```
