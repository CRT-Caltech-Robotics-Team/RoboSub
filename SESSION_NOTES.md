# Session Notes — 2026-09-14

Debugging the Pixhawk 4 arming and thruster bring-up on the sub (PX4 v1.17,
Jetson + MAVROS, DVL, Teensy thruster driver).

## Starting problem

The Pixhawk would not arm.

## Root cause and fixes (arming)

The Jetson's DVL data never reached the flight controller, so its estimator had
no valid position, velocity, or altitude, and every arming path was blocked.

1. **Installed `ros-humble-mavros-extras`.** Core fix. Without it there was no
   plugin to forward DVL odometry to the Pixhawk.
2. **Fixed the DVL node (`myprogram.cpp`).** It fed the estimator a fake
   velocity when it lost bottom lock, and — worse — when the required static
   transforms were missing, the MAVROS odometry plugin multiplied the data by
   uninitialized memory and sent garbage. The rebuilt node now:
   - broadcasts the `odom -> odom_ned` and `base_link -> base_link_frd`
     transforms itself,
   - drops bad DVL readings (PD0 bad-velocity sentinel),
   - marks its data velocity-only (position + orientation NaN) so PX4 fuses
     only velocity.

After these, the estimate went valid and the vehicle armed.

## Nodes built in `my_mav_node`

- `myprogram` — DVL to Pixhawk with lever-arm correction, plus the static TF.
- `offboard_node` — streams setpoints, engages Offboard, arms.
- `status_node` — one-line live status: armed, mode, x/y/z velocity, attitude, depth.
- `thruster_check` — open-loop MANUAL-mode forward nudge (arm, push, disarm).
- `bringup.launch.py` — brings up MAVROS + DVL + offboard together.

## Thruster investigation

After arming worked, the thrusters would not spin. The PX4 UUV manual gains were
tiny, so throttle produced almost no thrust:

| Parameter | Old | Changed to |
|---|---|---|
| UUV_MGM_THRTL | 0.1 | 0.5 |
| UUV_THRUST_SAT | 0.1 | 0.3 |

With those raised, the motors spun.

## Current status

Spin became intermittent. A servo capture during a no-spin run proved the
Pixhawk side is working:

```
during push:  channels [1625,1625,1625,1625, 1500,1500,1500,1500, ...]
peak:         channels [1679,1679,1679,1679, 1500,1500,1500,1500, ...]
after:        channels [1500,1500,1500,1500, 1500,1500,1500,1500, ...]
```

Outputs 1–4 ramp to 1679 (forward push), 5–8 stay neutral. Since the Pixhawk
output is correct even when the motors don't turn, the remaining fault is
**downstream: Teensy, RS485, ESCs, or thruster power.** A forward push should
look like outputs 1–4 at ~1625–1679 and 5–8 at 1500.

## Future steps

1. **Debug the Teensy path (current blocker).** Enable the commented-out
   `Serial.println` lines in `Thruster_MavLink.ino`; confirm during a run that
   it decodes `SERVO_OUTPUT_RAW` and reads ~1679 on the forward channels.
   - Sees them but no spin -> ESCs or power.
   - Doesn't see them -> TELEM1 link or MAVLink parsing.
2. **Suspect ESC arming** for the intermittency ("spun twice then stopped").
   Confirm the ESCs get a steady neutral at startup and a continuous signal
   between runs.
3. **Verify thruster mapping.** Pixhawk drives 8 outputs; the Teensy splits into
   6. Make sure outputs 1–4 map to the thrusters that produce forward motion,
   and check spin direction.
4. **Then test in water.** Offboard position hold and DVL-based motion only work
   where the DVL keeps continuous bottom lock. On the bench it arms then
   failsafes, which is expected.

## Loose ends

- **Height source:** EKF height reference (`EKF2_HGT_REF`) is set to vision, but
  no vision height is sent. Confirm where depth/altitude actually comes from.
- **Battery telemetry:** `/mavros/battery` reports nothing (status line shows
  `batt --`). Wire up the power module.
- **Workspaces:** an older, unfixed build still exists in `~/Robosub/install`.
  Always source
  `~/Robosub/prototypes/SummerTesting/MAVROSPixhawkDVLTest/install/setup.bash`.

## Reference: startup order

```
# Terminal 1 — link to Pixhawk (wait for "Got HEARTBEAT, connected")
ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyTHS1:115200

# Terminal 2 — DVL + static TF (needed for autonomous modes, not for thruster_check)
ros2 run my_mav_node myprogram

# Terminal 3 — live status
ros2 run my_mav_node status_node

# Then one of:
ros2 run my_mav_node thruster_check --ros-args -p throttle:=1.0 -p hold_s:=2.0   # dry bench
ros2 run my_mav_node offboard_node                                              # in water

# Disarm:
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```
