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

---

# Session Notes — 2026-09-15

Follow-up to the 2026-09-14 pool test: the forward thruster test passed, but the
status-line depth started wrong and ramped to 20 m in a 3 ft pool.

## Diagnosis

The pool log shows body z velocity steady at +0.09 m/s and depth ramping at the
same rate. The EKF had no absolute height source (baro off, GPS off, vision
height never arrived) and was integrating the DVL vertical-velocity bias.
Three stacked faults kept the Teensy's depth from ever reaching the EKF:

1. **Pixhawk did not forward TELEM1 traffic to the Jetson.** `MAV_2_FORWARD`
   (TELEM2 instance) was 0. `MAV_1_CONFIG` was also 101, a duplicate instance
   on TELEM1.
2. **EKF vision mask fused horizontal position** (`EKF2_EV_CTRL=15`) although
   the DVL node sends x/y as placeholders.
3. **Teensy transmit line was dead.** Serial2 TX (pin 8) fails a loopback test.
   The Pixhawk never heard the Teensy: no COMMAND_ACK for the 50 Hz servo
   request, so thrusters were driven from the 1 Hz default stream with the
   watchdog chopping it (the "finicky" motors on 09-14).

Verified along the way: injecting a NAMED_VALUE_FLOAT with the Teensy's exact
identity over the Pixhawk USB link was forwarded to the Jetson, proving PX4
routing and MAVROS were fine once the parameters were set.

## Changes made

| Where | Change |
|---|---|
| PX4 params | `MAV_2_FORWARD` 0 -> 1, `MAV_1_CONFIG` 101 -> 0, `EKF2_EV_CTRL` 15 -> 6, then reboot |
| Teensy wiring | MAVLink moved to **Serial4**: TELEM1 pin 2 (TX) -> Teensy 16, TELEM1 pin 3 (RX) -> Teensy 17, GND -> GND. Pins 7/8 unused. |
| Teensy firmware | `MAVSerial = Serial4`; prints `ACK cmd=511` on COMMAND_ACK; depth read averages 16 samples and uses named two-point constants `ADC_SURFACE`, `ADC_AT_DEPTH`, `CAL_DEPTH_M`; debug line prints raw `adc=` counts. Reference copy: `~/Downloads/TeensySensorThrusterBreakOutBoard/`. Flashed from the laptop. |
| Jetson | `adc_monitor.py` in `prototypes/SummerTesting/MAVROSPixhawkDVLTest/` prints the sensor ADC count once a second via MAVROS (no USB console needed). |

Result: `stream=OK` steady on the Teensy console, and `DEPTH` arrives on
`/mavros/debug_value/named_value_float` at ~10 Hz.

## Gotchas learned

- Teensy 4.1 `A0` is pin 14. Serial3 TX is also pin 14, so Serial3 cannot be
  used while the depth sensor is on A0 (`pinMode(A0, INPUT)` kills the UART).
- Teensy 4.1 analog pins are 3.3 V max. The transducer outputs 0-5 V.
- PX4's MAVLink shell (SERIAL_CONTROL) does not answer on this 1.17 build.
- `ros2 topic echo` can fail with a daemon xmlrpc error; add `--no-daemon`.
- Two MAVLink instances on one port (`MAV_0_CONFIG == MAV_1_CONFIG`) is silent
  misconfiguration; check both when a link misbehaves.

## Depth sensor status (open)

Transducer: 0-5 V output, ~11.7 V excitation measured (OK). Raw output reads
**4.5-4.6 V in air**, which is either a faulty/offset gauge sensor or an
absolute sensor with almost no range left (a 0-15 psia unit fits). A "filter"
board reduces the signal; its output on Teensy pin 14 reads 0.04 V in air and
did not change when the port was pressed. Part number not yet read.

## Future steps

1. **Depth sensor.** With a meter while pressing/blowing on the port: raw
   output, filter-board output, filter-board supply. Read the part number.
   If the sensor is absolute or saturated, replace with a gauge sensor of
   ~0-5 or 0-10 psi range. Whatever the signal, it needs a divider before
   pin 14 (10 k series, 15 k to GND, 0.1 uF across).
2. **Calibrate.** Run `adc_monitor.py` at the pool: `adc=` in air, `adc=` at a
   measured depth. Put the three constants in the firmware. Switch to 12-bit
   ADC if the count span is small.
3. **Pool: confirm fusion.** Status-line depth should hold, not ramp.
4. **Vertical thruster check** (add a downward nudge to `thruster_check`).
5. **Depth hold** in `offboard_node`: capture x/y/yaw on engage, stream a
   position setpoint with z = target depth. Test at 0.4-0.5 m.
6. **5 m forward** = same setpoint offset along the captured heading.
7. Later: DVL vertical-velocity bias (~0.09 m/s stationary), pin 8 post-mortem,
   change Teensy component id away from MAVROS's 191.

## Reference: pool-day startup

```
# Terminal 1 — Pixhawk link
ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyTHS1:115200
# Terminal 2 — DVL + static TF
ros2 run my_mav_node myprogram
# Terminal 3 — status line
ros2 run my_mav_node status_node
# Terminal 4 — depth sensor ADC (calibration only)
python3 ~/Robosub/prototypes/SummerTesting/MAVROSPixhawkDVLTest/adc_monitor.py
```
