#!/usr/bin/env python3
# Live depth-sensor ADC readout via MAVROS. Converts the Teensy's DEPTH value
# back to raw ADC counts using the calibration constants currently in the
# firmware, so the pool calibration can be done without the USB console.
#
#   ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyTHS1:115200
#   python3 adc_monitor.py
#
# Update these three numbers whenever the firmware constants change.
ADC_SURFACE = 403.0
ADC_AT_DEPTH = 415.0
CAL_DEPTH_M = 0.3048

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import DebugValue
import time, statistics

SLOPE = CAL_DEPTH_M / (ADC_AT_DEPTH - ADC_SURFACE)  # metres per count

class Mon(Node):
    def __init__(self):
        super().__init__("adc_monitor")
        self.buf = []
        self.last = time.time()
        self.create_subscription(DebugValue, "/mavros/debug_value/named_value_float",
                                 self.cb, qos_profile_sensor_data)
        print("waiting for DEPTH from the Teensy...")

    def cb(self, m):
        if m.name != "DEPTH":
            return
        self.buf.append(m.value_float / SLOPE + ADC_SURFACE)
        if time.time() - self.last >= 1.0:
            b = self.buf
            print(f"adc={statistics.mean(b):6.1f}  min={min(b):6.1f}  max={max(b):6.1f}"
                  f"  n={len(b):2d}   (firmware depth {m.value_float:+.2f} m)", flush=True)
            self.buf = []
            self.last = time.time()

def main():
    rclpy.init()
    n = Mon()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
