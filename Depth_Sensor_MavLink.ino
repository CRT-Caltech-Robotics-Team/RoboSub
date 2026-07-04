#include <Arduino.h>
#include <MAVLink.h>

const int sensorPin = A0;

const double sensorMaxVoltage = 3.3;
const double sensorMaxPressure = 30.0; \\psi
const double psiToMeters = 0.70324961490205;

const uint8_t system_id = 1;
const uint8_t component_id = MAV_COMP_ID_USER1;

HardwareSerial& MAVSerial = Serial1;

void send_depth_to_pixhawk(float current_depth_cm) {
  // Distance Sensor Message Details
  uint32_t time_boot_ms = millis(); // Time since boot
  uint16_t min_distance = 0;        // Minimum distance in cm
  uint16_t max_distance = 10000;    // Maximum distance in cm
  uint16_t current_distance = (uint16_t)current_depth_cm; // Distance in cm
  uint8_t sensor_type = 0;          // 0 = Mav_Distance_Sensor_Laser (or choose appropriate type)
  uint8_t id = 0;                   // Sensor ID
  uint8_t orientation = 0;          // 0 = Pitch 270 (downward facing)
  uint8_t covariance = 255;         // Measurement covariance, 255 if unknown

  // Initialize buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_distance_sensor_pack(
    system_id, component_id, &msg, time_boot_ms, min_distance, max_distance, current_distance, sensor_type, id, orientation, covariance, 0.0f, 0.0f, nullptr, 0
  );

  // Copy to buffer and send via Serial 
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  MAVSerial.write(buf, len); 
}

void setup()
{
    Serial.begin(115200);
    MAVSerial.begin(115200);

    analogRead(10); // 0-1023

    pinMode(sensorPin, INPUT);

    delay(1000);
}

void loop()
{
    int adcValue = analogRead(sensorPin);

    float voltage = adcValue * (sensorMaxVoltage / 1023.0f);

    float pressurePsi =
        (voltage / sensorMaxVoltage) * sensorMaxPressure - 14.696;

    float depthMeters = pressurePsi / 0.0361 / 39.3701; //PSI -> Inches -> meters

    send_depth_to_pixhawk(depthMeters * 100);

    // Serial.print("Voltage: ");
    // Serial.print(voltage, 3);

    // Serial.print(" V, Pressure: ");
    // Serial.print(pressurePsi, 2);

    // Serial.print(" psi, Depth: ");
    // Serial.print(depthMeters, 2);

    // Serial.println(" m");
    Serial.println(depthMeters);

    delay(1000);

}