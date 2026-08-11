#include <ArduinoRS485.h>
#include <CRC32.h>
#include <SoftwareSerial.h>
#include <MAVLink.h>
//need to conver to arrays since Arduino does not support vector
//there is also custom library Vector that is similar to standard c++ vectors, could be an option too
//then make a function that creates the packet to send
using namespace std;

#define DE 3
#define RE 2
#define ESP32_COMPONENT_ID 50
#define SYSTEM_ID 1

HardwareSerial &MAVSerial = Serial1;
HardwareSerial &RS485Serial = Serial2;

uint16_t SYNC_REQUEST  =  0x5FF5;
uint16_t SYNC_RESPONSE =  0x0FF0;
int16_t PROTOCOL_VRCSR_HEADER_SIZE = 6;
int PROTOCOL_VRCSR_XSUM_SIZE   = 4;
uint8_t RESPONSE_THRUSTER_STANDARD = 0x2;
int RESPONSE_THRUSTER_STANDARD_LENGTH = 1 + 4 * 4 + 1;
uint8_t PROPULSION_COMMAND = 0xAA;
uint8_t ADDR_CUSTOM_COMMAND = 0xF0;
uint8_t NODE_ID = 0x81;
uint8_t MOTOR_ID = 0x01;
const int NUM_THRUSTERS = 8;
const int HEADER_LENGTH = 6;
const int PAYLOAD_LENGTH = 2 + 4 * NUM_THRUSTERS;
const int CHECKSUM_LENGTH = 4;
const int PACKET_LENGTH = HEADER_LENGTH + CHECKSUM_LENGTH + PAYLOAD_LENGTH + CHECKSUM_LENGTH;
// vector<int> motors;

struct __attribute__((packed)) Header {
  uint16_t sync;
  uint8_t node_id;
  uint8_t flag;
  uint8_t csr_addr;
  uint8_t payload_length;
};

struct __attribute__((packed)) Payload {
  uint8_t propulsion;
  uint8_t motor_id;
  float thrusts[NUM_THRUSTERS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

void make_thrust_packet(uint8_t* packet, float thrust_values[NUM_THRUSTERS]) {
  Header hd;
  hd.sync = SYNC_REQUEST;
  hd.node_id = NODE_ID;
  hd.flag = RESPONSE_THRUSTER_STANDARD;
  hd.csr_addr = ADDR_CUSTOM_COMMAND;
  hd.payload_length = 2 + NUM_THRUSTERS * 4;

  Payload py;
  py.propulsion = PROPULSION_COMMAND;
  py.motor_id = MOTOR_ID;
  //py.thrusts = thrust_values;
  memcpy(py.thrusts, thrust_values, NUM_THRUSTERS * sizeof(float));

  uint8_t header_buf[HEADER_LENGTH];
  uint8_t payload_buf[PAYLOAD_LENGTH];
  memcpy(header_buf, &hd, HEADER_LENGTH);
  memcpy(payload_buf, &py, PAYLOAD_LENGTH);

  CRC32 crc;
  for (int i = 0; i < HEADER_LENGTH; i++){
    uint8_t byte = header_buf[i];
    crc.update(byte);
  }

  uint32_t hcrc = crc.finalize();
  crc.reset();

  for (int i = 0; i < PAYLOAD_LENGTH; i++){
    uint8_t byte = payload_buf[i];
    crc.update(byte);
  }

  uint32_t pcrc = crc.finalize();
  crc.reset();

  uint8_t hcrc_buf[4];
  uint8_t pcrc_buf[4];
  memcpy(hcrc_buf, &hcrc, 4);
  memcpy(pcrc_buf, &pcrc, 4);

  memcpy(packet, header_buf, HEADER_LENGTH);
  memcpy(packet + HEADER_LENGTH, hcrc_buf, CHECKSUM_LENGTH);
  memcpy(packet + HEADER_LENGTH + CHECKSUM_LENGTH, payload_buf, PAYLOAD_LENGTH);
  memcpy(packet + HEADER_LENGTH + CHECKSUM_LENGTH + PAYLOAD_LENGTH, pcrc_buf, CHECKSUM_LENGTH);
};

void drive(float thrust_values[NUM_THRUSTERS], int de, int re, HardwareSerial &RS485){
  uint8_t packet[PACKET_LENGTH];
  make_thrust_packet(packet, thrust_values);
  digitalWrite(de, HIGH);
  digitalWrite(re, HIGH);
  RS485.write(packet, PACKET_LENGTH);
  RS485.flush();
  digitalWrite(de, LOW);
  digitalWrite(re, LOW);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  MAVSerial.begin(115200); //Mavlink Communication From Telem 2
  RS485Serial.begin(115200);
  pinMode(DE, OUTPUT);
  pinMode(RE, OUTPUT);

  Serial.println("Starting");

  //RS485.receive();
  // unsigned long startTime = millis();
  // unsigned long interval = 5000;
  // float thrusts[NUM_THRUSTERS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // uint8_t packet[PACKET_LENGTH];
}

float pwm_to_thrust(int pwm_val){
  uint16_t max_pwm = 2000; //microseconds
  uint16_t min_pwm = 1000; //microseconds
  float max_output = 1.0;
  float min_output = -1.0;
  return max_output - (max_output - min_output) * ((float)(max_pwm - pwm_val)/(max_pwm - min_pwm));
}

void loop() {
  // put your main code here, to run repeatedly:
  mavlink_message_t msg;
  mavlink_status_t status;
  uint16_t pwm[NUM_THRUSTERS];
  float thrusts[NUM_THRUSTERS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // Read MAVLink messages from Serial
  while (MAVSerial.available() > 0) {
    uint8_t c = MAVSerial.read();
    //Serial.println(c, HEX);
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      //Serial.println("Packet decoded");
      if (msg.msgid == MAVLINK_MSG_ID_SERVO_OUTPUT_RAW) {
        //Serial.println("Thrust Packet Received");
        mavlink_servo_output_raw_t servo;
        mavlink_msg_servo_output_raw_decode(&msg, &servo);
        //Serial.println(servo.servo1_raw);
        //Serial.println(servo.servo2_raw);
        pwm[0] = servo.servo1_raw;
        pwm[1] = servo.servo2_raw;
        pwm[2] = servo.servo3_raw;
        pwm[3] = servo.servo4_raw;
        pwm[4] = servo.servo5_raw;
        pwm[5] = servo.servo6_raw;
        pwm[6] = servo.servo7_raw;
        pwm[7] = servo.servo8_raw;
        for (int i = 0; i < 8; i++) {
          thrusts[i] = pwm_to_thrust(pwm[i]);
          Serial.println(thrusts[i]);
        }
        drive(thrusts, DE, RE, RS485Serial);
      }
    }
  }
  if (RS485Serial.available()) {
    uint8_t b = RS485Serial.read();
    Serial.write(b);  // print response to USB Serial
  }
}
