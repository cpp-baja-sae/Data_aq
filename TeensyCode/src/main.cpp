
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include <cmath>
#include <TimeLib.h>
#include <FlexCAN_T4.h>
#include <Adafruit_LSM6DSOX.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

constexpr uint32_t POTENTIOMETER_ID = 0x100;
constexpr uint32_t ACCELEROMETER_ID = 0x101;

void canDecode(const CAN_message_t &msg) {
  switch (msg.id) {
    case POTENTIOMETER_ID: {
      //Potentiometer is a 4 Byte payload
      if (msg.len != 4) {
        Serial.print("Error: Potentiometer message length is not 4 bytes, it is ");
        Serial.println(msg.len);
        break;
      }
      uint16_t voltage_mV = (static_cast<uint16_t>(msg.buf[1]) << 8) | static_cast<uint16_t>(msg.buf[0]);
      uint8_t status = msg.buf[2];
      uint8_t counter = msg.buf[3];
      // Convert mV to V
      float voltage = voltage_mV / 1000.0f;
        Serial.print("Potentiometer Voltage: ");
        Serial.print(voltage);
        Serial.print(" V, Status: ");
        Serial.print(status);
        Serial.print(", MessageCounter: ");
        Serial.println(counter);
      break;
    }
    case ACCELEROMETER_ID: {
    // Accelerometer is a 5 Byte payload
      if (msg.len != 5) {
        Serial.print("Error: Accelerometer message length is not 5 bytes, it is ");
        Serial.println(msg.len);
        break;
      }
      int8_t ax_encoded = static_cast<int8_t>(msg.buf[0]);
      int8_t ay_encoded = static_cast<int8_t>(msg.buf[1]);
      int8_t az_encoded = static_cast<int8_t>(msg.buf[2]);
      uint8_t status = msg.buf[3];
      uint8_t counter = msg.buf[4];
      // Convert the encoded values back to G's
      float ax_g = ax_encoded/8.0f; // Assuming the encoding is in 1/8 G increments
      float ay_g = ay_encoded/8.0f;
      float az_g = az_encoded/8.0f;
        Serial.print("Ax: ");
        Serial.print(ax_g);
        Serial.print(" G, Ay: ");
        Serial.print(ay_g);
        Serial.print(" G, Az: ");
        Serial.print(az_g);
        Serial.print(" G, Status: ");
        Serial.print(status);
        Serial.print(", MessageCounter: ");
        Serial.println(counter);
      break;
    }
    case GYROSCOPE_ID: {
      // Accelerometer + Gyroscope is a 8 Byte payload
      if (msg.len != 8) {
        Serial.print("Error: Gyroscope message length is not 8 bytes, it is ");
        Serial.println(msg.len);
        break;
      }
      int8_t ax_encoded = static_cast<int8_t>(msg.buf[0]);
      int8_t ay_encoded = static_cast<int8_t>(msg.buf[1]);
      int8_t az_encoded = static_cast<int8_t>(msg.buf[2]);
      int8_t gx_encoded = static_cast<int8_t>(msg.buf[3]);
      int8_t gy_encoded = static_cast<int8_t>(msg.buf[4]);
      int8_t gz_encoded = static_cast<int8_t>(msg.buf[5]);
      uint8_t status = msg.buf[6];
      uint8_t counter = msg.buf[7];
      // Convert the encoded values back to G's and rad/s
      float ax_g = ax_encoded/8.0f; // Assuming the encoding is in 1/8 G increments
      float ay_g = ay_encoded/8.0f;
      float az_g = az_encoded/8.0f;
      float gx_deg_s = gx_encoded*4.0f;
      float gy_deg_s = gy_encoded*4.0f;
      float gz_deg_s = gz_encoded*4.0f; // Assuming the encoding is in 4 deg/s increments, 500 degrees/s. 
        Serial.print("Ax: ");
        Serial.print(ax_g);
        Serial.print(" G, Ay: ");
        Serial.print(ay_g);
        Serial.print(" G, Az: ");
        Serial.print(az_g);
        Serial.print(" G, Gx: ");
        Serial.print(gx_deg_s);
        Serial.print(" deg/s, Gy: ");
        Serial.print(gy_deg_s);
        Serial.print(" deg/s, Gz: ");
        Serial.print(gz_deg_s);
        Serial.print(" deg/s, Status: ");
        Serial.print(status);
        Serial.print(", MessageCounter: ");
        Serial.println(counter);
      break;
    }
    default:
      Serial.print("Unknown message ID: ");
      Serial.println(msg.id, HEX);
      break;
  }
}

void setup() {
   Serial.begin(115200);
   can1.begin();
   can1.setBaudRate(500000);
   can1.setMaxMB(16);
   can1.enableFIFO();
   can1.enableFIFOInterrupt();
   can1.onReceive(canDecode);
}

void loop() {
  can1.events();
}
