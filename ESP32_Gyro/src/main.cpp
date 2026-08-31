

#include <TimeLib.h>
#include <SD.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_LSM6DSOX.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/TomThumb.h>
#include <ESP32-TWAI-CAN.hpp>
#include <Adafruit_ADXL345_U.h>

#define CAN_TX 2
#define CAN_RX 3

// Initialize your sensor (find the online library, each has a different initialization method)
Adafruit_LSM6DSOX gyro;

// Ignore this for now
const uint8_t status = 0x05; // Example status byte 00000101 represents GOOD!

// Replace the parameters in sendMessage(a, b, c, d, e, f, g, h) with the actual values you want to send.
// If your value is STRICTLY POSITIVE, use uint8_t. This will give you 0-255 range. If your value can be negative, use int8_t. This will give you -128 to 127 range.
void sendMessage(int8_t ax_g, int8_t ay_g, int8_t az_g, int8_t x_dps, int8_t y_dps, int8_t z_dps, uint8_t status){
  static uint8_t count = 0;
  CanFrame txFrame = {0};
  txFrame.identifier = 0x102; // Matches the gyroscope data message identifier
  txFrame.extd = 0; // Indicates 11 bit id ^
  txFrame.data_length_code = 8; // 8 Bytes
  txFrame.data[0] = static_cast<uint8_t>(ax_g); 
  txFrame.data[1] = static_cast<uint8_t>(ay_g); 
  txFrame.data[2] = static_cast<uint8_t>(az_g);
  txFrame.data[3] = static_cast<uint8_t>(x_dps);
  txFrame.data[4] = static_cast<uint8_t>(y_dps);
  txFrame.data[5] = static_cast<uint8_t>(z_dps);
  txFrame.data[6] = static_cast<uint8_t>(status); // Status byte
  txFrame.data[7] = static_cast<uint8_t>(count); // Count byte

  bool queuedFrame = ESP32Can.writeFrame(txFrame);

  if (queuedFrame){
    count++; // count byte 0 - 255, then wraps around to 0
  }
  else{
    Serial.println("CAN frame failed to send");
  }
}

int8_t encodeAcceleration(float acceleration_g) {
  float scaled = roundf(acceleration_g * 8.0f); // Scale factor of 8 for 1/8th g resolution
  
  if (scaled > 127.0f){
    scaled = 127.0f;
  }
  if (scaled < -128.0f){
    scaled = -128.0f;
  }

  return static_cast<int8_t>(scaled);
}

int8_t encodeGyro(float gyro_dps) {
  float scaled = roundf(gyro_dps / 4.0f); // 1 count = 4 degrees per second, so divide by 4 to scale
  
  if (scaled > 127.0f){
    scaled = 127.0f;
  }
  if (scaled < -128.0f){
    scaled = -128.0f;
  }

  return static_cast<int8_t>(scaled);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  if (!gyro.begin_I2C()) {
    Serial.println("No Gyro sensor detected.");
    while (1);
  } 
  else {
    gyro.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    gyro.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    gyro.setAccelDataRate(LSM6DS_RATE_208_HZ);
    gyro.setGyroDataRate(LSM6DS_RATE_208_HZ);
    Serial.println("Adafruit LSM6DSOX Initialized");
  }

  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setSpeed(TWAI_SPEED_500KBPS);
  ESP32Can.setRxQueueSize(10); // Set the receive queue size to 10
  ESP32Can.setTxQueueSize(10); // Set the transmit queue size to 10

  if (!ESP32Can.begin()) {
    Serial.println("Error: CAN failed to start.");
    while(1);
  }

}

void loop() {
  static uint32_t lastSendTime = 0;
  uint32_t currentTime = millis();

  sensors_event_t accelEvent;
  sensors_event_t gyroEvent;
  sensors_event_t tempEvent;
  gyro.getEvent(&accelEvent, &gyroEvent, &tempEvent);

  float x_g = (accelEvent.acceleration.x / 9.8); // Convert m/s^2 to g
  float y_g = (accelEvent.acceleration.y / 9.8);
  float z_g = (accelEvent.acceleration.z / 9.8);
  float x_radps = gyroEvent.gyro.x; // Gyroscope data in radians per second
  float y_radps = gyroEvent.gyro.y;
  float z_radps = gyroEvent.gyro.z;

  float x_dps = x_radps * (180.0 / PI); // Convert rad/s to degrees per second
  float y_dps = y_radps * (180.0 / PI);
  float z_dps = z_radps * (180.0 / PI);

  int8_t x_dps_encoded = encodeGyro(x_dps);
  int8_t y_dps_encoded = encodeGyro(y_dps);
  int8_t z_dps_encoded = encodeGyro(z_dps);

  int8_t ax_encoded = encodeAcceleration(x_g);
  int8_t ay_encoded = encodeAcceleration(y_g);
  int8_t az_encoded = encodeAcceleration(z_g);

  if (currentTime - lastSendTime >= 1000) { // Send every 1 second
    sendMessage(ax_encoded, ay_encoded, az_encoded, x_dps_encoded, y_dps_encoded, z_dps_encoded, 0x05);
    lastSendTime = currentTime;
  }

}
