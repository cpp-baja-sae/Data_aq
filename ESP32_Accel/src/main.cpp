

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

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

const uint8_t status = 0x05; // Example status byte 00000101 represents GOOD!

void sendMessage(int8_t ax_g, int8_t ay_g, int8_t az_g, uint8_t status){
  static uint8_t count = 0;
  CanFrame txFrame = {0};
  txFrame.identifier = 0x101; // Matches the accelerometer data message identifier
  txFrame.extd = 0; // Indicates 11 bit id ^
  txFrame.data_length_code = 5; // 5 Bytes
  txFrame.data[0] = static_cast<uint8_t>(ax_g); 
  txFrame.data[1] = static_cast<uint8_t>(ay_g); 
  txFrame.data[2] = static_cast<uint8_t>(az_g);
  txFrame.data[3] = static_cast<uint8_t>(status); // Status byte
  txFrame.data[4] = static_cast<uint8_t>(count); // Count byte

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
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  if (!accel.begin()) {
    Serial.println("Error: ADXL345 not detected.");
    while(1);
  }
  else{
    Serial.println("ADXL345 detected.");
    accel.setDataRate(ADXL345_DATARATE_200_HZ);
    accel.setRange(ADXL345_RANGE_16_G);
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

  sensors_event_t event;
  accel.getEvent(&event);

  float x_g = (event.acceleration.x / 9.8); // Convert m/s^2 to g
  float y_g = (event.acceleration.y / 9.8);
  float z_g = (event.acceleration.z / 9.8);

  int8_t ax_encoded = encodeAcceleration(x_g);
  int8_t ay_encoded = encodeAcceleration(y_g);
  int8_t az_encoded = encodeAcceleration(z_g);

  if (currentTime - lastSendTime >= 1000) { // Send every 1 second
    sendMessage(ax_encoded, ay_encoded, az_encoded, 0x05);
    lastSendTime = currentTime;
  }

}
