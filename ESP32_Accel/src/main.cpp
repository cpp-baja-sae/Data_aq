

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


#define CAN_TX 2
#define CAN_RX 3

constexpr uint16_t AnalogRes = 4095; // 12-bit ADC resolution
const int Vref = 3300; // Reference voltage in millivolts
const int potPin = A0; // Analog pin for the potentiometer
const uint16_t status = 0x05; // Example status byte 00000101 represents GOOD!

void sendMessage(uint16_t voltage_mV, uint8_t status){
  static uint8_t count = 0;
  CanFrame txFrame = {0};
  txFrame.identifier = 0x100; // Matches the potentiometer data message identifier
  txFrame.extd = 0; // Indicates 11 bit id ^
  txFrame.data_length_code = 4; // 4 Bytes
  txFrame.data[0] = static_cast<uint8_t>(voltage_mV & 0xFF); // 0xFF = 11111111, voltage_mV is like 01010101, 
  txFrame.data[1] = static_cast<uint8_t>((voltage_mV >> 8) & 0xFF); // Shift right by 8 bits to get the high byte
  txFrame.data[2] = status; // Status byte
  txFrame.data[3] = count;

  bool queuedFrame = ESP32Can.writeFrame(txFrame);

  if (queuedFrame){
    count++; // count byte 0 - 255, then wraps around to 0
  }
  else{
    Serial.println("CAN frame failed to send");
  }
}
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setSpeed(TWAI_SPEED_500KBPS);
  ESP32Can.setRxQueueSize(10); // Set the receive queue size to 10
  ESP32Can.setTxQueueSize(10); // Set the transmit queue size to 10

  ESP32Can.begin();

}

void loop() {
  static uint32_t lastSendTime = 0;
  uint32_t currentTime = millis();

  if (currentTime - lastSendTime >= 1000) { // Send every 1 second
    uint16_t potValue = analogRead(potPin);
    uint16_t voltage_mV = (potValue * Vref) / AnalogRes;
    sendMessage(voltage_mV, 0x05);
    lastSendTime = currentTime;
  }

}
