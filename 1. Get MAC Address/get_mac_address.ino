/*
  Project:   Tesla-ESP-CAN
  Module:    ESP32 MAC Address Retrieval
  Company:   Honeer Automotive
  Date:      October 4, 2025
  Author:    Honeer Automotive Development Team

  Description:
  This sketch retrieves and displays the unique MAC address of an ESP32 module.
  It is part of the Tesla-ESP-CAN project, which provides a CAN Bus interface 
  using ESP controllers for Tesla vehicles. The MAC address can be used for 
  device identification, networking, and secure communication within the project.

  Hardware:  ESP32 Development Board
  IDE:       Arduino IDE 2.3.6
  Libraries: Built-in WiFi library (installed with ESP32 board support package)
  Board:     ESP32 Dev Module (or compatible ESP32 board)
  Version:   v1.0
*/

#include "WiFi.h"

void setup() {
  Serial.begin(115200);

  // Set WiFi mode to Station (client) only
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin();

  // Print the ESP32 MAC Address
  Serial.println(WiFi.macAddress());
}

void loop() {
  // Nothing to do here
}
