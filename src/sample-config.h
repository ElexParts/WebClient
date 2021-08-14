#include <Arduino.h>

// WiFi
#define WLAN_SSID   "myNetwork"      // Cannot be longer than 32 characters!
#define WLAN_PASS   "myPassword"

// Configure an MQTT server.
union ArrayToIp {
  byte array[4];
  uint32_t ip;
};

// Set MQTT server's ip address.
ArrayToIp server = { 0, 0, 0, 0 };

// MQTT device.
char* device_name = "";
char* device_id = "";
char* topic_state = "";
char* topic_telemetry = "";
