/*
 * If you encounter any issues:
 * - check the readme.md at https://github.com/sinricpro/esp8266-esp32-sdk/blob/master/README.md
 * - ensure all dependent libraries are installed
 *   - see https://github.com/sinricpro/esp8266-esp32-sdk/blob/master/README.md#arduinoide
 *   - see https://github.com/sinricpro/esp8266-esp32-sdk/blob/master/README.md#dependencies
 * - open serial monitor and check whats happening
 * - check full user documentation at https://sinricpro.github.io/esp8266-esp32-sdk
 * - visit https://github.com/sinricpro/esp8266-esp32-sdk/issues and check for existing issues or open a new one
 */

// Uncomment the following line to enable serial debug output
//#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
       #define DEBUG_ESP_PORT Serial
       #define NODEBUG_WEBSOCKETS
       #define NDEBUG
#endif 

#include <Arduino.h>
#ifdef ESP8266 
       #include <ESP8266WiFi.h>
#endif 
#ifdef ESP32   v
       #include <WiFi.h>
#endif

#include "AsyncTCP.h"
#include "ESPAsyncWebSrv.h"
#include "MyAsyncElegantOTA.h"

#include "SinricPro.h"
#include "SinricProSwitch.h"

#include <map>

#define WIFI_SSID         "del Matrix IoT 2.4 GHz"    
#define WIFI_PASS         "strongcheese127"
#define APP_KEY           "efdbabb0-5e04-42b6-b785-271aecca401c"      // Should look like "de0bxxxx-1x3x-4x3x-ax2x-5dabxxxxxxxx"
#define APP_SECRET        "45814603-66ff-4ce1-a5af-beebda233e32-82b6033d-5dcb-478c-a740-0318a2ca224c"   // Should look like "5f36xxxx-x3x7-4x3x-xexe-e86724a9xxxx-4c4axxxx-3x3x-x5xe-x9x3-333d65xxxxxx"
#define SWITCH_ID         "641d39cd0f750d41bd147fd1"    // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define BAUD_RATE         9600                // Change baudrate to your need

#define DEBOUNCE_TIME 500

#ifdef ESP32
#define LED_BUILTIN 33

#define RELAYPIN_1 14
#define SWITCHPIN_1 13
#endif


typedef struct {
  int relayPIN;
  int flipSwitchPIN;
} deviceConfig_t;

std::map<String, deviceConfig_t> devices = {
  //{deviceId, {relayPIN,  flipSwitchPIN}}
  {"641d39cd0f750d41bd147fd1", {  RELAYPIN_1, SWITCHPIN_1}}
};

typedef struct {
  String deviceId;
  bool lastFlipSwitchState;
  unsigned long lastFlipSwitchChange;
} flipSwitchConfig_t;

std::map<int, flipSwitchConfig_t> flipSwitches;

void setupRelays() {
  for (auto &device : devices) {           // for each device (relay, flipSwitch combination)
    int relayPIN = device.second.relayPIN; // get the relay pin
    pinMode(relayPIN, OUTPUT);             // set relay pin to OUTPUT
  }
}

void setupFlipSwitches() {
  for (auto &device : devices)  {                     // for each device (relay / flipSwitch combination)
    flipSwitchConfig_t flipSwitchConfig;              // create a new flipSwitch configuration

    flipSwitchConfig.deviceId = device.first;         // set the deviceId
    flipSwitchConfig.lastFlipSwitchChange = 0;        // set debounce time
    flipSwitchConfig.lastFlipSwitchState = false;     // set lastFlipSwitchState to false (LOW)

    int flipSwitchPIN = device.second.flipSwitchPIN;  // get the flipSwitchPIN

    flipSwitches[flipSwitchPIN] = flipSwitchConfig;   // save the flipSwitch config to flipSwitches map
    pinMode(flipSwitchPIN, INPUT);                   // set the flipSwitch pin to INPUT
  }
}

bool onPowerState(String deviceId, bool &state)
{
  Serial.printf("%s: %s\r\n", deviceId.c_str(), state ? "on" : "off");
  int relayPIN = devices[deviceId].relayPIN; // get the relay pin for corresponding device
  digitalWrite(relayPIN, state);             // set the new relay state
  return true;
}

void handleFlipSwitches() {
  unsigned long actualMillis = millis();                                          // get actual millis
  for (auto &flipSwitch : flipSwitches) {                                         // for each flipSwitch in flipSwitches map
    unsigned long lastFlipSwitchChange = flipSwitch.second.lastFlipSwitchChange;  // get the timestamp when flipSwitch was pressed last time (used to debounce / limit events)

    if (actualMillis - lastFlipSwitchChange > DEBOUNCE_TIME) {                    // if time is > debounce time...

      int flipSwitchPIN = flipSwitch.first;                                       // get the flipSwitch pin from configuration
      bool lastFlipSwitchState = flipSwitch.second.lastFlipSwitchState;           // get the lastFlipSwitchState
      bool flipSwitchState = digitalRead(flipSwitchPIN);                          // read the current flipSwitch state
      if (flipSwitchState != lastFlipSwitchState) {                               // if the flipSwitchState has changed...
#ifdef TACTILE_BUTTON
        if (flipSwitchState) {                                                    // if the tactile button is pressed 
#endif      
          flipSwitch.second.lastFlipSwitchChange = actualMillis;                  // update lastFlipSwitchChange time
          String deviceId = flipSwitch.second.deviceId;                           // get the deviceId from config
          int relayPIN = devices[deviceId].relayPIN;                              // get the relayPIN from config
          bool newRelayState = !digitalRead(relayPIN);                            // set the new relay State
          digitalWrite(relayPIN, newRelayState);                                  // set the trelay to the new state

          SinricProSwitch &mySwitch = SinricPro[deviceId];                        // get Switch device from SinricPro
          mySwitch.sendPowerStateEvent(newRelayState);                            // send the event
#ifdef TACTILE_BUTTON
        }
#endif      
        flipSwitch.second.lastFlipSwitchState = flipSwitchState;                  // update lastFlipSwitchState
      }
    }
  }
}

void setupWiFi()
{
  Serial.printf("\r\n[Wifi]: Connecting");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

 pinMode(LED_BUILTIN, OUTPUT);
 digitalWrite(LED_BUILTIN, LOW);

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.printf(".");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
  }
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.printf("connected!\r\n[WiFi]: IP-Address is %s\r\n", WiFi.localIP().toString().c_str());
}

void setupSinricPro()
{
  for (auto &device : devices)
  {
    const char *deviceId = device.first.c_str();
    SinricProSwitch &mySwitch = SinricPro[deviceId];
    mySwitch.onPowerState(onPowerState);
  }

  //SinricPro.restoreDeviceStates(true); // Uncomment to restore the last known state from the server.
  SinricPro.begin(APP_KEY, APP_SECRET);  
  Serial.printpln("loop");
}

AsyncWebServer server(80);

void setup()
{
  Serial.begin(BAUD_RATE);
  setupRelays();
  setupFlipSwitches();
  setupWiFi();
  setupSinricPro();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am ESP32.");
  });

  MyAsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");

}

void loop()
{
  Serial.println("loop");
  SinricPro.handle();
  Serial.println("SinricPro");
  handleFlipSwitches();
  Serial.println("HandleFlipSwitches");
}