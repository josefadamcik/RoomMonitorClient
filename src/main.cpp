#include "keys.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <ESP8266HTTPClient.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <U8x8lib.h>

const char aioServer[] = "io.adafruit.com";
const int aioServerport = 8883; 
const char ssid[] = MYSSID; //put #define MYSSID "xyz" in keys.h
const char password[] = MYPASS; //put #define MYPASS "blf" in keys.h
const char aioUsername[] = AIO_USERNAME; //put #define AIO_USERNAME "xyz" in keys.h
const char aioKey[] = AIO_KEY; //put #define AIO_KEY "xyz" in keys.h
const char tempfeed[] = AIO_USERNAME "/feeds/room-monitor.temperature";
const char humfeed[] = AIO_USERNAME "/feeds/room-monitor.humidity";
const char vccfeed[] = AIO_USERNAME "/feeds/room-monitor.vcc";
const char vccrawfeed[] = AIO_USERNAME "/feeds/room-monitor.vcc-raw";
const char vccwarning[] = AIO_USERNAME "/feeds/room-monitor.vcc-warning";
const char photovfeed[] = AIO_USERNAME "/feeds/room-monitor.light";
const char pressurefeed[] = AIO_USERNAME "/feeds/room-monitor.pressure";
const char msgWifiConnecting[] PROGMEM = "WIFI connecting to: ";
const char aioSslFingreprint[] = "77 00 54 2D DA E7 D8 03 27 31 23 99 EB 27 DB CB A5 4C 57 18";

// IPAddress ip(192, 168, 178, 27);
// IPAddress gateway(192, 168, 178, 1);
// IPAddress subnet(255, 255, 255, 0);

U8X8_SSD1306_128X32_UNIVISION_SW_I2C display(5,4);

void displayMessage(const char* s) {
  display.clearDisplay();
  display.setFont(u8x8_font_chroma48medium8_r);
  display.drawString(0, 0, s);
}

void verifyFingerprint() {
  displayMessage("Verifying...");
  Serial.println(aioServer);
  // if (! client.connect(aioServer, aioServerPort)) {
    // Serial.println(F("Connection failed."));
    // while(1);
  // }
  // if (client.verify(aioSslFingreprint, aioServer)) {
  //   Serial.println(F("Connection secure."));
  // } else {
  //   Serial.println(F("Connection insecure!"));
  //   while(1);
  // }

  // client.stop(); //otherwise the MQTT.connected() will return true, because the implementation
  // just asks the client if there is a connectioni. It actully doesn't check if there was a mqtt connection established.
}

boolean wifiConect() {

    WiFi.begin(ssid, password);
    int retries = 0;
    int wifiStatus = WiFi.status();
    while (wifiStatus != WL_CONNECTED) {
        retries++;
        if (retries == 600) {
            Serial.println(F("Give up"));
            // Giving up after 30 seconds and going back to sleep
            WiFi.disconnect(true);
            delay(1);
            return false;
        }
        if (retries % 10 == 0) {
            Serial.print(F("."));
        }
        delay(50);
        wifiStatus = WiFi.status();
    }
    Serial.println(WiFi.status());
    Serial.println(F("Setup done"));
    return true;

}

void setup(void)
{
  Serial.begin(115200);
  display.begin();
  display.setPowerSave(0);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  // WiFi.config(wifiSetup.ip, wifiSetup.gateway, wifiSetup.subnet, wifiSetup.gateway, wifiSetup.gateway);
  displayMessage("Connecting...");
  if (wifiConect()) {
    displayMessage("Wifi OK"); 
  } else {
    displayMessage("Wifi NOK");
    while(true) {};
  }

  pinMode(0, INPUT);

}

void loop(void)
{

}