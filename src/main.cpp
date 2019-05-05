#include "keys.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <ESP8266HTTPClient.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <U8x8lib.h>

const char aioServer[] = "io.adafruit.com";
const int aioServerport = 1883; 
const int aioServerportSecure = 8883; 
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

// IPAddress ip(192, 168, 178, 33);
// IPAddress gateway(192, 168, 178, 1);
// IPAddress subnet(255, 255, 255, 0);

U8X8_SSD1306_128X32_UNIVISION_SW_I2C display(5,4);

WiFiClient clientn;
WiFiClientSecure client;
Adafruit_MQTT_Client mqtt(&clientn, aioServer, aioServerport, aioUsername, aioKey);
Adafruit_MQTT_Subscribe mqttTempFeed(&mqtt, tempfeed, MQTT_QOS_1);

void displayMessage(const char* s) {
  display.clearDisplay();
  display.setFont(u8x8_font_chroma48medium8_r);
  display.drawString(0, 0, s);
}

// void displayUpdate(const char* s) {
  // display.clearLine()
// }

bool verifyFingerprint() {
  displayMessage("Verifying...");
  Serial.println(aioServer);
  if (! client.connect(aioServer, aioServerport)) {
    Serial.println(F("Connection failed."));
    displayMessage("CON fail.");
    return false;
  }
  if (client.verify(aioSslFingreprint, aioServer)) {
    Serial.println(F("Connection secure."));
    return true;
  } else {
    Serial.println(F("Connection insecure!"));
    displayMessage("CON insecure.");
    return false;
  }
  client.stop(); //otherwise the MQTT.connected() will return true, because the implementation
  //just asks the client if there is a connectioni. It actully doesn't check if there was a mqtt connection established.
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
    Serial.println("Mac address:"); Serial.println(WiFi.macAddress());
    Serial.println("IP address: "); Serial.println(WiFi.localIP());
    Serial.println(WiFi.status());
    Serial.println(F("Setup done"));
    return true;
}
void otaInitialize() {
    ArduinoOTA.setHostname("roommonitorclient.local");
    ArduinoOTA.onStart([]() { 
      Serial.println("OTA Start"); 
      displayMessage("OTA Start");
    });
    ArduinoOTA.onEnd([]() { 
      Serial.println("OTA End"); 
      displayMessage("OTA End");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
            Serial.println("Auth Failed");
            displayMessage("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
            Serial.println("Begin Failed");
            displayMessage("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
            Serial.println("Connect Failed");
            displayMessage("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
            Serial.println("Receive Failed");
            displayMessage("Receive Failed");
        } else if (error == OTA_END_ERROR) {
            Serial.println("End Failed");
            displayMessage("End Failed");
        }
    });
    ArduinoOTA.begin();
}

boolean shouldWaitForOTA() {
  pinMode(0, INPUT);
  bool waiforOTA = false;
  int keeptrying = 6;
  while (keeptrying-- > 0 && !waiforOTA) {
      if (digitalRead(0) == LOW) {
        return true;
      }
      Serial.print(".");
      delay(500);
  } 
  return false;
}

bool connectMQTT() {
  if (mqtt.connected()) {
    return true;
  }
  displayMessage("MQTT...");
  Serial.println("Connecting to MQTT");
  Serial.print(aioServer); Serial.print(" "); Serial.println(aioServerport);
  uint8_t retries = 3;
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
      Serial.println(mqtt.connectErrorString(ret));
      Serial.println(F("Retr MQTT connection in 1 second..."));
      mqtt.disconnect();
      delay(1000); 
      retries--;
      if (retries == 0) {
          return false; 
      }
  }

  if (mqtt.connected()) {
      Serial.println(F(" MQTT Connected!"));
      return true;
  } else {
      Serial.print(F(" MQTT still NOT onnected! "));
      Serial.println(ret);
      return false;
  }
}

void setup(void)
{
  Serial.begin(115200);
  display.begin();
  display.setPowerSave(0);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  // WiFi.config(ip, gateway, subnet, gateway, gateway);
  displayMessage("Connecting...");
  if (wifiConect()) {
    displayMessage("Wifi OK"); 
  } else {
    displayMessage("Wifi NOK");
    while(true) {};
  }

  //Give some time to the user to press the programming button in order to indicate we will be doing OTA update.
  displayMessage("OTA?");
  bool waitforOTA = shouldWaitForOTA();
  if (waitforOTA) {
    Serial.println("Waiting for OTA");
    displayMessage("Waiting for OTA");
    otaInitialize();
    while(1) {
        Serial.print(".");
        ArduinoOTA.handle();
        delay(500);
    }
  } else {
    displayMessage("No OTA. ");
  }
  //verify we are able to connect securely
  // if (!verifyFingerprint()) {
    // return;
  // }
  // if (clientn.connect(IPAddress(192, 168, 178, 29), 4000)) {
  if (clientn.connect("192.168.178.29", 4000)) {
    Serial.println("Local OK");
  } else {
    Serial.println("Local NOK");
  }
  clientn.stop();

  if (clientn.connect("josef-adamcik.cz", 443)) {
    Serial.println("JA OK");
  } else {
    Serial.println("JA NOK");
  }
  clientn.stop();
  if (clientn.connect(aioServer, 1883)) {
    Serial.println("adafruit nonsecure OK");
  } else {
    Serial.println("adafruit nonsecure NOK");
  }
  clientn.stop();

  if (client.connect(aioServer, aioServerport)) {
    Serial.println("adafruit ok");
  } else {
    Serial.println("adafruit nok");
  }
  if (!connectMQTT()) {
    displayMessage("MQTT fail");
    return;
  }
}

void loop(void)
{
  if (!connectMQTT()) {
    displayMessage("MQTT fail");
    delay(5000);
    return;
  }

}