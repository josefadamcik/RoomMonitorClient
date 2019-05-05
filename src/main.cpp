#include "keys.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <ESP8266HTTPClient.h>
// #include <Adafruit_MQTT.h>
// #include <Adafruit_MQTT_Client.h>
#include <U8x8lib.h>
#include <AsyncMqttClient.h>
#include <Ticker.h>

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

IPAddress ip(192, 168, 178, 33);
IPAddress gateway(192, 168, 178, 1);
IPAddress subnet(255, 255, 255, 0);

U8X8_SSD1306_128X32_UNIVISION_SW_I2C display(5,4);

// WiFiClient clientn;
// WiFiClientSecure client;
// Adafruit_MQTT_Client mqtt(&clientn, aioServer, aioServerport, aioUsername, aioKey);
// Adafruit_MQTT_Subscribe mqttTempFeed(&mqtt, tempfeed, MQTT_QOS_1);
// Adafruit_MQTT_Publish mqttVccRawFeed(&mqtt, vccrawfeed, MQTT_QOS_1);

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

void displayMessage(const char* s) {
  display.clearDisplay();
  display.setFont(u8x8_font_chroma48medium8_r);
  display.drawString(0, 0, s);
}

void displayUpdate(const char* s) {
  display.setFont(u8x8_font_chroma48medium8_r);
  for (uint8_t row = 8; row < 16; row++) {
    display.clearLine(row);
  }
  display.drawString(0, 1, s);
}

void displayProgress(bool reset) {
  static uint8_t step;
  if (reset || step == 4) {
    step = 0;
  }
  if (step == 0) {
    displayUpdate("=");
  } else if (step == 1) {
    displayUpdate("==");
  } else if (step == 2) {
    displayUpdate("===");
  } else if (step == 3) {
    displayUpdate("====");
  }
  step++;
}

// bool verifyFingerprint() {
//   displayMessage("Verifying...");
//   Serial.println(aioServer);
//   if (! client.connect(aioServer, aioServerport)) {
//     Serial.println(F("Connection failed."));
//     displayMessage("CON fail.");
//     return false;
//   }
//   if (client.verify(aioSslFingreprint, aioServer)) {
//     Serial.println(F("Connection secure."));
//     return true;
//   } else {
//     Serial.println(F("Connection insecure!"));
//     displayMessage("CON insecure.");
//     return false;
//   }
//   client.stop(); //otherwise the MQTT.connected() will return true, because the implementation
//   //just asks the client if there is a connectioni. It actully doesn't check if there was a mqtt connection established.
// }

boolean wifiConect() {
    displayProgress(true);
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
        if (retries % 4 == 0) {
            displayProgress(false);
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
    ArduinoOTA.setHostname("192.168.178.33");
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
        char s[5]; 
        sprintf(s,"%d %%", (progress / (total / 100)));
        displayUpdate(s);
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
  char s[2];
  while (keeptrying-- > 0 && !waiforOTA) {
      sprintf(s, "%d", keeptrying);
      displayUpdate(s);
      if (digitalRead(0) == LOW) {
        return true;
      }
      Serial.print(".");
      delay(500);
  } 
  return false;
}

bool connectMQTT() {
  displayMessage("MQTT...");
  Serial.println("Connecting to MQTT");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  displayMessage("MQTT OK");
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe(tempfeed, 1);
  Serial.print("Subscribing at QoS 1, packetId: ");
  Serial.println(packetIdSub);
  uint16_t packetIdPub1 = mqttClient.publish(vccrawfeed, 1, true, "898");
  Serial.print("Publishing at QoS 1, packetId: ");
  Serial.println(packetIdPub1);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
   displayMessage("MQTT OFF");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectMQTT);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);

  char subbuff[len+1];
  strncpy(subbuff, payload, len);
  subbuff[len] = '\0';
  Serial.println(" payload: ");
  Serial.println(subbuff);
  display.clearDisplay();
  display.drawString(0,0, topic);
  display.drawString(0,1, subbuff);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
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
  // if (clientn.connect("192.168.178.29", 4000)) {
  //   Serial.println("Local OK");
  // } else {
  //   Serial.println("Local NOK");
  // }
  // clientn.stop();

  // if (clientn.connect("josef-adamcik.cz", 443)) {
  //   Serial.println("JA OK");
  // } else {
  //   Serial.println("JA NOK");
  // }
  // clientn.stop();
  // if (clientn.connect(aioServer, 1883)) {
  //   Serial.println("adafruit nonsecure OK");
  // } else {
  //   Serial.println("adafruit nonsecure NOK");
  // }
  // clientn.stop();

  // if (client.connect(aioServer, aioServerport)) {
  //   Serial.println("adafruit ok");
  // } else {
  //   Serial.println("adafruit nok");
  // }
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(aioServer, aioServerport);
  mqttClient.setCredentials(aioUsername, aioKey);
  connectMQTT();
}

void loop(void)
{
  
}