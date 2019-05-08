#include "keys.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <ESP8266HTTPClient.h>
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
const char photovfeed[] = AIO_USERNAME "/feeds/room-monitor.light";
const char pressurefeed[] = AIO_USERNAME "/feeds/room-monitor.pressure";
const char aioSslFingreprint[] = "77 00 54 2D DA E7 D8 03 27 31 23 99 EB 27 DB CB A5 4C 57 18";

IPAddress ip(192, 168, 178, 33);
IPAddress gateway(192, 168, 178, 1);
IPAddress subnet(255, 255, 255, 0);

U8X8_SSD1306_128X32_UNIVISION_SW_I2C display(5,4);

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

const uint8_t ledPin = 13;
bool mqttStarted = false;
bool mqttConnected = false;
unsigned long lastTopLineRefresh = 0;
unsigned long lastTouchDetected = 0;
unsigned long lastTouchChecked = 0;
const unsigned long refreshTopLineEach = 1000;
const unsigned long touchDetectInterval = 1000;
char topLineBuffer[17];
int analogTouchValue = 0;
bool monitorVoltageAlarmOn = false;
const int analogTouchThreshold = 900;
double monitorVoltageAlarmTreshold = 3.0; 
bool additionalInfoActivated = false;
int ledPulseValue = 0;
int ledPulseStep = 128;

struct MqttData {
  double temperature;
  double vcc;
  double humidity;
  double light;
  double pressure;
  unsigned long lastUpdate;
} mqttData;


// WiFiEventHandler wifiConnectHandler;
// WiFiEventHandler wifiDisconnectHandler;
// Ticker wifiReconnectTimer;

void displayMessage(const char* s) {
  display.clearDisplay();
  display.setFont(u8x8_font_chroma48medium8_r);
  display.drawString(0, 0, s);
}

void displayUpdate(const char* s) {
  display.setFont(u8x8_font_chroma48medium8_r);
  display.clearLine(1);
  display.drawString(0, 1, s);
}

void displayTemperature(const char* s) {
  display.clearDisplay();
  display.setFont(u8x8_font_courB18_2x3_f);
  display.drawString(1,1, s);
  display.drawGlyph(11,1, '°');
  display.drawString(13,1, "C");
}

void displayOnTopRow(const char* s) {
   display.clearLine(0);
   display.setFont(u8x8_font_chroma48medium8_r);
   display.drawString(0, 0, s);
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

void connectMQTT() {
  displayMessage("MQTT...");
  Serial.println("Connecting to MQTT");
  mqttStarted = true;
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  mqttConnected = true;
  displayMessage("MQTT OK");

  Serial.println("Connected to MQTT."); Serial.print("Session present: "); Serial.println(sessionPresent);

  uint16_t packetIdSub = mqttClient.subscribe(tempfeed, 1);
  Serial.println(packetIdSub);
  packetIdSub = mqttClient.subscribe(vccfeed, 1);
  Serial.println(packetIdSub);
  packetIdSub = mqttClient.subscribe(humfeed, 1);
  Serial.println(packetIdSub);
  packetIdSub = mqttClient.subscribe(photovfeed, 1);
  Serial.println(packetIdSub);
  packetIdSub = mqttClient.subscribe(pressurefeed, 1);
  Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  Serial.print("Reason: ");
  Serial.print((uint8_t)reason);
  Serial.print(" wifi state: ");
  Serial.println(WiFi.status());
  displayMessage("MQTT OFF");
  mqttConnected = false;
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

void printMessageDebugInfo(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: "); Serial.println(topic);
  Serial.print("  qos: "); Serial.println(properties.qos);
  Serial.print("  dup: "); Serial.println(properties.dup);
  Serial.print("  retain: "); Serial.println(properties.retain);
  Serial.print("  len: "); Serial.println(len);
  Serial.print("  index: "); Serial.println(index);
  Serial.print("  total: ");  Serial.println(total);
  Serial.print(" payload: "); Serial.println(payload);
}

void processNewTemperatureValue(char* temperatureStr) {
    displayTemperature(temperatureStr);
    mqttData.temperature = atof(temperatureStr);
    mqttData.lastUpdate = millis(); 
}

void processNewVccValue(char* str) {
    mqttData.vcc = atof(str);
    mqttData.lastUpdate = millis(); 
    if (mqttData.vcc > 0.0 && mqttData.vcc <= monitorVoltageAlarmTreshold) {
      monitorVoltageAlarmOn = true;
    } else {
      monitorVoltageAlarmOn = false;
    }
}

void processNewHumidityValue(char* str) {
    mqttData.humidity = atof(str);
    mqttData.lastUpdate = millis(); 
}

void processNewPressureValue(char* str) {
    mqttData.pressure = atof(str);
    mqttData.lastUpdate = millis(); 
}

void processNewLightValue(char* str) {
    mqttData.light = atof(str);
    mqttData.lastUpdate = millis(); 
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  //fix the payload string
  char payloadStr[len+1];
  strncpy(payloadStr, payload, len);
  payloadStr[len] = '\0';

  printMessageDebugInfo(topic, payloadStr, properties, len, index, total);
  
  if (strcmp(topic, tempfeed) == 0) {
      processNewTemperatureValue(payloadStr);
  } else if (strcmp(topic, humfeed) == 0) {
      processNewHumidityValue(payloadStr);
  } else if (strcmp(topic, vccfeed) == 0) {
      processNewVccValue(payloadStr);
  } else if (strcmp(topic, photovfeed) == 0) {
      processNewLightValue(payloadStr);
  } else if (strcmp(topic, pressurefeed) == 0) {
      processNewPressureValue(payloadStr);
  }
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
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

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

  //start the mqtt
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(aioServer, aioServerport);
  mqttClient.setCredentials(aioUsername, aioKey);
  connectMQTT();
  // topLineBuffer[0] = '\0';
}



void loop(void)
{
  if (mqttStarted) {
    //FIXME: handle this properly, has an aenum for the state idle, connectind, disconnected or whatever
    // if (!WiFi.isConnected()) {
    //   digitalWrite(ledPin, HIGH);
    //   wifiConect();
    // } else if (!mqttConnected) {
    //   digitalWrite(ledPin, HIGH);
    //   mqttReconnectTimer.once(2, connectMQTT);
    // }

    //pulse diode if we need to change the battery for RoomMonitor
    if (monitorVoltageAlarmOn) {
      ledPulseValue += ledPulseStep;
      if (ledPulseValue >= 1023) {
        ledPulseStep = -ledPulseStep;
        ledPulseValue = 1023;
      } else if (ledPulseValue <= 0) {
        ledPulseValue = 0;
        ledPulseStep = -ledPulseStep;
      }
      analogWrite(ledPin, ledPulseValue);
    } else {
      ledPulseValue = 0;
      ledPulseStep = 16;
      analogWrite(ledPin, 0);
      digitalWrite(ledPin, LOW);
    }

    unsigned long now = millis();

    analogTouchValue = analogRead(A0);
    if (analogTouchValue > analogTouchThreshold) {
      Serial.println("Touch detected");
      lastTouchDetected = now;
    }

    if (mqttData.lastUpdate > 0) {
      if (!additionalInfoActivated && lastTouchDetected + refreshTopLineEach > now) {
        additionalInfoActivated = true;
        char line[17]; 
        /* 4 is mininum width, 2 is precision; float value is copied onto str_temp*/
        char strValue[8];
        dtostrf(mqttData.vcc, 4, 2, strValue);
        sprintf(line, "vcc %s V", strValue);
        displayOnTopRow(line);
      } else if (additionalInfoActivated && lastTouchDetected + refreshTopLineEach < now) {
        additionalInfoActivated = false;
        display.clearLine(0);
      }

      // if (lastTopLineRefresh + refreshTopLineEach < now) {
      //   //TODO: cycle information on the top display row
      //   char line[17]; 
      //   /* 4 is mininum width, 2 is precision; float value is copied onto str_temp*/
      //   char strValue[8];
      //   dtostrf(mqttData.vcc, 4, 2, strValue);
      //   sprintf(line, "vcc %s V", strValue);

      //   if (strcmp(line, topLineBuffer) != 0) {
      //     Serial.println("Refresh top line, data changed");
      //     Serial.println(line);
      //     strcpy(topLineBuffer, line);
      //     displayOnTopRow(topLineBuffer);
      //   }
      //   lastTopLineRefresh = now;
      // }
    }
  }
  delay(50); //leave some time for networking layer to do it's stufff
}