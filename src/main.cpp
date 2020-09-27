#include "keys.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <ESP8266HTTPClient.h>
#include <U8x8lib.h>
#include <Ticker.h>
#include <PubSubClient.h>


const int roomCount = 2;
const char rooms[roomCount][11] = {"bedroom", "livingroom"};
const char feedPrefix[] = "home/";
int selectedRooomIndex = 0;


const char aioServer[] = "192.168.178.58";
const int aioServerport = 1883; 
const char ssid[] = MYSSID; //put #define MYSSID "xyz" in keys.h
const char password[] = MYPASS; //put #define MYPASS "blf" in keys.h
const char tempfeed[] = "/temperature";
const char humfeed[] = "/humidity";
const char vccfeed[] = "/vcc";
const char photovfeed[] = "/light";

IPAddress ip(192, 168, 178, 34);
IPAddress gateway(192, 168, 178, 1);
IPAddress subnet(255, 255, 255, 0);

U8X8_SSD1306_128X32_UNIVISION_SW_I2C display(5,4);

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

const uint8_t ledPin = 13;
unsigned long lastTopLineRefresh = 0;
unsigned long lastTouchDetected = 0;
const unsigned long refreshTopLineEach = 1000;
const unsigned long touchTriggerTimeout = 250;
char topLineBuffer[17];
int analogTouchValue = 0;
bool monitorVoltageAlarmOn = false;
const int analogTouchThreshold = 900;
double monitorVoltageAlarmTreshold = 3.0; 
uint8_t additionalInfoActivated = 0;
int ledPulseValue = 0;
int ledPulseStep = 128;

struct MqttData {
  double temperature;
  double vcc;
  double humidity;
  double light;
  unsigned long lastUpdate;
} mqttData[roomCount];

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

void ensureWifiConnection() {
  if (!WiFi.isConnected()) {
    displayMessage("Connecting...");
    if (wifiConect()) {
      displayMessage("Wifi OK"); 
    } else {
      displayMessage("Wifi NOK");
    }
  }
}

void otaInitialize() {
    ArduinoOTA.setHostname("192.168.178.34");
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

void subscribe() {
  displayMessage("MQTT OK");

  Serial.println("Connected to MQTT."); 
  for (uint8_t roomIndex = 0; roomIndex < roomCount; roomIndex++) {
    String feedRoomPrefix = feedPrefix + String(rooms[roomIndex]);
    Serial.print("Room: "); Serial.println(feedRoomPrefix);
    Serial.println((feedRoomPrefix + tempfeed).c_str());
    mqttClient.subscribe((feedRoomPrefix + tempfeed).c_str(), 1);
    mqttClient.subscribe((feedRoomPrefix + vccfeed).c_str(), 1);
    mqttClient.subscribe((feedRoomPrefix + humfeed).c_str(), 1);
    mqttClient.subscribe((feedRoomPrefix + photovfeed).c_str(), 1);
  }
}

void connectMQTT() {
  while (!mqttClient.connected()) {
    displayMessage("MQTT...");
    Serial.print("Attempting MQTT connection...");
    String clientId = "RoomMointorClient";
    // clientId += String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");
      mqttClient.publish("/home/client/ping", "hi");
      subscribe();
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in few seconds");
      delay(2000);
    }
  }
}

void processNewTemperatureValue(int index, char* temperatureStr) {
   if (index == selectedRooomIndex) {
      displayTemperature(temperatureStr);
   }
   mqttData[index].temperature = atof(temperatureStr);
   mqttData[index].lastUpdate = millis(); 
}

void processNewVccValue(int index, char* str) {
    mqttData[index].vcc = atof(str);
    mqttData[index].lastUpdate = millis(); 
    if (mqttData[index].vcc > 0.0 && mqttData[index].vcc <= monitorVoltageAlarmTreshold) {
      monitorVoltageAlarmOn = true;
    } else {
      monitorVoltageAlarmOn = false;
    }
}

void processNewHumidityValue(int index, char* str) {
    mqttData[index].humidity = atof(str);
    mqttData[index].lastUpdate = millis(); 
}

void processNewLightValue(int index, char* str) {
    mqttData[index].light = atof(str);
    mqttData[index].lastUpdate = millis(); 
}

void onMqttMessage(char* topic, byte* payload, unsigned int len) {
  //fix the payload string
  char payloadStr[len+1];
  strncpy(payloadStr, (char *)payload, len);
  payloadStr[len] = '\0';
  
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(payloadStr);
  Serial.println();

  int foundRoomIndex = -1;
  for (int roomIndex = 0; roomIndex < roomCount; roomIndex++) {
    if (strstr(topic, rooms[roomIndex]) != NULL) {
      foundRoomIndex = roomIndex;
      break;
    }
  }   
  if (foundRoomIndex != -1)  {
    if (strstr(topic, tempfeed) != NULL) {
        processNewTemperatureValue(foundRoomIndex, payloadStr);
    } else if (strstr(topic, humfeed) != NULL) {
        processNewHumidityValue(foundRoomIndex, payloadStr);
    } else if (strstr(topic, vccfeed) != NULL) {
        processNewVccValue(foundRoomIndex, payloadStr);
    } else if (strstr(topic, photovfeed) != NULL) {
        processNewLightValue(foundRoomIndex, payloadStr);
    } 
  }
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
  ensureWifiConnection();

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
  mqttClient.setServer(aioServer, aioServerport);
  mqttClient.setCallback(onMqttMessage);
  connectMQTT();
}

void loop(void)
{
  if (!mqttClient.connected()){
    digitalWrite(ledPin, HIGH);

    if (!WiFi.isConnected()) {
      displayMessage("WIFI OFF");
      WiFi.mode(WIFI_OFF);
      delay(10);
      WiFi.mode(WIFI_STA);
      delay(10);
      ensureWifiConnection();
      delay(100);
    } 
    
    if (WiFi.isConnected()) {
      displayMessage("MQTT retry");
      delay(100);
      connectMQTT();
      delay(100);
    }
  } else if (mqttClient.connected()) {
    mqttClient.loop();

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
    boolean touchTriggered = false;
    if (analogTouchValue > analogTouchThreshold) {
      if (lastTouchDetected + touchTriggerTimeout < now) {
        Serial.println("Touch detected");
        lastTouchDetected = now;
        touchTriggered = true;
      }
    }

    if (mqttData[selectedRooomIndex].lastUpdate > 0) {
      if (!additionalInfoActivated && touchTriggered) {
        additionalInfoActivated++;
        char line[17]; 
        /* 4 is mininum width, 2 is precision; float value is copied onto str_temp*/
        char strValue[8];
        dtostrf(mqttData[selectedRooomIndex].vcc, 4, 2, strValue);
        sprintf(line, "vcc %s V", strValue);
        displayOnTopRow(line);
      } else if (additionalInfoActivated == 1 && touchTriggered) {
        additionalInfoActivated++;
        displayOnTopRow(rooms[selectedRooomIndex]);
      } else if (additionalInfoActivated == 2 && touchTriggered) {
        additionalInfoActivated = 1;
        //swtich room
        selectedRooomIndex++;
        if (selectedRooomIndex >= roomCount) {
          selectedRooomIndex = 0;
        }
        //display temp in anothre room
        char strValue[5];
        dtostrf(mqttData[selectedRooomIndex].temperature, 5,2, strValue);
        displayTemperature(strValue);
        displayOnTopRow(rooms[selectedRooomIndex]);
      } else if (additionalInfoActivated > 0 && lastTouchDetected + refreshTopLineEach < now) {
        additionalInfoActivated = 0;
        display.clearLine(0);
      }
    }
  }

  delay(50); //leave some time for networking layer to do it's stufff
}