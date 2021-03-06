# RoomMonitorClient

Software/firmware for a ESP8266 OLED MQTT client for DYI [room conditions monitor](https://github.com/josefadamcik/RoomMonitor).

![Freeform ESP8266 OLED MQTT client](img/esp8266oledmqttclient.jpg)

PlatformIO is used to build this project but it should be easy to convert it back to a project for Arduino IDE.

More information can be found in this blog post:

- [Freeform ESP8266 OLED MQTT client](https://josef-adamcik.cz/electronics/freeform-esp8266-based-mqtt-oled-client.html)


## Programming

Buttons on the above image: 

- RST is in the back
- PRG is on the side

### Frist time

- Use breakout at the back (TODO: add pinout)
- hold PRG and press RST to enter programming mode
- program and reset

### OTA

- make sure platformio.ini is properly setup
- RST device
- Displya wil show `OTA?` and a count-down. Press PRG while counting down, "Waiting for OTA" should be displayed. 
- Use platformio to upload the firmware.



