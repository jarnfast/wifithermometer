# Wifi Thermometer

This is a small bit of software designed to run on a ESP8266 (NodeMCU) with an DS18B20 temperature sensor attached.
The program will read the temperature at defined intervals and publish them to a MQTT broker.

Configuration of WiFI connection, MQTT broker and interval is possible using a captive portal.

## Configuration

1. Reset the NodeMCU (press RST)
2. Immediately press and hold FLASH button for 5 seconds
3. When the LED starts blinking a WiFI access point has been enabled
4. Connect your PC or phone to the access point named `WifiThermometer-<id>`
5. This should launch a captive portal navigating your browser to `http://192.168.4.1` (if not, manually go to specified address)
6. Click `Configure WiFi` (wait for scan to complete)
7. Select WiFi network the device should connect to - remember to enter the passphrase
8. Configure the MQTT broker settings and interval between transmits (default 60 seconds)
9. Click `Save`
10. Do something with the published temperatures

_Note:_ Re-configuration of MQTT seconds also requires re-connecting to the WiFI network.

## Building the device

### Hardware

Connect the DS18B20 to the `D2` pin with an external pull-up resistor

```
NodeMCU     DS18B20

D2  ---+--- DATA
       | 
       R1
       |
3V3 ---+--- VCC

GND ------- GND

R1 = 4k7
```

### Software

1. Install `Visual Studio Code`
2. Install extension `PlatformIO IDE`
3. Run PIO `Build`
4. Run PIO `Upload`