#include <Arduino.h>

#include <FS.h> // first to avoid problems
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

// needed for WiFiManager
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

// for LED status
#include <Ticker.h>

// for storing configuration
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include <OneWire.h>
#include <DallasTemperature.h>

#include <PubSubClient.h>

// forward declarations
bool saveConfigurationToFlash();
void factoryReset();
void tick();
bool readConfigurationFromFlash();
void saveConfigCallback();
bool isFlashButtonPressed();
void enterSleep(uint32_t currentRuntime, bool enterDeepSleep);
void configModeCallback (WiFiManager *myWiFiManager);
bool ensureWifiConnection();
void readTemperature();
bool ensureMqttConnection();
void mqttPublishTemperature();

// Interval between wake-ups / temperature readings
#define INTERVAL 30e6

#define STATUS_LED D4
#define FLASH_BUTTON D3

#define ONEWIRE_PIN D2

Ticker ticker;

OneWire oneWire(ONEWIRE_PIN);

DallasTemperature sensors(&oneWire);

const char* configuration_file = "/wifithermometer_config.json";

float temperature; // read temp, reset when transmitted

// Configurable parameters
char mqtt_server[40];
long mqtt_port = 1883;
char mqtt_user[50];
char mqtt_pass[50];
char mqtt_topic[100];
long sleep_time = 60;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// flag for saving data
bool shouldSaveConfig = false;

void setup() {
  uint32_t time_start = millis();

  //Serial.begin(115200);
  Serial.begin(9600);
  Serial.setTimeout(2000);

  // Wait for serial to initialize.
  while (!Serial) { }

  Serial.println("I'm awake.");

  // Start DS18B20 sensor
  sensors.begin();

  // Configure ticker LED pin
  pinMode(STATUS_LED, OUTPUT);

  // Read configuration from flash
  if (readConfigurationFromFlash()) {
    Serial.println("Configuration read, continue.");
  } else {
    // TODO proper error handling
    Serial.println("ERROR: Failed reading configuration, aborting.");
  }

  if (true) {
    uint32_t run_duration = millis() - time_start;
    Serial.print(F("Duration after reading config:"));
    Serial.println(run_duration);
  }

  // Setup the MQTT stuff
  mqttClient.setServer(mqtt_server, mqtt_port);

  // give us at least 5 seconds to trigger the configuration portal
  uint32_t run_duration = millis() - time_start;
  uint32_t delay_sleep = 3000 - run_duration;
  if (run_duration > 3000)
    delay_sleep = 0;

  Serial.print(F("Run duration: "));
  Serial.println(run_duration);
  Serial.print(F("Delay sleep: "));
  Serial.println(delay_sleep);

  if (delay_sleep > 0) {
    delay(delay_sleep);
  }

  if (isFlashButtonPressed()) {
    Serial.println(F("Configuration requested.."));

    WiFiManager wifiManager;

    char buf_mqtt_port[12];
    ltoa(mqtt_port, buf_mqtt_port, 10);

    char buf_sleep_time[12];
    ltoa(sleep_time, buf_sleep_time, 10);

    WiFiManagerParameter custom_mqtt_server("mqtt_server", "MQTT Server", mqtt_server, 40);
    WiFiManagerParameter custom_mqtt_port("mqtt_port", "MQTT Port", buf_mqtt_port, 6);
    WiFiManagerParameter custom_mqtt_user("mqtt_user", "MQTT User", mqtt_user, 50);
    WiFiManagerParameter custom_mqtt_pass("mqtt_pass", "MQTT Password", mqtt_pass, 50);
    WiFiManagerParameter custom_mqtt_topic("mqtt_topic", "MQTT Topic", mqtt_topic, 120);

    WiFiManagerParameter custom_sleep_time("sleep_time", "Interval between transmits (seonds)", buf_sleep_time, 6);

    WiFiManagerParameter custom_factory_reset("factory_reset", "Factory Reset (y)", "", 2);

    wifi_station_disconnect();
    String ssid = "WifiThermometer-" + String(ESP.getChipId());
    wifiManager.setAPCallback(configModeCallback);

    // set config save notify callback
    wifiManager.setSaveConfigCallback(saveConfigCallback);
    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_port);
    wifiManager.addParameter(&custom_mqtt_user);
    wifiManager.addParameter(&custom_mqtt_pass);
    wifiManager.addParameter(&custom_mqtt_topic);
    wifiManager.addParameter(&custom_sleep_time);

    wifiManager.addParameter(&custom_factory_reset);

    wifiManager.setConfigPortalTimeout(180);
    wifiManager.startConfigPortal(ssid.c_str(), NULL);

    if (shouldSaveConfig) {
      Serial.println(F("Storing configuration"));

      // Handle factory reset
      char freset[2];
      strcpy(freset, custom_factory_reset.getValue());
      if (freset[0] == 'Y' || freset[0] == 'y') {
        factoryReset();
      }

      // Convert param
      strcpy(mqtt_server, custom_mqtt_server.getValue());
      mqtt_port = strtol(custom_mqtt_port.getValue(), NULL, 0);
      strcpy(mqtt_user, custom_mqtt_user.getValue());
      strcpy(mqtt_pass, custom_mqtt_pass.getValue());
      strcpy(mqtt_topic, custom_mqtt_topic.getValue());
      sleep_time = strtol(custom_sleep_time.getValue(), NULL, 0);

      // Store parameters
      saveConfigurationToFlash();
    }
  }
  ticker.detach();
  digitalWrite(STATUS_LED, HIGH); // act led off
  Serial.println(F("Setup completed."));
}

uint32_t time_last_tx = 0;

void loop() {
  Serial.println(F("Loop started."));

  digitalWrite(STATUS_LED, LOW);

  Serial.print(F("Wifi status: "));
  Serial.println(WiFi.status());

  uint32_t time_start = millis();

  readTemperature();

  // Publish MQTT message with temperature
  if (ensureWifiConnection()) {
    Serial.println(F("Transmitting.."));
    Serial.print(F("The temp: "));
    Serial.println(temperature);

    if (ensureMqttConnection()) {
      mqttPublishTemperature();
    } else {
      Serial.println(F("No MQTT connection, skip MQTT publish"));
    }
  } else {
    Serial.println(F("No WiFi, skip MQTT publish"));
  }

  digitalWrite(STATUS_LED, HIGH); // act led off

  Serial.println(F("Loop complete."));

  enterSleep(millis() - time_start, false);
}

bool ensureMqttConnection() {
  uint8_t attempts = 3;

  while (!mqttClient.connected()) {
    Serial.print(F("Attempting MQTT connection..."));
    String clientId = "WifiThermometer-" + String(ESP.getChipId());
    // Attempt to connect
    boolean mqttConnected = false;
    if (strlen(mqtt_user) > 0) {
      mqttConnected = mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass);
    } else {
      mqttConnected = mqttClient.connect(clientId.c_str());
    }
    if (mqttConnected) {
      Serial.println(F("connected"));
      return true;
    } else {
      Serial.print(F("failed, rc="));
      Serial.println(mqttClient.state());
      if (--attempts > 0) {
        Serial.println(F("Retrying MQTT connection in 3 seconds"));
        delay(3000);
      } else {
        Serial.println(F("Exhausted MQTT connection retries. Giving up."));
        return false;
      }
    }
  }
  return true;
}

void mqttPublishTemperature() {
  char buf[20];
  dtostrf(temperature, 2, 2, buf);
  boolean result;
  result = mqttClient.publish(mqtt_topic, buf);
  Serial.print(F("MQTT publish result="));
  Serial.println(result);
}

bool ensureWifiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    // Start 'Connecting' ticker
    ticker.attach(0.6, tick);

    WiFiManager wifiManager;

    wifiManager.setDebugOutput(true);

    // hack to disable configuration portal when unable to autoconnect()
    wifiManager.setConfigPortalTimeout(1);

    // Connected to WiFi
    if (wifiManager.autoConnect()) {
      Serial.print(F("Connected to: "));
      Serial.println(WiFi.SSID());
    } else {
      Serial.println(F("Unable to connect to WiFi"));
    }

    // Stop ticker, keep LED on
    ticker.detach();
    digitalWrite(STATUS_LED, LOW);
  }
  return WiFi.status() == WL_CONNECTED;
}

void tick() {
  //toggle state
  int state = digitalRead(STATUS_LED);  // get the current state of GPIO1 pin
  digitalWrite(STATUS_LED, !state);     // set pin to the opposite state
}

void readTemperature() {
  sensors.requestTemperatures(); 
  temperature = sensors.getTempCByIndex(0);
  Serial.println("Temperature:");
  Serial.print(temperature);
  Serial.println("ºC");
}

void factoryReset() {
  Serial.println(F("Factory Reset!"));

  // Clear WiFi configuration
  WiFi.disconnect(true);

  Serial.println(F("Mounting FileSystem.."));

  if (SPIFFS.begin()) {
    Serial.println(F("mounted file system"));
    if (SPIFFS.exists(configuration_file)) {
      //file exists, reading and loading
      Serial.println(F("reading config file"));
      if (SPIFFS.remove(configuration_file)) {
        Serial.print(F("Removed:"));
        Serial.println(configuration_file);
      }
    }
  }
  Serial.println(F("Everything reset, rebooting."));
  ESP.reset();
}

bool saveConfigurationToFlash() {
  Serial.print(F("Saving configuration to"));
  Serial.println(configuration_file);
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();

  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["mqtt_user"] = mqtt_user;
  json["mqtt_pass"] = mqtt_pass;
  json["mqtt_topic"] = mqtt_topic;
  json["sleep_time"] = sleep_time;

  if (SPIFFS.begin()) {
    File configFile = SPIFFS.open(configuration_file, "w");
    if (!configFile) {
      Serial.print(F("failed to open file for writing: "));
      Serial.println(configuration_file);
    }

    json.prettyPrintTo(Serial);
    json.printTo(configFile);
    configFile.close();
    return true;
  }
  return false;
}

bool readConfigurationFromFlash() {
  Serial.println(F("Mounting FileSystem.."));

  if (SPIFFS.begin()) {
    Serial.println(F("Mounted FileSystem."));
    if (SPIFFS.exists(configuration_file)) {
      Serial.println(F("Reading config file"));
      File configFile = SPIFFS.open(configuration_file, "r");
      if (configFile) {
        Serial.println(F("Opened config file"));
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println(F("\nParsed configuration from json:"));

          strcpy(mqtt_server, json["mqtt_server"]);
          mqtt_port = json["mqtt_port"];
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_pass, json["mqtt_pass"]);
          strcpy(mqtt_topic, json["mqtt_topic"]);
          sleep_time = json["sleep_time"];

          Serial.println(F("Configuration read:"));

          Serial.print(F("mqtt_server = "));
          Serial.println(mqtt_server);

          Serial.print(F("mqtt_port = "));
          Serial.println(mqtt_port);

          Serial.print(F("mqtt_user = "));
          Serial.println(mqtt_user);

          Serial.print(F("mqtt_pass = "));
          Serial.println(mqtt_pass);

          Serial.print(F("mqtt_topic = "));
          Serial.println(mqtt_topic);

          Serial.print(F("sleep_time = "));
          Serial.println(sleep_time);

          return true;
        } else {
          Serial.print(F("Failed to load config"));
          Serial.println(configuration_file);
        }
        configFile.close();
      }
    }
  } else {
    Serial.println(F("failed to mount FS"));
  }

  return false;
}

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println(F("Should save config"));
  shouldSaveConfig = true;
}

bool isFlashButtonPressed() {
  pinMode(FLASH_BUTTON, INPUT);
  uint8_t btn = digitalRead(FLASH_BUTTON);
  Serial.print(F("FlashButton: "));
  Serial.println(btn);
  return btn == 0;
}

void enterSleep(uint32_t currentRuntime, bool enterDeepSleep) {
  uint32_t wantedSleepTime = sleep_time * 1e6;
  uint32_t calculatedSleepTime = wantedSleepTime - (1000 * currentRuntime);
  if (calculatedSleepTime > wantedSleepTime)
    calculatedSleepTime = wantedSleepTime;
  Serial.print(F("Preparing to sleep: "));
  Serial.println(calculatedSleepTime);

  Serial.print(F("Wifi status: "));
  Serial.println(WiFi.status());

  if (enterDeepSleep) {
    // 20e6 is 20 seconds
    ESP.deepSleep(calculatedSleepTime);
  } else {
    wifi_set_sleep_type(LIGHT_SLEEP_T);
    delay(calculatedSleepTime / 1000);
  }
}

/* WiFiManager callbacks */

//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println(F("Entered config mode"));
  Serial.print(F("AP IP: "));
  Serial.println(WiFi.softAPIP());
  Serial.print(F("AP SSID: "));
  Serial.println(myWiFiManager->getConfigPortalSSID());

  // Start 'Configuration' ticker
  ticker.attach(0.2, tick);
}