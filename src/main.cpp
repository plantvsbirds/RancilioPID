#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "PID_v1.h"
#include "config.h"

AsyncMqttClient mqttClient;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;

Ticker mqttReconnectTimer;
Ticker wifiReconnectTimer;
Ticker publishStateTimer;

OneWire oneWire(PIN_TEMP_SENSOR);
DallasTemperature sensors(&oneWire);

double currentTemp, heatingPower, targetTemp = 96.0;
double startKp = 17.5, startKi = 0, startKd = 0;
double aggKp = 17.5, aggKi = 0.14, aggKd = 10;

boolean isPoweredOn = false, isStarting = true, reachedTargetTemp = false;

PID rancilioPID(&currentTemp, &heatingPower, &targetTemp, aggKp, aggKi, aggKd, DIRECT);

unsigned int WindowSize = 800;
unsigned long windowStartTime;
unsigned long windowRestart;

bool doPowerSwitch = false;

void connectToWifi();
void onWifiConnect(const WiFiEventStationModeGotIP& event);
void onWifiDisconnect(const WiFiEventStationModeDisconnected& event);
void connectToMqtt();
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttSubscribe(uint16_t packetId, uint8_t qos);
void onMqttUnsubscribe(uint16_t packetId);
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
void onMqttPublish(uint16_t packetId);
void onPublishState();
void toggleMachinePower();

void connectToWifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to WiFi");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from WiFi");
  mqttReconnectTimer.detach(); // ensure cant renonnect MQTT while reconnecting WiFi
  wifiReconnectTimer.once(2, connectToWifi);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT");
  
  mqttClient.subscribe(MQTT_TOPIC_MODE, 0);
  mqttClient.subscribe(MQTT_TOPIC_TARGET, 0);

  mqttClient.publish(MQTT_TOPIC_CONFIG, 0, false, "{\"~\":\"homeassistant/climate/ranciliopid/ranciliocontrol\",\"name\":\"Rancilio PID\",\"mode_cmd_t\":\"~/modeCmd\",\"mode_stat_t\":\"~/state\",\"mode_stat_tpl\":\"{{ value_json.mode }}\",\"avty_t\":\"~/available\",\"pl_avail\":\"on\",\"pl_not_avail\":\"off\",\"temp_cmd_t\":\"~/targetTempCmd\",\"temp_stat_t\":\"~/state\",\"temp_stat_tpl\":\"{{ value_json.target }}\",\"curr_temp_t\":\"~/state\",\"curr_temp_tpl\":\"{{ value_json.temp }}\",\"min_temp\":\"0\",\"max_temp\":\"100\",\"temp_step\":\"0.25\",\"modes\":[\"off\",\"heat\"]}");
  mqttClient.publish(MQTT_TOPIC_AVAILABLE, 0, false, "on");

  publishStateTimer.attach(5, onPublishState);
} 

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT");

  publishStateTimer.detach();

  if(WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  publishStateTimer.detach();

  char *value = new char[len+1];
  strncpy(value, payload, len);
  value[len] = '\0';
  
  if(strcmp(topic, MQTT_TOPIC_TARGET) == 0) {
    Serial.println("Incomming target temp cmd");
    float value = atof(payload);
    targetTemp = value;
  } else if(strcmp(topic, MQTT_TOPIC_MODE) == 0) {
    Serial.println("Incomming mode cmd");
    if(strcmp(value, "off") == 0) {
      if(isPoweredOn) {
        isPoweredOn = false;
        doPowerSwitch = true;
        rancilioPID.SetMode(MANUAL);
      } else
        Serial.println("Machine is already off");
    } else if(strcmp(value, "heat") == 0) {
      if(!isPoweredOn) {
        isPoweredOn = true;
        doPowerSwitch = true;
        rancilioPID.SetMode(AUTOMATIC);
      } else
        Serial.println("Machine is already heating");
    } else {
      Serial.println("Unknown target mode ");
      Serial.print(value);
    }
  }

  onPublishState();
  publishStateTimer.attach(5, onPublishState);  
}

void onPublishState() {
  if(mqttClient.connected()) {
    const size_t capacity = JSON_OBJECT_SIZE(3);
    DynamicJsonDocument doc(capacity);

    doc["temp"] = currentTemp;
    doc["target"] = targetTemp;
    if(isPoweredOn)
      doc["mode"] = "heat";
    else
      doc["mode"] = "off";

    char buffer[256];
    size_t n = serializeJson(doc, buffer);

    mqttClient.publish(MQTT_TOPIC_STATE, 0, true, buffer, n);  
  }
}

void toggleMachinePower() {
  Serial.println("Toggle Machine Power");
  digitalWrite(PIN_POWER_SWITCH, LOW);
	delay(200);
	digitalWrite(PIN_POWER_SWITCH, HIGH);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USER, MQTT_PASSWORD);
  mqttClient.setWill(MQTT_TOPIC_AVAILABLE, 2, true, "off");

  connectToWifi();

  sensors.requestTemperatures();
	currentTemp = sensors.getTempCByIndex(0);

	windowStartTime = millis();
  windowRestart = millis();

	rancilioPID.SetOutputLimits(0, WindowSize);

	pinMode(PIN_HEATER, OUTPUT);
	pinMode(PIN_POWER_SWITCH, OUTPUT);
  
	digitalWrite(PIN_POWER_SWITCH, HIGH);

  isPoweredOn = true;
  toggleMachinePower();
  rancilioPID.SetMode(AUTOMATIC);
}

void loop() {
  if(doPowerSwitch) {
    toggleMachinePower();
    doPowerSwitch = false;
  }

  sensors.requestTemperatures();
	currentTemp = sensors.getTempCByIndex(0);

  if(currentTemp < 40)
    isStarting = true;

  if(currentTemp <= 0) {
    Serial.println("Current temperature is negative, sensor might not be connected");
  } else if (currentTemp > 0 && currentTemp < 100 && isPoweredOn) {
		if (millis() - windowStartTime > WindowSize) {
			windowStartTime += WindowSize;
			if (isStarting) {
				// Coldstart
				if (currentTemp < targetTemp - 15) {
					rancilioPID.SetTunings(startKp, startKi, startKd);
				}
				else {
					rancilioPID.SetTunings(aggKp, aggKi, aggKd);
					if (currentTemp > targetTemp - 0.5) {
						isStarting = false;
					}
				}
			}
			else
			{
				// Brewdetection when coldstart is finished
				if (currentTemp > targetTemp - 0.5) {
					rancilioPID.SetTunings(aggKp, aggKi, aggKd);
				}
				else if (currentTemp < targetTemp - 1)
				{
					rancilioPID.SetTunings(80, 0, 800);
				}
			}
		}

		rancilioPID.Compute();

		int compare = millis() - windowStartTime;
		Serial.print(heatingPower);
		Serial.print(" ");
		Serial.print(compare);
		if (heatingPower < compare)
		{
			digitalWrite(PIN_HEATER, LOW);
			Serial.println("Write Output PIN L0W");
		}
		else
		{
			digitalWrite(PIN_HEATER, HIGH);
			Serial.println("Write Output PIN H1GH");
		}
	}
	else
	{
		digitalWrite(PIN_HEATER, LOW);
		Serial.println("Machine is not powered on");
		windowStartTime = millis();
	}

  // Restart machine every 25 mins before auto turn off
  if (millis() > windowRestart + (25 * 60 * 1000) && isPoweredOn) {
    toggleMachinePower();
    delay(300);
    toggleMachinePower();
    windowRestart = millis();
  }
}