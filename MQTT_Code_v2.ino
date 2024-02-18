#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Ultrasonic.h>
#include <MQ135.h>

// WiFi credentials
const char* ssid = "ANDY";
const char* password = "feda43WEKAjeda34..BASEge";

// MQTT broker details
const char* mqtt_server = "192.168.0.74";
const int mqtt_port = 1883;
const char* mqtt_user = "mqtt_matus";
const char* mqtt_password = "Metju123";

#define TRIGGER_PIN D5  // Change the pins for the ultrasonic sensor
#define ECHO_PIN D6

#define BME_SCL D1  // Use D1 and D2 for BME280 sensor
#define BME_SDA D2

#define GAS_SENSOR_ANALOG_PIN A0
#define RZERO 172  // Replace this value with your calibrated RZero !!!

#define MIN_PERCENTAGE_CHANGE 1.0
#define AVG_PERCENTAGE_CHANGE 5.0
#define MAX_PERCENTAGE_CHANGE 10.0

#define FAST_TIME_INTERVAL 5000
#define AVG_TIME_INTERVAL 10000
#define SLOW_TIME_INTERVAL 15000

Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BME280 bme;  // BME280 object
MQ135 gasSensor(GAS_SENSOR_ANALOG_PIN, RZERO);

unsigned long ultrasonicLastUpdateTime = 0;
unsigned long bme280LastUpdateTime = 0;
unsigned long gasSensorLastUpdateTime = 0;

// Initialize former values with initial readings
float formerDistanceValue = 0.0;
float formerBmeTemperatureValue = 0.0;
float formerBmeHumidityValue = 0.0;
float formerBmePressureValue = 0.0;
float formerGasSensorValue = 0.0;

void setup() {
  Serial.begin(9600);
  delay(1000);

  Serial.println("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  Serial.println("Connecting to MQTT broker");
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  Wire.begin(BME_SDA, BME_SCL);   // Initialize I2C communication
  bool status = bme.begin(0x77);  // BME280 address is 0x77

  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  Serial.println("------------------------------------------------");
  unsigned long currentMillis = millis();  // Get the current time

  int currentDistanceValue = measureUltrasonic();
  if (currentMillis - ultrasonicLastUpdateTime >= calculateInterval(formerDistanceValue, currentDistanceValue)) {
    sendSensorData(currentDistanceValue, "ultrasonic", formerDistanceValue);
    ultrasonicLastUpdateTime = millis();
  }

  float currentTemperatureValue, currentHumidityValue, currentPressureValue;
  measureBME280(currentTemperatureValue, currentHumidityValue, currentPressureValue);
  if (currentMillis - bme280LastUpdateTime >= calculateBmeInterval(formerBmeTemperatureValue, formerBmeHumidityValue, formerBmePressureValue, currentTemperatureValue, currentHumidityValue, currentPressureValue)) {
    sendSensorData(currentTemperatureValue, currentHumidityValue, currentPressureValue, "bme280", formerBmeTemperatureValue, formerBmeHumidityValue, formerBmePressureValue);
    bme280LastUpdateTime = millis();
  }

  int currentGasSensorValue = measureGasSensor();
  if (currentMillis - gasSensorLastUpdateTime >= calculateInterval(formerGasSensorValue, currentGasSensorValue)) {
    sendSensorData(currentGasSensorValue, "gasSensor", formerGasSensorValue);
    gasSensorLastUpdateTime = millis();
  }

  delay(2000);  // A small delay to avoid high CPU usage
}

unsigned long calculateInterval(float formerValue, float currentValue) {
  float absoluteChange = abs(formerValue - currentValue);
  float percentageChange = (absoluteChange / formerValue) * 100.0;

  if (percentageChange <= MIN_PERCENTAGE_CHANGE) {
    return SLOW_TIME_INTERVAL;
  } else if (percentageChange <= AVG_PERCENTAGE_CHANGE) {
    return AVG_TIME_INTERVAL;
  } else {
    return FAST_TIME_INTERVAL;
  }
}

unsigned long calculateBmeInterval(float formerValue1, float formerValue2, float formerValue3, float currentTemperature, float currentHumidity, float currentPressure) {
  float temperatureChange = abs(formerValue1 - currentTemperature) / formerValue1 * 100.0;
  float humidityChange = abs(formerValue2 - currentHumidity) / formerValue2 * 100.0;
  float pressureChange = abs(formerValue3 - currentPressure) / formerValue3 * 100.0;

  float maxChange = max(temperatureChange, max(humidityChange, pressureChange));

  if (maxChange <= MIN_PERCENTAGE_CHANGE) {
    return SLOW_TIME_INTERVAL;
  } else if (maxChange <= AVG_PERCENTAGE_CHANGE) {
    return AVG_TIME_INTERVAL;
  } else {
    return FAST_TIME_INTERVAL;
  }
}

int measureUltrasonic() {
  int distance = ultrasonic.read();

  if (distance <= 1 || distance >= 300) {
    distance = -1;  // out of range - bad value
  }

  Serial.println("---------------");
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}

void measureBME280(float& temperature, float& humidity, float& pressure) {
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;  // Pressure in hPa

  Serial.println("---------------");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" Celsius");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");
}

int measureGasSensor() {
  int rzero = gasSensor.getRZero();                                    // Get RZero value
  int correctedResistance = gasSensor.getCorrectedResistance(20, 33);  // Assuming 20°C & 33% humidity
  int ppm = gasSensor.getCorrectedPPM(20, 33);                         // Assuming 20°C & 33% humidity

  Serial.println("---------------");

  Serial.print("Calibrated RZero: ");
  Serial.println(rzero);

  Serial.print("Corrected Resistance: ");
  Serial.println(correctedResistance);

  Serial.print("Corrected PPM: ");
  Serial.println(ppm);

  return ppm;
}

// For gas and distance sensors
void sendSensorData(int value, const char* sensorType, float& formerValue) {
  // Create the HTTP client
  WiFiClient http_client;
  const int httpPort = 80;
  String url = "/script2.php?" + String(sensorType) + "=" + String(value);

  if (http_client.connect("192.168.0.74", httpPort)) {
    http_client.print(String("GET ") + url + " HTTP/1.1\r\n" + "Host: your_server_ip\r\n" + "Connection: close\r\n\r\n");
    delay(10);
    http_client.stop();

    // Update formerValue after successful data transmission
    formerValue = value;

    Serial.println(String(sensorType) + " value sent");
  } else {
    Serial.println("Unable to connect to server");
  }
}

// For bme sensor
void sendSensorData(float value1, float value2, float value3, const char* sensorType, float& formerValue1, float& formerValue2, float& formerValue3) {
  // Create the HTTP client
  WiFiClient http_client;
  const int httpPort = 80;
  String url = "/script2.php?" + String(sensorType) + "1=" + String(value1) + "&" + String(sensorType) + "2=" + String(value2) + "&" + String(sensorType) + "3=" + String(value3);

  if (http_client.connect("192.168.0.74", httpPort)) {
    http_client.print(String("GET ") + url + " HTTP/1.1\r\n" + "Host: your_server_ip\r\n" + "Connection: close\r\n\r\n");
    delay(10);
    http_client.stop();

    // Update formerValues after successful data transmission
    formerValue1 = value1;
    formerValue2 = value2;
    formerValue3 = value3;

    Serial.println("BME values sent");
  } else {
    Serial.println("Unable to connect to server");
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Handle MQTT messages if needed
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("NodeMCU", mqtt_user, mqtt_password)) {
      Serial.println("connected");
      client.subscribe("some_topic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 10 seconds");
      delay(10000);
    }
  }
}
