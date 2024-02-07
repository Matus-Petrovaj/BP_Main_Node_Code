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

#define DISTANCE_SENSOR_THRESHOLD 5.0
#define BME_SENSOR_THRESHOLD 5.0
#define GAS_SENSOR_THRESHOLD 5.0

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

  if (millis() - ultrasonicLastUpdateTime >= 5000) {  // Update every 5 seconds
    int distance = measureUltrasonic();
    sendSensorData(distance, "ultrasonic", formerDistanceValue, DISTANCE_SENSOR_THRESHOLD);
    ultrasonicLastUpdateTime = millis();
  }

  if (millis() - bme280LastUpdateTime >= 5000) {  // Update every 10 seconds
    float temperature, humidity, pressure;
    measureBME280(temperature, humidity, pressure);
    sendSensorData(temperature, humidity, pressure, "bme280", formerBmeTemperatureValue, formerBmeHumidityValue, formerBmePressureValue, BME_SENSOR_THRESHOLD);
    bme280LastUpdateTime = millis();
  }

  if (millis() - gasSensorLastUpdateTime >= 5000) {  // Update every 15 seconds
    int ppm = measureGasSensor();
    sendSensorData(ppm, "gasSensor", formerGasSensorValue, GAS_SENSOR_THRESHOLD);
    // Assuming gas sensor data is sent immediately without delay
    gasSensorLastUpdateTime = millis();
  }

  delay(1000);  // A small delay to avoid high CPU usage
}

bool isChangeExceedsPercentage(float formerValue, float currentValue, float percentageThreshold) {
  // Exclude the initial -1 value from percentage calculation
  if (formerValue == -1) {
    return true;
  }
  // Calculate the absolute change between former and current values
  float absoluteChange = abs(formerValue - currentValue);

  // Calculate the percentage change
  float percentageChange = (absoluteChange / formerValue) * 100.0;

  // Check if the percentage change exceeds the threshold
  return (percentageChange > percentageThreshold);
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
void sendSensorData(int value, const char* sensorType, float& formerValue, float percentageThreshold) {
  // Check if the percentage change exceeds the threshold
  if (!isChangeExceedsPercentage(formerValue, value, percentageThreshold)) {
    Serial.println("Change does not exceed the threshold. Skipping data transmission.");
    return;
  }

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

  } else {
    Serial.println("Unable to connect to server");
  }
}

// For bme sensor
void sendSensorData(float value1, float value2, float value3, const char* sensorType, float& formerValue1, float& formerValue2, float& formerValue3, float percentageThreshold) {
  // Check if the percentage change exceeds the threshold
  if (!isChangeExceedsPercentage(formerValue1, value1, percentageThreshold) &&
      !isChangeExceedsPercentage(formerValue2, value2, percentageThreshold) &&
      !isChangeExceedsPercentage(formerValue3, value3, percentageThreshold)) {
    Serial.println("Change does not exceed the threshold. Skipping data transmission.");
    return;
  }

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
