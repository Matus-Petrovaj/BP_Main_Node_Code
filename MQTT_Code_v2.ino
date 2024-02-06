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
#define RZERO 185 // Replace this value with your calibrated RZero !!!

Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BME280 bme;  // BME280 object
MQ135 gasSensor(GAS_SENSOR_ANALOG_PIN, RZERO);

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

  int distance = ultrasonic.read();

  if (distance <= 3 || distance >= 200) {
    distance = -1;    // out of range - bad value
  }

  Serial.println("---------------");
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();
  float pressure = bme.readPressure() / 100.0F;  // Pressure in hPa

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

  // Gas Sensor
  float rzero = gasSensor.getRZero(); // Get RZero value
  float correctedResistance = gasSensor.getCorrectedResistance(20, 33); // Assuming 20°C & 33% humidity
  float ppm = gasSensor.getCorrectedPPM(20, 33); // Assuming 20°C & 33% humidity

  Serial.println("---------------");

  Serial.print("Calibrated RZero: ");
  Serial.println(rzero);

  Serial.print("Corrected Resistance: ");
  Serial.println(correctedResistance);

  Serial.print("Corrected PPM: ");
  Serial.println(ppm);

  // Create the HTTP client
  WiFiClient http_client;
  const int httpPort = 80;
  String url = "/script2.php?distance=" + String(distance) +
               "&temperature=" + String(temperature) +
               "&humidity=" + String(humidity) +
               "&pressure=" + String(pressure) +
               "&rzero=" + String(rzero) +
               "&correctedResistance=" + String(correctedResistance) +
               "&ppm=" + String(ppm);

  if (http_client.connect("192.168.0.74", httpPort)) {
    http_client.print(String("GET ") + url + " HTTP/1.1\r\n" + "Host: your_server_ip\r\n" + "Connection: close\r\n\r\n");
    delay(10);
    http_client.stop();
  } else {
    Serial.println("Unable to connect to server");
  }

  delay(4000);
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
