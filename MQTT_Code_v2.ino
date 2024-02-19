#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Ultrasonic.h>
#include <MQ135.h>

// WiFi parametre
const char* ssid = "ANDY";
const char* password = "feda43WEKAjeda34..BASEge";

// MQTT broker parametre
const char* mqtt_server = "192.168.0.74";
const int mqtt_port = 1883;
const char* mqtt_user = "mqtt_matus";
const char* mqtt_password = "Metju123";

// Definicie pre piny ultrasonickeho senzora
#define TRIGGER_PIN D5
#define ECHO_PIN D6

// Definicie pre piny ultrasonickeho senzora
#define BME_SCL D1
#define BME_SDA D2

#define GAS_SENSOR_ANALOG_PIN A0
// Hodnota RZero pre plynovy senzor MQ-135, potrebne vlozit spravne nakalibrovanu hodnotu
#define RZERO 115

// Percentualne makra pre pracu s meniacimi sa hodnotami
#define EXTRA_SMALL_PERCENTAGE_CHANGE 1.0
#define SMALL_PERCENTAGE_CHANGE 3.0
#define AVG_PERCENTAGE_CHANGE 5.0
#define LARGE_PERCENTAGE_CHANGE 7.0
#define EXTRA_LARGE_PERCENTAGE_CHANGE 10.0

// Makra pre reprezentaciu intervalov odosielania(v milisekundach)
#define EXTRA_FAST_TIME_INTERVAL 3000
#define FAST_TIME_INTERVAL 5000
#define AVG_TIME_INTERVAL 10000
#define SLOW_TIME_INTERVAL 15000
#define EXTRA_SLOW_TIME_INTERVAL 20000

// Potrebne inicializacie komponentov
Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BME280 bme;  // BME280 object
MQ135 gasSensor(GAS_SENSOR_ANALOG_PIN, RZERO);

// Inicializacia premennych pre zaznamenavanie poslednej aktualizacie(cas)
unsigned long ultrasonicLastUpdateTime = 0;
unsigned long bme280LastUpdateTime = 0;
unsigned long gasSensorLastUpdateTime = 0;

// Inicializacia predoslych premennych pre vypocet intervalov odosielania
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

  // Inicializacia I2C komunikacie
  Wire.begin(BME_SDA, BME_SCL);
  // BME280 adresa je 0x77
  bool status = bme.begin(0x77);

  // chybovy vypis pri nespravnom hardverovom zapojeni senzora BME280
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

  // Nasledujuce podmienky sluzia pre meranie hodnot, nasledne porovnanie s predoslou hodnotou, na zaklade ktoreho je vyhodnoteny interval
  // odosielania a proces samotneho odosielania dat
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

  // Oneskorenie aby sme predisli pretazeniu CPU
  delay(2000);
}

// Funkcia pre kalkulaciu intervalu odosielania na zaklade aktualnej a predoslej hodnoty na zaklade percentualneho porovnania a
// nasledne priradenie spravneho casoveho intervalu
unsigned long calculateInterval(float formerValue, float currentValue) {
  float absoluteChange = abs(formerValue - currentValue);
  float percentageChange = (absoluteChange / formerValue) * 100.0;

  // Priradenie individualnych percentualnych zmien k prisluchajucim casovym intervalom
  if (percentageChange <= EXTRA_SMALL_PERCENTAGE_CHANGE) {
    return EXTRA_SLOW_TIME_INTERVAL;
  } else if (percentageChange <= SMALL_PERCENTAGE_CHANGE) {
    return SLOW_TIME_INTERVAL;
  } else if (percentageChange <= AVG_PERCENTAGE_CHANGE) {
    return AVG_TIME_INTERVAL;
  } else if (percentageChange <= LARGE_PERCENTAGE_CHANGE) {
    return FAST_TIME_INTERVAL;
  } else {
    return EXTRA_FAST_TIME_INTERVAL;
  }
}

// Funkcia podobna k calculateInterval, akurat optimalizovana pre BME280 senzor
unsigned long calculateBmeInterval(float formerValue1, float formerValue2, float formerValue3, float currentTemperature, float currentHumidity, float currentPressure) {
  float temperatureChange = abs(formerValue1 - currentTemperature) / formerValue1 * 100.0;
  float humidityChange = abs(formerValue2 - currentHumidity) / formerValue2 * 100.0;
  float pressureChange = abs(formerValue3 - currentPressure) / formerValue3 * 100.0;

  // Na rozdiel od ostatnych senzorov, pri tomto senzore porovnavame az tri merane hodnoty a vyberieme najvacsi percentualny rozdiel
  // a nasledne priradime vhodny interval
  float maxChange = max(temperatureChange, max(humidityChange, pressureChange));

  // Priradenie individualnych percentualnych zmien k prisluchajucim casovym intervalom
  if (maxChange <= EXTRA_SMALL_PERCENTAGE_CHANGE) {
    return EXTRA_SLOW_TIME_INTERVAL;
  } else if (maxChange <= SMALL_PERCENTAGE_CHANGE) {
    return SLOW_TIME_INTERVAL;
  } else if (maxChange <= AVG_PERCENTAGE_CHANGE) {
    return AVG_TIME_INTERVAL;
  } else if (maxChange <= LARGE_PERCENTAGE_CHANGE) {
    return FAST_TIME_INTERVAL;
  } else {
    return EXTRA_FAST_TIME_INTERVAL;
  }
}

// Funkcia pre senzor HY-SRF05, ktora zabezpecuje meranie vzdialenosti
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

// Funkcia pre senzor BME280, ktora zabezpecuje meranie teploty, hustoty a tlaku vzduchu
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

// Funkcia pre senzor MQ-135, ktora zabezpecuje meranie mnozstva oxidu uhliciteho vo vzduchu
int measureGasSensor() {
  // Ziskanie aktualnej hodnoty RZero
  int rzero = gasSensor.getRZero();
  // Odhadujeme optimalne 20°C & 33% vlhkost
  int correctedResistance = gasSensor.getCorrectedResistance(20, 33);
  int ppm = gasSensor.getCorrectedPPM(20, 33);

  Serial.println("---------------");

  Serial.print("Calibrated RZero: ");
  Serial.println(rzero);

  Serial.print("Corrected Resistance: ");
  Serial.println(correctedResistance);

  Serial.print("Corrected PPM: ");
  Serial.println(ppm);

  return ppm;
}

// Funckia pre posielanie dat senzorov HY-SRF05 a MQ-135
void sendSensorData(int value, const char* sensorType, float& formerValue) {
  // Vytvorenie HTTP klienta
  WiFiClient http_client;
  const int httpPort = 80;
  String url = "/script2.php?" + String(sensorType) + "=" + String(value);

  if (http_client.connect("192.168.0.74", httpPort)) {
    http_client.print(String("GET ") + url + " HTTP/1.1\r\n" + "Host: your_server_ip\r\n" + "Connection: close\r\n\r\n");
    delay(10);
    http_client.stop();

    // Aktualizacia predoslej hodnoty po uspesnom prenose dat
    formerValue = value;

    Serial.println(String(sensorType) + " value sent");
  } else {
    Serial.println("Unable to connect to server");
  }
}

// Funckia pre posielanie dat senzoru BME280
void sendSensorData(float value1, float value2, float value3, const char* sensorType, float& formerValue1, float& formerValue2, float& formerValue3) {
  // Vytvorenie HTTP klienta
  WiFiClient http_client;
  const int httpPort = 80;
  String url = "/script2.php?" + String(sensorType) + "1=" + String(value1) + "&" + String(sensorType) + "2=" + String(value2) + "&" + String(sensorType) + "3=" + String(value3);

  if (http_client.connect("192.168.0.74", httpPort)) {
    http_client.print(String("GET ") + url + " HTTP/1.1\r\n" + "Host: your_server_ip\r\n" + "Connection: close\r\n\r\n");
    delay(10);
    http_client.stop();

    // Aktualizacia predoslych hodnot po uspesnom prenose dat
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
