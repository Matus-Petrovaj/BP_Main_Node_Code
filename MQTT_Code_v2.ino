// Zahrnutie potrebných voľne dostupných knižníc z tretích strán
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

// Definície pre piny ultrasonického senzora
#define ULTRASONIC_TRIGGER_PIN D5
#define ULTRASONIC_ECHO_PIN D6

// Definície pre piny BME280 senzora
#define BME_SCL D1
#define BME_SDA D2

// Definície pre piny plynového senzora
#define GAS_SENSOR_ANALOG_PIN A0
// Hodnota RZero pre plynový senzor MQ-135, potrebné vložit správne nakalibrovanú hodnotu
#define RZERO 156

// Percentuálne makrá pre prácu s meniacimi sa hodnotami
#define EXTRA_SMALL_PERCENTAGE_CHANGE 1.0
#define SMALL_PERCENTAGE_CHANGE 3.0
#define AVG_PERCENTAGE_CHANGE 5.0
#define LARGE_PERCENTAGE_CHANGE 7.0
#define EXTRA_LARGE_PERCENTAGE_CHANGE 10.0

// Makrá pre reprezentáciu intervalov odosielania(v milisekundách)
#define EXTRA_FAST_TIME_INTERVAL 7000
#define FAST_TIME_INTERVAL 10000
#define AVG_TIME_INTERVAL 15000
#define SLOW_TIME_INTERVAL 20000
#define EXTRA_SLOW_TIME_INTERVAL 23000

// Potrebné inicializácie komponentov
Ultrasonic ultrasonic(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN);
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BME280 bme;
MQ135 gasSensor(GAS_SENSOR_ANALOG_PIN, RZERO);

// Inicializácia premenných pre zaznamenávanie poslednej aktualizácie(čas)
unsigned long ultrasonicLastUpdateTime = 0;
unsigned long bme280LastUpdateTime = 0;
unsigned long gasSensorLastUpdateTime = 0;

// Inicializácia predošlých premenných pre výpočet intervalov odosielania
float formerDistanceValue = 0.0;
float formerBmeTemperatureValue = 0.0;
float formerBmeHumidityValue = 0.0;
float formerBmePressureValue = 0.0;
float formerGasSensorValue = 0.0;

// Funkcia setup() zodpovedná za správnu inicializáciu komponentov
void setup() {
  // Začatie sériovej komunikácie s baud rate hodnotou 9600
  Serial.begin(9600);
  delay(1000);

  // Pripojenie k sieti Wi-Fi pomocou zadefinovaného SSID a hesla
  Serial.println("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // pripojenie k MQTT broker pomocou serverovej adresy a portu
  Serial.println("Connecting to MQTT broker");
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Inicializácia I2C komunikácie pomocou portov definovaných v makrách
  Wire.begin(BME_SDA, BME_SCL);
  // BME280 adresa je 0x77
  bool status = bme.begin(0x77);

  // Chybový výpis pri nesprávnom hardvérovom zapojení senzora BME280
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }
}

// Funckia loop(), ktorá sústavne cyklicky beží po funckii setup()
void loop() {
  // Kontrola pripojenia MQTT klienta, volanie funckie reconnect() pokial nie je pripojený 
  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  Serial.println("------------------------------------------------");
  unsigned long currentMillis = millis();  // Get the current time

  // Nasledujúce podmienky slúžia pre meranie hodnôt, následné porovnanie s predošlou hodnotou, na základe ktorého je vyhodnotený interval
  // odosielania a proces samotného odosielania dat
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

  // Oneskorenie aby sme predišli pretaženiu CPU
  delay(2000);
}

/* Funkcia pre kalkuláciu intervalu odosielania na základe aktuálnej a predošlej hodnoty na základe percentuálneho porovnania a
následné priradenie správneho časového intervalu */
unsigned long calculateInterval(float formerValue, float currentValue) {
  float absoluteChange = abs(formerValue - currentValue);
  float percentageChange = (absoluteChange / formerValue) * 100.0;

  // Priradenie individuálnych percentuálnych zmien k prislúchajúcim časovým intervalom
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

// Funkcia podobná k funckii calculateInterval(), akurát optimalizovaná pre BME280 senzor
unsigned long calculateBmeInterval(float formerValue1, float formerValue2, float formerValue3, float currentTemperature, float currentHumidity, float currentPressure) {
  float temperatureChange = abs(formerValue1 - currentTemperature) / formerValue1 * 100.0;
  float humidityChange = abs(formerValue2 - currentHumidity) / formerValue2 * 100.0;
  float pressureChange = abs(formerValue3 - currentPressure) / formerValue3 * 100.0;

  /* Na rozdiel od ostatných senzorov, pri tomto senzore porovnaváme až tri merané hodnoty a vyberieme najvačší percentuálny rozdiel
  a následne priradíme vhodný interval */
  float maxChange = max(temperatureChange, max(humidityChange, pressureChange));

  // Priradenie individuálnych percentuálnych zmien k prislúchajúcim časovým intervalom
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

// Funkcia pre senzor HY-SRF05, ktorá zabezpečuje meranie vzdialenosti
int measureUltrasonic() {
  // Prečíta hodnotu vzdialenosti z ultrasonického senzora
  int distance = ultrasonic.read();

  // Kontrola, či sa hodnota nachádza v správnom rozsahu
  if (distance <= 1 || distance >= 300) {
    distance = -1;
  }

  Serial.println("---------------");
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Návratová hodnota vzdialenosti
  return distance;
}

// Funkcia pre senzor BME280, ktorá zabezpečuje meranie teploty, hustoty a tlaku vzduchu
void measureBME280(float& temperature, float& humidity, float& pressure) {
  // Prečítanie hodnot teploty, vlhkosti a tlaku vzduchu a následný zápis do príslušných premenných
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;

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

// Funkcia pre senzor MQ-135, ktorá zabezpečuje meranie množstva oxidu uhličitého vo vzduchu
int measureGasSensor() {
  // Získanie aktuálnej hodnoty RZero
  int rzero = gasSensor.getRZero();
  // Odhadujeme optimálne 20°C & 33% vlhkosť
  int correctedResistance = gasSensor.getCorrectedResistance(20, 33);
  // Výpočet hodnoty ppm a následné priradenia do premennej ppm
  int ppm = gasSensor.getCorrectedPPM(20, 33);

  Serial.println("---------------");

  Serial.print("Calibrated RZero: ");
  Serial.println(rzero);

  Serial.print("Corrected Resistance: ");
  Serial.println(correctedResistance);

  Serial.print("Corrected PPM: ");
  Serial.println(ppm);

  // Návrat hodnoty premennej ppm
  return ppm;
}

// Funckia pre posielanie dát senzorov HY-SRF05 a MQ-135
void sendSensorData(int value, const char* sensorType, float& formerValue) {
  // Vytvorenie HTTP klienta pomocou inštancie triedy WiFiClient
  WiFiClient http_client;
  // Definícia HTTP portu servera
  const int httpPort = 80;
  // Vytvorenie URL pre HTTP GET Request s typom senzora a hodnotou
  String url = "/script2.php?" + String(sensorType) + "=" + String(value);

  // Kontrola spojenia so serverom
  if (http_client.connect("192.168.0.74", httpPort)) {
    // Konštrukcia a odosielanie HTTP GET Request na server so špecifickou adresou URL
    http_client.print(String("GET ") + url + " HTTP/1.1\r\n" + "Host: your_server_ip\r\n" + "Connection: close\r\n\r\n");
    delay(10);
    // Uzatvorenie konekcie po odoslaní na server
    http_client.stop();

    // Aktualizácia predošlej hodnoty po úspešnom prenose dát
    formerValue = value;

    Serial.println(String(sensorType) + " value sent");
  } else {
    Serial.println("Unable to connect to server");
  }
}

/* Funckia pre posielanie dát senzoru BME280, ktorá je modifikovanou funkciou funckie sendSensorData(int value, const char* sensorType, 
float& formerValue), s rozdielnym počtom parametrov pre zabezpečenie správnej funkcionality pri použití senzora BME280 */
void sendSensorData(float value1, float value2, float value3, const char* sensorType, float& formerValue1, float& formerValue2, float& formerValue3) {
  WiFiClient http_client;
  const int httpPort = 80;
  String url = "/script2.php?" + String(sensorType) + "1=" + String(value1) + "&" + String(sensorType) + "2=" + String(value2) + "&" + String(sensorType) + "3=" + String(value3);

  if (http_client.connect("192.168.0.74", httpPort)) {
    http_client.print(String("GET ") + url + " HTTP/1.1\r\n" + "Host: your_server_ip\r\n" + "Connection: close\r\n\r\n");
    delay(10);
    http_client.stop();

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
