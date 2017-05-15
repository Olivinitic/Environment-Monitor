/***************************************************
  Adafruit ESP8266 Sensor Module

  Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino
  Works great with Adafruit's Huzzah ESP board:
  ----> https://www.adafruit.com/product/2471
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
  Written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

// Libraries
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
// Sensor Libraries
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif
#include <Wire.h>                       // Adafruit I2C Library
#include <Adafruit_Sensor.h>            // Adafruit Sensor Library
#include <OneWire.h>                    // One Wire library for the digital temp sensor
#include <DallasTemperature.h>          // Library for digital temp sensor
#include "DHT.h"                        // DHT11 Sensor Library

// DEFINITIONS
  // WiFi parameters
  #define WLAN_SSID       "CenturyLink0141"
  #define WLAN_PASS       "ae7nad6ecabapy"
  // Adafruit IO
  #define AIO_SERVER      "io.adafruit.com"
  #define AIO_SERVERPORT  1883
  #define AIO_USERNAME    "oligon"
  #define AIO_KEY         "9b9ec3768ccd4ebf83a67d2223e6e2b8"
  // COUNTER DEFINITIONS
  int counter = 0;  // counter variable to keep track of # of measurements.
  // LED INDICATOR DEFINITIONS
  int LED0 = 0;             // connect Red LED to Digital pin 9 (PWM pin)
  int LED0brightness;       // Used when controlling brightness
  int LED1 = 4;            // connect Blue LED to Digital pin 10 (PWM pin)
  int LED1brightness;       // Used when controlling brightness (PWM)
  int LED2 = 5;            // connect Green LED to Digital pin 11 (PWM pin)
  int LED2brightness;       // Used when controlling brightness (PWM)
  // PHOTOCELL DEFINITIONS
  int photocellControl = 13;      // Digital 2 Control Signal
  int photocellPin = A0;          // Analog 'A' Data Signal
  int photocellReading;          // The analog reading from the sensor divider
  // SOIL MOISTURE SENSOR DEFINITIONS
  /*int soilMoistureControl = 12;   // Digital 4 Control Signal
  int soilMoisturePin = A;      // Analog 1 Data Signal
  int soilMoistureReading;       // The analog reading from the sensor itself
  int thresholdUp = 400;         // High threshold that is used in the Water Cycle (Stop Water)
  int thresholdDown = 250;       // Low threshold that is used in the Water Cycle (Start Water)
  */
  // DIGITAL TEMPERATURE DEFINITIONS
  int soilTempReading;
  #define ONE_WIRE_BUS 14    // Data wire is plugged into port 2 on the Arduino
  OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
  DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
  // DHT11 HUMIDITY AND TEMPERATURE SENSOR DEFINITIONS
  #define DHTPIN 16          // Digital pin 7
  #define DHTTYPE DHT11     // DHT 11 type (The one I have)
  //#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
  //#define DHTTYPE DHT21   // DHT 21 (AM2301)
  DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor.


// Functions
void connect();

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;

// Store the MQTT server, client ID, username, and password in flash memory.
const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;

// Set a unique MQTT client ID using the AIO key + the date and time the sketch
// was compiled (so this should be unique across multiple devices for a user,
// alternatively you can manually set this to a GUID or other random value).
const char MQTT_CLIENTID[] PROGMEM  = AIO_KEY __DATE__ __TIME__;
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT, MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD);

/****************************** Feeds ***************************************/

// Setup feeds for Light Intensity, Soil Moisture, Soil Temperature,
// Humidity, Temp (C), Temp (F), Heat Index (C), Heat Index (F)
const char LIGHT_INTESITY_FEED[] PROGMEM = AIO_USERNAME "/feeds/light_intensity";
Adafruit_MQTT_Publish light_intensity = Adafruit_MQTT_Publish(&mqtt, LIGHT_INTESITY_FEED);

//const char SOIL_MOISTURE_FEED[] PROGMEM = AIO_USERNAME "/feeds/soil_moisture";
//Adafruit_MQTT_Publish soil_moisture = Adafruit_MQTT_Publish(&mqtt, SOIL_MOISTURE_FEED);

const char SOIL_TEMP_FEED[] PROGMEM = AIO_USERNAME "/feeds/soil_temp";
Adafruit_MQTT_Publish soil_temp = Adafruit_MQTT_Publish(&mqtt, SOIL_TEMP_FEED);

const char HUMIDITY_FEED[] PROGMEM = AIO_USERNAME "/feeds/humidity";
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, HUMIDITY_FEED);

const char TEMP_CELSIUS_FEED[] PROGMEM = AIO_USERNAME "/feeds/temp_celsius";
Adafruit_MQTT_Publish temp_celsius = Adafruit_MQTT_Publish(&mqtt, TEMP_CELSIUS_FEED);

const char TEMP_FAHRENHEIT_FEED[] PROGMEM = AIO_USERNAME "/feeds/temp_fahrenheit";
Adafruit_MQTT_Publish temp_fahrenheit = Adafruit_MQTT_Publish(&mqtt, TEMP_FAHRENHEIT_FEED);

const char HEAT_INDEX_C_FEED[] PROGMEM = AIO_USERNAME "/feeds/heat_index_c";
Adafruit_MQTT_Publish heat_index_c = Adafruit_MQTT_Publish(&mqtt, HEAT_INDEX_C_FEED);

const char HEAT_INDEX_F_FEED[] PROGMEM = AIO_USERNAME "/feeds/heat_index_f";
Adafruit_MQTT_Publish heat_index_f = Adafruit_MQTT_Publish(&mqtt, HEAT_INDEX_F_FEED);

/*************************** Sketch Code ************************************/

void setup() {
  //while (!Serial);  // required for Flora & Micro
  //delay(500);       // Wait 0.5 sec before checking again

  Serial.begin(115200);

  // System Information
  Serial.println(F("Garden Code Test! HUZZAH"));
  Serial.println(F("Version 1.0"));
  Serial.println();

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  delay(10);
  Serial.print(F("Connecting to "));
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);  //Start Wifi Connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
  // connect to adafruit io
  connect();

  // Inidcator LEDs setup
  pinMode(LED0, OUTPUT);  // Sets Digital Pin 9 as Output
  pinMode(LED1, OUTPUT); // Sets Digital Pin 10 as Output
  pinMode(LED2, OUTPUT);  // Sets Digital Pin 11 as Output
  // Photocell setup
  pinMode(photocellControl, OUTPUT);    // Sets Digital Pin 2 as Output
  // Soil Moisture setup
  //pinMode(soilMoistureControl, OUTPUT); // Sets Digital Pin 4 as Output
  // Digital Temp setup
  sensors.begin();
  // DHT11 setup
  dht.begin();
}

void loop() {
  delay(2000);
  ++counter;  // Increment the counter variable

  // ping adafruit io a few times to make sure we remain connected
  if(! mqtt.ping(3)) {
    // reconnect to adafruit io
    if(! mqtt.connected())
      connect();
  }

  /////////////////////////////////////////////////////////////////////
  // Photocell Light Intensity loop
  // (Define sensorValue as the pin's analog value)
  // (Eventually, integrate with the counter variable, take samples periodically)
  /////////////////////////////////////////////////////////////////////
  delay(2000);  // Wait a few seconds before each sensor
  digitalWrite(photocellControl, HIGH);    // Switch Sensor On
  digitalWrite(LED0, HIGH);              // Turn on RED Led with sensor
  //Serial.println("Photocell: ON");
  delay(5000);                             // Wait for Sensor to charge
  photocellReading = analogRead(photocellPin);  // Take Sensor Reading
  delay(2000);                             // Wait to read Sensor
  digitalWrite(photocellControl, LOW);     // Switch Sensor Off
  digitalWrite(LED0, LOW);               // Turn off RED Led with sensor
  //Serial.println("Photocell: OFF");

  /////////////////////////////////////////////////////////////////////
  // Soil Moisture Sensor loop
  /////////////////////////////////////////////////////////////////////
  delay(2000);  // Wait a few seconds before each reading
  //digitalWrite(soilMoistureControl, HIGH);  // Switch Sensor On
  digitalWrite(LED0, HIGH);              // Turn on Blue Led with sensor
  //Serial.println("Soil Moisture Sensor: ON");
  delay(5000);                              // Wait for Sensor to charge
  //soilMoistureReading = analogRead(soilMoisturePin);  // Take Sensor Reading
  delay(2000);                              // Wait to read Sensor
  //digitalWrite(soilMoistureControl, LOW);   // Switch Sensor Off
  digitalWrite(LED0, LOW);               // Turn off Blue Led with sensor
  //Serial.println("Soil Moisture Sensor: OFF");

  /////////////////////////////////////////////////////////////////////
  // Digital Temperature loop
  /////////////////////////////////////////////////////////////////////
  delay(2000);
  //Serial.println("Requesting digital temperature sensor information...");
  digitalWrite(LED0, HIGH);              // Turn on Blue Led with sensor
  sensors.requestTemperatures(); // Send the command to get temperatures
  soilTempReading = sensors.getTempCByIndex(0);
  delay(2000);
  digitalWrite(LED0, LOW);

  /////////////////////////////////////////////////////////////////////
  // DHT11 Humidity and Temperature loop
  /////////////////////////////////////////////////////////////////////
  delay(2000);  // Wait a few seconds before each reading
  //Serial.println("Requesting DHT11 sensor information...");
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  digitalWrite(LED0, HIGH);              // Turn on Blue Led with sensor
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);
  delay(2000);
  digitalWrite(LED0, LOW);  // Turn off RED Led with sensor

  /////////////////////////////////////////////////////////////////////
  // OUTPUT: print system information
  /////////////////////////////////////////////////////////////////////
  // Single Output Line (Parsed in excel)
  delay(100);  // Wait a few seconds before printing outputs
  digitalWrite(LED0, HIGH);         // Turn LED 0 ON
  //Serial.println("REDled,BLUEled,GRNled,Light,Moisture,SoilTemp,Humidity,Temperature,HeatIndex,WaterValve,WaterUsed");
  //Serial.println("***********************************************************************************");
  //Serial.print("This is data reading # ");
  //Serial.println("'Counter Variable'");
  //Serial.println("Counter,Light,Moisture,SoilTemp,Humidity,Temperature,HeatIndex,WaterValve,WaterUsed");
  Serial.print(counter);                      Serial.print(",");  // Measurement #
  Serial.print(photocellReading);             Serial.print(",");  // Light intensity
  //Serial.print(soilMoistureReading);          Serial.print(",");  // Soil Moisture level
  Serial.print(sensors.getTempCByIndex(0));   Serial.print(",");  // Temperature (Digital temp)
  Serial.print(h);                            Serial.print(",");  // Humidity level
  Serial.print(t);                            Serial.print(",");  // Temperature (C)
  Serial.print(f);                            Serial.print(",");  // Temperature (F)
  Serial.print(hic);                          Serial.print(",");  // Heat Index (C)
  Serial.println(hif);                                            // Heat Index (F)
  //Serial.println("***********************************************************************************");
  // Flash LED's to signal 1 full measurement cycle
  delay(100); digitalWrite(LED0, LOW);        // Turn Led 0 OFF
  delay(100); digitalWrite(LED1, HIGH);       // Turn Led 1 ON
  delay(100); digitalWrite(LED1, LOW);        // Turn Led 1 OFF
  delay(100); digitalWrite(LED2, HIGH);       // Turn Led 2 ON
  delay(100); digitalWrite(LED2, LOW);        // Turn Led 2 OFF

  delay(100); digitalWrite(LED0, HIGH);       // Turn Led 0 ON
  delay(100); digitalWrite(LED0, LOW);        // Turn Led 0 OFF
  delay(100); digitalWrite(LED1, HIGH);       // Turn Led 1 ON
  delay(100); digitalWrite(LED1, LOW);        // Turn Led 1 OFF
  delay(100); digitalWrite(LED2, HIGH);       // Turn Led 2 ON
  delay(100); digitalWrite(LED2, LOW);        // Turn Led 2 OFF

  // Publish data
  if (! light_intensity.publish(photocellReading))
    Serial.println(F("Failed to publish temperature"));
  else
    Serial.println(F("Temperature published!"));

  /*if (! soil_moisture.publish(soilMoistureReading))
    Serial.println(F("Failed to publish humidity"));
  else
    Serial.println(F("Humidity published!"));
    */

  if (! soil_temp.publish(soilTempReading))
    Serial.println(F("Failed to publish humidity"));
  else
    Serial.println(F("Humidity published!"));

  if (! humidity.publish(h))
    Serial.println(F("Failed to publish humidity"));
  else
    Serial.println(F("Humidity published!"));

  if (! temp_celsius.publish(t))
    Serial.println(F("Failed to publish humidity"));
  else
    Serial.println(F("Humidity published!"));

  if (! temp_fahrenheit.publish(f))
    Serial.println(F("Failed to publish humidity"));
  else
    Serial.println(F("Humidity published!"));

  if (! heat_index_c.publish(hic))
    Serial.println(F("Failed to publish humidity"));
  else
    Serial.println(F("Humidity published!"));

  if (! heat_index_f.publish(hif))
    Serial.println(F("Failed to publish humidity"));
  else
    Serial.println(F("Humidity published!"));
}

// connect to adafruit io via MQTT
void connect() {
  Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }
    if(ret >= 0)
      mqtt.disconnect();
    Serial.println(F("Retrying connection..."));
    delay(5000);
  }
  Serial.println(F("Adafruit IO Connected!"));
}

