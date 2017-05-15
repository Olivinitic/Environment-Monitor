/*************************************************************************************
*   Project:      Automated Garden Monitor
*   Author:       Oliver Salmeron
*   Date Began:   May, 2016
*   Current Date: December 11th, 2016
*   Version:      1.1
*
*   Website:      https://io.adafruit.com/
*
*   Description:  This program monitors and controls the parameters of a garden space.
*                 All inputs and outputs are modular by design and can be selected as desired.
*
*   Goals:        Implement a modular, wireless sensor network to monitor plant health.
*                 Central units, Wifi and Bluetooth enabled.
*                 Peripheral units, Sensor ports, bluetooth paired to center.
*                 Wifi connects to data server (arduino.io) and sends information.
*                 Bluetooth also broadcasts the website link (physical web).
*                 Excel SPreadsheet to Parse Data.
*
*   Arduino.io Information:
*     AIO Username:   oligon
*     AIO Key:        9b9ec3768ccd4ebf83a67d2223e6e2b8
*
*   Wireless Communication:
*     - WiFi (HUZZAH)
*       -- IP Address:      192.168.0.2
*       -- SSID:            CenturyLink0141
*       -- Password:        ae7nad6ecabapy
*       -- Description:     Connects the project to the internet.
*                           All sensors display information to io.adafruit.com
*     - Bluetooth (Feather)
*       -- Eddystone URL:   https://io.adafruit.com/oligon/garden-monitor
*                           https://freeboard.io/board/Rqzu2j (alternative)
*       -- Description:     The bluetooth nRF51 chip broadcasts the link above to any nearby devices.
*                           This website is where all the information is displayed visually.
*
*   Hardware Hook Up:
*     - Indicator LED 0
*       -- Digital Pin 9 -> 220 ohm resistor -> LED -> GND (Series)
*       -- LED: Positive Lead is the longer lead (Anode)
*     - Indicator LED 1
*       -- Digital Pin 10 -> 220 ohm resistor -> LED -> GND (Series)
*       -- LED: Positive Lead is the longer lead (Anode)
*     - Indicator LED 2
*       -- Digital Pin 11 -> 220 ohm resistor -> LED -> GND (Series)
*       -- LED: Positive Lead is the longer lead (Anode)
*     - Photocell
*       -- 3.3V/5V            -   Either End
*       -- Analog Pin 1       -   Opposite End + 5kOhm Pull Down Resistor (to GND)
*     - Soil Moisture Sensor
*       -- 3.3V/5V            -   Red Wire
*       -- Analog Pin 0       -   Green Wire
*       -- GND                -   Black Wire
*     - Digital Temperature Sensor
*       -- 5V Power Source
*       -- Digital Pin 13     -   10 K ohm resistor Pull up resistor to 5V
*       -- GND
*     - DHT11 Humidity and Temperature Sensor
*       -- 3.3V/5V            -   pin 1
*       -- Digital Pin 3      -   pin 2, 10K Pull up resistor
*       -- GND                -   pin 4
*     [STILL NEEDED]
*     - Water Flow Sensor
*       -- 5V Power Source    -   Red Wire
*       -- Digital Pin 4      -   Yellow Wire
*       -- GND                -   Black Wire
*     - Water Solenoid Valve
*       -- 12V                -
*       -- Digital Pin 5      -   Transistor: 12V -> Both Terminals
*     - Huzzah ESP8266 (TX/RX only, UART?)
*       -- Huzzah TX          -   Arduino RX
*       -- Huzzah RX          -   Arduino TX
*     - Feather Bluefruit (TX/RX)
*
*   Steps:
*     1. XXGet Parts
*     2. XXSet Up sensors individually (Still need waterflow sensor and valve)
*     3. Set Up sensors under 1 code
*     4. Enclose
*     5. Make a projected Business Plan
*       - Executive Summary
*         -- Expected Revenues, Expenses, Profits (5 years)
*         -- Funding needed
*       - Company Overview
*       - Industry Analysis
*         -- Market Overview
*         -- Relevant Market Size
*       - Customer Analysis
*         -- Target Customers
*         -- Customer Needs
*       - Competitive Analysis
*         -- Direct Competitors
*         -- Indirect Competitors
*         -- Competitive Advantages
*       - Marketing Plan
*         -- Products & Services
*         -- Pricing
*         -- Promotions Plan
*         -- Distributions Plan
*       - Price of Production
*       - Price of Retail
*       -
*
*
*   Psuedoinstructions:
*   - INPUTS (Settings and Sensors)
*       Config/Settings   <-- Presets or User "Recipes"
*                               Defines the Parameters (# of plants, strains, area)
*                               Controls the Outputs and Automation
*       Documentation     <-- Create a new portfolio or Select an existing one
*         | Portfolio Name:       Name                    |
*         | Strain/Plant Name:    Name                    |
*         | Setting:              Name                    |
*         | # of plants:          1                       |
*         | Sensors Connected:    4/5                     |
*         |   Humidity        -- ok                       |
*         |   Temperature     -- ok                       |
*         |   Soil Moisture   -- ok                       |
*         |   Light Intensity -- ok                       |
*         |   Water Flow      -- error, not connected     |
*         | Sensor Data:          Saved to SD card        |
*         |   Current Humidity        -- %                |
*         |   Current Temperature     -- Fahrenheit       |
*         |   Current Heat Index      -- Fahrenheit       |
*         |   Current Soil Moisture   -- ?                |
*         |   Current Light Intensity -- lux              |
*         |   Current Water Flow      -- lps              |
*         |   Water Used      -- Liters or Gallons        |
*         |   Water Tank      -- Liters or Gallons        |
*         | Picture:              Timelapse               |
*         | Graphs:               Select Timeframe        |
*         |   Past 24 Hours                               |
*         |   Past 2 Weeks                                |
*         |   Past Month                                  |
*         |   Full Length                                 |

*       Sensors           <-- Modular and Wireless
*         | Humidity          -- DHT11 Humidity & Temp Sensor              |
*         | Temperature       -- DHT11 Humidity & Temp Sensor              |
*         |                      Amazon Digital Temp Sensor                |
*         | Soil Moisture     -- Sparkfun Soil Moisture Sensor             |
*         | Lights            -- Photocell Diode                           |
*         | Water Flow        -- Adafruit Water Flow Sensor                |
*         | Display Interface -- Joystick, Button and Text inputs          |
*         | Camera            -- FLIR, Webcam (Need to Purchase)           |
*         | Water pH          -- eTape Chemical Sensor (Need to purchase)  |
*         | Soil pH           -- (Need to Purchase)                        |
*         | pH Level          -- (Need to purchase)                        |
*         | Nutrients         -- (Need to purchase)                        |
*         |                      Foliar Feeding or Injection to Water line |
*
*   - OUTPUTS (Controls and Displays)
*       Automation        <-- Defined in the Settings
*         | 1. Water Controls                                   |
*         |     Solenoid Valve  -- Soil Moisture                |
*         | 2. Light Cycle and Intensity Controls (Need to Buy) |
*         |     LED's           -- Light & Temperature Sensors  |
*         | 3. Fan Controls (Need to Buy)                       |
*         |     Computer Fan    -- Humidity & Temperature       |
*         | 4. [Nutrient Controls] (Need to Buy)                |
*         | 5. [pH Controls] (Need to Buy)                      |
*       Micro OLED        <-- [System Information example]
*         | Internet Connection:  Yes (If No, info is saved on SD)                  |
*         | Datalogging:          Yes                                               |
*         | Plant Groups:         5                                                 |
*         |   1. Cannabis                                                           |
*         |   2. Succulents                                                         |
*         |   3. Jalapenos/ Veggies                                                 |
*         |   4. Herbs                                                              |
*         |   5. Houseplants                                                        |
*         | Group 1:              Name (each plant/section has a unique Portfolio)  |
*         | Strain:               Name                                              |
*         | Settings:             Preset 1/ mySettings 1                            |
*         | # of plants:          1                                                 |
*         | Sensors Connected:    4/5 (display errors if any)                       |
*         |   Humidity        -   ok                                                |
*         |   Temperature     -   ok                                                |
*         |   Soil Moisture   -   ok                                                |
*         |   Light Intensity -   ok                                                |
*         |   Water Flow      -   error, not connected                              |
*         | Sensor Data:          Averaged over 30 second intervals                 |
*         |   Humidity        -   %                                                 |
*         |   Temperature     -   Fahrenheit & Heat Index                           |
*         |   Soil Moisture   -                                                     |
*         |   Light Intensity -   lux                                               |
*         |   Water Flow      -   lps                                               |
*       Wireless Hardware <-- Any IoT hardware used in the project
*         | Bluetooth chip:   Connects the Modular Devices, broadcast Website Link  |
*         | WiFi chip:        Connects the System with the Internet                 |
*       Wireless Software <-- Any IoT software used in the project
*         | Arduino.io        - https://arduino.io/
*         | Dweet.io          - https://dweet.io/
*         | Freeboard.io      - https://freeboard.io/board/Rqzu2j
*         | Sparkfun Phant    - https://learn.sparkfun.com/tutorials/pushing-data-to-datasparkfuncom/what-is-phant
*
*       Data Logs         <-- Each Plant/Group has a Portfolio
*         | 1. Plant/Strain Name                                |
*         | 2. Log # (auto check for last log)                  |
*         | 3. Date                                             |
*         | 4. Setting Name                                     |
*         | 5. Picture                                          |
*         | 6. Data (last 24 hrs, 1 month, beggining of time)   |
*         | 7. Notes                                            |
*
*******************************************************************
*  To-do:  Add User Input for initial setup Configuration
*          Add Inputs (Sensors and Settings)
*            - Presets [Nutrients Portfolio]
*          Add Outputs (Controls, Communications and Displays)
*            - Light Controls
*              -- Linked with Light Intensity and settings
*              -- Add if/else statements to integrate light schedule
*              -- Preset Cycle Settings
*            - Fan Controls
*              -- Linked with Humidity and Temp
*              -- Add if/else statements to integrate fan schedule
*              -- Preset Ideal Heat Index (humidity & temp) Settings
*            - Water Controls
*              -- Linked with Soil Moisture
*              -- Add if/else statements to integrate watering schedule
*              -- Preset Ideal Water Thresholds Settings
*              -- Adjust for soil
*            - [Nutrient Controls]
*              -- Linked with [Nutrient Sensor]
*              -- Preset Strain Nutrient Recipes
*            - Communication (BLE)
*              -- Sensor Data Transmission
*              -- API
*              -- GUI
*            - Graphs and Displays
*              -- Light Intensity vs Time
*              -- Humidity, Temp, Fan Cycles vs Time
*              -- Soil Moisture vs Time
*              -- Average range of values
*          Update entire code to Feather M0, SAMD21 support
*
*
*       MIT license, check LICENSE for more information
*       All text above, and the splash screen below must be included in
*       any redistribution
*********************************************************************/

/*********************************************************************
* void setup() {
*   // Initialize Input Sensors
*     // Camera
*     // Temperature (Digital Temp)
*     // Humidity (Digital)
*     // Soil Moisture (Analog)
*
*     // Light Sensor (Analog)
*     // Water Level Sensor ()
*     // pH Sensor (Digital)
* }
*
* void loop() {
*   // Read Sensors
*   // Display Sensor Readings (Micro OLED)
*   // Transmit Sensor Readings (Bluetooth)
*   // Output Controls
*     // Light Controls
*     // Water Controls
*     // Fan Controls
* }
*********************************************************************/


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
/*
// Adafruit Bluefruit Libraries
#include "Adafruit_BLE.h"               // Adafruit Bluetooth Library
#include "Adafruit_BluefruitLE_SPI.h"   // Adafruit Bluefruit I2C Definition
#include "Adafruit_BluefruitLE_UART.h"  // Adafruit Bluefruit UART Definition
#include "BluefruitConfig.h"            // Adafruit Bluefruit Config file
*/

// DEFINITIONS
  // COUNTER DEFINITIONS
  int counter = 1;  // Define a counter variable to keep track of # of measurements.
  // LED INDICATOR DEFINITIONS
  int LED0 = 9;             // connect Red LED to Digital pin 9 (PWM pin)
  int LED0brightness;       // Used when controlling brightness
  int LED1 = 10;            // connect Blue LED to Digital pin 10 (PWM pin)
  int LED1brightness;       // Used when controlling brightness (PWM)
  int LED2 = 11;            // connect Green LED to Digital pin 11 (PWM pin)
  int LED2brightness;       // Used when controlling brightness (PWM)
  // PHOTOCELL DEFINITIONS
  int photocellControl = 2;      // Digital 2 Control Signal
  int photocellPin = 0;          // Analog 0 Data Signal
  int photocellReading;          // The analog reading from the sensor divider
  // SOIL MOISTURE SENSOR DEFINITIONS
  int soilMoistureControl = 4;   // Digital 4 Control Signal
  int soilMoisturePin = A1;      // Analog 1 Data Signal
  int soilMoistureReading;       // The analog reading from the sensor itself
  int thresholdUp = 400;         // High threshold that is used in the Water Cycle (Stop Water)
  int thresholdDown = 250;       // Low threshold that is used in the Water Cycle (Start Water)
  // DIGITAL TEMPERATURE DEFINITIONS
  #define ONE_WIRE_BUS 8    // Data wire is plugged into port 2 on the Arduino
  OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
  DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
  // DHT11 HUMIDITY AND TEMPERATURE SENSOR DEFINITIONS
  #define DHTPIN 7          // Digital pin 7
  #define DHTTYPE DHT11     // DHT 11 type (The one I have)
  //#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
  //#define DHTTYPE DHT21   // DHT 21 (AM2301)
  DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor.

void setup(void)
  {
    while (!Serial);  // required for Flora & Micro
    delay(500);       // Wait 0.5 sec before checking again

    Serial.begin(115200);     // Set the Baud Rate

    // System Information
    Serial.println(F("Garden Code Test!"));
    Serial.println(F("Version 1.0"));
    Serial.println();
    // Print the Data String Format referance
    //Serial.println("Measurement,Light(lux),SoilMoisture,SoilTemp(C),Humidity,Temp(C),Temp(F),HeatIndex(C),HeatIndex(F),WaterValve,WaterUsed");
    Serial.println("Measurement,Light(lux),SoilMoisture,SoilTemp(C),Humidity,Temp(C),Temp(F),HeatIndex(C),HeatIndex(F)");
    //Serial.println("-------------------------------------------------------------------------------------------------");
    delay(500);       // wait for display to boot up

    // Inidcator LEDs setup
    pinMode(LED0, OUTPUT);  // Sets Digital Pin 9 as Output
    pinMode(LED1, OUTPUT); // Sets Digital Pin 10 as Output
    pinMode(LED2, OUTPUT);  // Sets Digital Pin 11 as Output
    // Photocell setup
    pinMode(photocellControl, OUTPUT);    // Sets Digital Pin 2 as Output
    // Soil Moisture setup
    pinMode(soilMoistureControl, OUTPUT); // Sets Digital Pin 4 as Output
    // Digital Temp setup
    sensors.begin();
    // DHT11 setup
    dht.begin();
  }

void loop(void)
  {
    delay(2000);
    ++counter;  // Increment the counter variable

    /////////////////////////////////////////////////////////////////////
    // Photocell Light Intensity loop
    // (Define sensorValue as the pin's analog value)
    // (Eventually, implement with the counter variable, take samples periodically)
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
    digitalWrite(soilMoistureControl, HIGH);  // Switch Sensor On
    digitalWrite(LED0, HIGH);              // Turn on Blue Led with sensor
    //Serial.println("Soil Moisture Sensor: ON");
    delay(5000);                              // Wait for Sensor to charge
    soilMoistureReading = analogRead(soilMoisturePin);  // Take Sensor Reading
    delay(2000);                              // Wait to read Sensor
    digitalWrite(soilMoistureControl, LOW);   // Switch Sensor Off
    digitalWrite(LED0, LOW);               // Turn off Blue Led with sensor
    //Serial.println("Soil Moisture Sensor: OFF");

    /////////////////////////////////////////////////////////////////////
    // Digital Temperature loop
    /////////////////////////////////////////////////////////////////////
    delay(2000);
    //Serial.println("Requesting digital temperature sensor information...");
    digitalWrite(LED0, HIGH);              // Turn on Blue Led with sensor
    sensors.requestTemperatures(); // Send the command to get temperatures
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
    // Water Solenoid Valve loop
    /////////////////////////////////////////////////////////////////////
    //delay(2000);  // Wait a few seconds before each reading
    //digitalWrite(solenoidPin, HIGH);  // Switch Solenoid ON
    //digitalWrite(LED0, HIGH);         // Turn on Green Led with Valve
    //Serial.println("Water Solenoid Valve: OPEN");
    //delay(2000);                        // Wait 2 Second
    //digitalWrite(solenoidPin, LOW);   // Switch Solenoid OFF
    //digitalWrite(LED0, LOW);          // Turn on Green Led with Valve
    //Serial.println("Water Solenoid Valve: CLOSED");

    /////////////////////////////////////////////////////////////////////
    // LED Indicators loop
    /////////////////////////////////////////////////////////////////////
    // Red LED Indicator (to show the brightness of the photocell on the LED)
    /*LED0brightness = map(photocellReading, 0, 1023, 0, 255);
    analogWrite(LED0,LED0brightness);

    // Control the brightness of an LED in response to photocell
    // LED gets brighter the darker it is at the sensor
    // that means we have to -invert- the reading from 0-1023 back to 1023-0
    photocellReading = 1023 - photocellReading;
    //now we have to map 0-1023 to 0-255 since thats the range analogWrite uses
    LEDbrightness = map(photocellReading, 0, 1023, 0, 255);
    analogWrite(LEDpin, LEDbrightness);
    */

    // PRINT SYSTEM INFORMATION
    // Single Output Line (Parsed in excel)
    delay(100);  // Wait a few seconds before printing outputs
    digitalWrite(LED0, HIGH);         // Turn on Green Led with Valve
    //Serial.println("REDled,BLUEled,GRNled,Light,Moisture,SoilTemp,Humidity,Temperature,HeatIndex,WaterValve,WaterUsed");
    //Serial.println("***********************************************************************************");
    //Serial.print("This is data reading # ");
    //Serial.println(counter);
    Serial.print(counter);                      Serial.print(",");  // Measurement #
    Serial.print(photocellReading);             Serial.print(",");  // Light intensity
    Serial.print(soilMoistureReading);          Serial.print(",");  // Soil Moisture level
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
/*
    // Print the raw analog light intensity value
    Serial.print("Analog light reading: ");
    Serial.print(photocellReading);                 // Print the raw analog reading
      // We'll have a few threshholds, qualitatively determined
      if (photocellReading < 10) {
        Serial.println(" - Dark");
      } else if (photocellReading < 200) {
        Serial.println(" - Dim");
      } else if (photocellReading < 500) {
        Serial.println(" - Light");
      } else if (photocellReading < 800) {
        Serial.println(" - Bright");
      } else {
        Serial.println(" - Very bright");
      }
    Serial.println("--------------------------------------------");

    // Print the Soil Moisture Level info.
    Serial.print("Soil Moisture Level: ");
    Serial.println(soilMoistureReading);
    Serial.println("--------------------------------------------");

    // Print the Digital Temp info.
    Serial.print("Temperature for the device 1 (index 0) is: ");
    Serial.println(sensors.getTempCByIndex(0));
    Serial.println("--------------------------------------------");

    // Print Humidity/Temp Info
    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.println(" %");
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.println(" *C");
    Serial.print(f);
    Serial.println(" *F");
    Serial.print("Heat index: ");
    Serial.print(hic);
    Serial.println(" *C");
    Serial.print(hif);
    Serial.println(" *F");
    Serial.println("-------------------------------------------------------------------------------------------------");
    */
  }

/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
void getUserInput(char buffer[], uint8_t maxSize)
{
  memset(buffer, 0, maxSize);
  while( Serial.available() == 0 ) {
    delay(1);
  }

  uint8_t count=0;

  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && !(Serial.available() == 0) );
}
