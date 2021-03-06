/*************************************************************************************
*   Project:      Automated Garden Monitor
*   Author:       Oliver Salmeron
*   Date Began:   May, 2016
*   
*   Description:  This program monitors and controls the parameters of a garden space. 
*                 All inputs and outputs are modular by design and can be selected as desired.
*   
*   AIO Username:   oligon
*   AIO Key:        9b9ec3768ccd4ebf83a67d2223e6e2b8
*   
*   Hardware Hook Up:
*     - Indicator LED
*       -- Digital Pin 2 -> 1k Resistor -> LED -> GND (Series)
*       -- LED: Positive Lead is the longer lead
*     - DHT11 Humidity and Temperature Sensor
*       -- 3.3V               -   pin 1
*       -- GND                -   pin 4
*       -- Digital Pin 3      -   pin 2
*     - Soil Moisture Sensor
*       -- 3.3V               -   Red Wire
*       -- GND                -   Black Wire
*       -- Analog Pin 0       -   Green Wire
*     - Photocell
*       -- 3.3V               -   Either End
*       -- Analog Pin 1       -   Opposite End + 5kOhm Pull Down Resistor (to GND)
*     - Digital Temperature Sensor
*       -- ??
*     - Water Flow Sensor
*       -- 5V Power Source    -   Red Wire
*       -- Digital Pin 4      -   Yellow Wire
*       -- GND                -   Black Wire
*     - Water Solenoid Valve
*       -- Digital Pin 5      -   Transistor: 12V -> Both Terminals
*     
*   Wireless Communication:
*     - Bluetooth (Feather)
*       -- Eddystone URL:   https://freeboard.io/board/Rqzu2j
*       -- Description:     The bluetooth nRF51 chip broadcasts the link above to any nearby devices. This website is where all the information is displayed visually.
*     - WiFi (HUZZAH)
*       -- IP Address:      
*       -- SSID:            
*       -- Password:        
*       
*   Steps:
*     1. Get Parts
*     2. Set Up sensors individually
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
*   - Inputs (Settings and Sensors)
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
*         |   Humidity        -- %                        |
*         |   Temperature     -- Fahrenheit               |         
*         |   Heat Index      -- Fahrenheit               |
*         |   Soil Moisture   -- ?                        |
*         |   Light Intensity -- lux                      |
*         |   Water Flow      -- lps                      |                   
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
*         | Camera            -- FLIR, Webcam                              |
*         | Display Interface -- Joystick, Button and Text                 |                                 
*         | Water pH          -- eTape Chemical Sensor (Need to purchase)  |
*         | Soil pH           -- (Need to Purchase)                        |
*         | pH Level          -- (Need to purchase)                        |
*         | Nutrients         -- (Need to purchase)                        |
*         |                      Foliar Feeding or Injection to Water line |
*  
*   - Outputs (Controls and Displays)
*       Automation        <-- Defined in the Settings
*         | 1. Light Cycle and Intensity Controls
*         |     LED's           -- Light & Temperature Sensors  |
*         | 2. Fan Controls                                     |
*         |     Computer Fan    -- Humidity & Temperature       |
*         | 3. Water Controls                                   |
*         |     Solenoid Valve  -- Soil Moisture                |
*         | 5. [Nutrient Controls]                              |
*         | 6. [pH Controls]                                    |
*       Micro OLED        <-- [System Information]
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
*            - Digital Temp sensor (from amazon)
*            - [Nutrients Portfolio]
*            - [Connect 5 led level display for each sensor (7x = 35 leds)]
*              -- Average range of values
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

#include "DHT.h"                        // DHT11 Sensor Library

// Adafruit Bluefruit Libraries
#include "Adafruit_BLE.h"               // Adafruit Bluetooth Library
#include "Adafruit_BluefruitLE_SPI.h"   // Adafruit Bluefruit I2C Definition
#include "Adafruit_BluefruitLE_UART.h"  // Adafruit Bluefruit UART Definition


#include "BluefruitConfig.h"            // Adafruit Bluefruit Config file


// DEFINITIONS
// DHT11 Humidity and Temperature Sensor Definitions
#define DHTPIN 2          // what digital pin we're connected to
#define DHTTYPE DHT11     // DHT 11 type

DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor.

// DHT11 pinout
// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Soil Moisture Sensor Definitions
int sensorPin1 = A0;      // sensorPin1 <-- Soil Moisture (Analog pin A0)

int thresholdUp = 400;    // High threshold that is used in the Water Cycle
int thresholdDown = 250;  // Low threshold that is used in the Water Cycle

// Photocell Light Intensity Definitions
int photocellPin = 1;     // the cell and 10K pulldown are connected to a1
int photocellReading;     // the analog reading from the sensor divider

// LED Definitions
int LEDpin = 3;           // connect Red LED to Digital pin 3 (PWM pin)
int LEDbrightness;        // Used when controlling brightness

// Water Flow Sensor Definitions
#define FLOWSENSORPIN 4  // Digital Pin 4 <-- Flow Sensor

// count how many pulses!
volatile uint16_t pulses = 0;
// track the state of the pulse pin
volatile uint8_t lastflowpinstate;
// you can try to keep time of how long it is between pulses
volatile uint32_t lastflowratetimer = 0;
// and use that to calculate a flow rate
volatile float flowrate;
// Interrupt is called once a millisecond, looks for any pulses from the sensor!
SIGNAL(TIMER0_COMPA_vect) {
  uint8_t x = digitalRead(FLOWSENSORPIN);
  
  if (x == lastflowpinstate) {
    lastflowratetimer++;
    return; // nothing changed!
  }
  
  if (x == HIGH) {
    //low to high transition!
    pulses++;
  }
  lastflowpinstate = x;
  flowrate = 1000.0;
  flowrate /= lastflowratetimer;  // in hertz
  lastflowratetimer = 0;
}

// Create useInterrupt function tool for Water Flow Sensor
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
  }
}


void setup(void)
{
  while (!Serial);          // required for Flora & Micro
  delay(500);               // Wait 0.5 sec before checking again

  Serial.begin(115200);     // Set the Baud Rate
  // System Information
  Serial.println(F("Automated Garden Code"));
  Serial.println(F("------------------------------------"));
  delay(500);               // wait for display to boot up

/*
  pinMode(FLOWSENSORPIN, INPUT);                  // Define digital pin 4 as input
  digitalWrite(FLOWSENSORPIN, HIGH);              // Set digital pin 4 to high
  lastflowpinstate = digitalRead(FLOWSENSORPIN);  // Set the read digital value to lastflowpinstate
  useInterrupt(true);
  
  dht.begin();
*/
}

void loop(void)
{
 
  // Wait a few seconds between measurements.
  delay(2000);
  // Add the reading # and timestamp to each reading measured

  /////////////////////////////////////////////////////////////////////
  // Reading # and Timestamp loop
  /////////////////////////////////////////////////////////////////////
  // Add the reading # and timestamp to each measurement recorded
  int reading;
  int i;
  String timestamp;
  String ts;
  
  timestamp = ("May 3, 2015 ");
  ts = timestamp;
     
  i = 1;
  reading = reading + i;
  
  Serial.print(reading);
  Serial.print("\t");
  Serial.println(ts);
  
  /////////////////////////////////////////////////////////////////////
  // Soil Moisture Sensor loop
  /////////////////////////////////////////////////////////////////////
  // Define sensorValue as the pin's analog value.
  int sensorValue1;
  sensorValue1 = analogRead(sensorPin1);
  
  // Print the Soil Moisture Level info.
  Serial.print("Soil Moisture Level: ");
  Serial.print(sensorValue1);
  Serial.print("\t");
  
  /////////////////////////////////////////////////////////////////////
  // Photocell Light Intensity loop
  /////////////////////////////////////////////////////////////////////
  photocellReading = analogRead(photocellPin);  
 
  // Print the raw analog light intensity value
  Serial.print("| Analog light reading: ");
  Serial.print(photocellReading);     // the raw analog reading
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
  Serial.println("\t| ");
  // Convert to Lux
  // ??????????????
/*  // Control the brightness of an LED in response to photocell
  // LED gets brighter the darker it is at the sensor
  // that means we have to -invert- the reading from 0-1023 back to 1023-0
  photocellReading = 1023 - photocellReading;
  //now we have to map 0-1023 to 0-255 since thats the range analogWrite uses
  LEDbrightness = map(photocellReading, 0, 1023, 0, 255);
  analogWrite(LEDpin, LEDbrightness);
*/
 
  /////////////////////////////////////////////////////////////////////
  // DHT11 Humidity and Temperature loop
  /////////////////////////////////////////////////////////////////////
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
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
  
  // Print Humidity/Temp Info
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("| Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print(f);
  Serial.print(" *F");
  Serial.print("| Heat index: ");
  Serial.print(hic);
  Serial.print(" *C ");
  Serial.print(hif);
  Serial.println(" *F");
  
  /////////////////////////////////////////////////////////////////////
  // Water Flow sensor loop
  /////////////////////////////////////////////////////////////////////
  // Print Water Flow sensor info
  Serial.print("Freq: "); Serial.print(flowrate); Serial.print(" \t\t");
  Serial.print("| Pulses: "); Serial.print(pulses, DEC); Serial.print(" \t\t\t");
  
  // if a plastic sensor use the following calculation
  // Sensor Frequency (Hz) = 7.5 * Q (Liters/min)
  // Liters = Q * time elapsed (seconds) / 60 (seconds/minute)
  // Liters = (Frequency (Pulses/second) / 7.5) * time elapsed (seconds) / 60
  // Liters = Pulses / (7.5 * 60)
  float liters = pulses;
  liters /= 7.5;
  liters /= 60.0;

  // Print the amount of liquid passed in liters
  Serial.print("| Liters: "); Serial.println(liters);
  
  /////////////////////////////////////////////////////////////////////
  // Light Intensity sensor loop
  /////////////////////////////////////////////////////////////////////
  
  /////////////////////////////////////////////////////////////////////
  // Water Resevoir Level sensor loop
  /////////////////////////////////////////////////////////////////////
  
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
