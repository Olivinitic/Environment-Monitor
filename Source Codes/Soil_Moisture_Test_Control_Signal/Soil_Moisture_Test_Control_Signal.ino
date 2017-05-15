/*******************************************************************
*  Sensor:       Sparkfun's Soil Moisture Sensor
*  Author:       Sparkfun
*  Contributer:  Oliver S
*  Description:  This is an example sketch to test the soil moisture sensor.
*
*  To-do:        Add if/else statements to integrate watering schedule
*                
*******************************************************************/

// DEFINITIONS
  // Soil Moisture Sensor DEFINITIONS
  int soilMoisturePin = A0;           // soilMoisturePin     <-- Analog 0 Data Signal 
  int soilMoistureControl = 4;      // soilMoistureControl <-- Digital 4 Control Signal
  
  int thresholdUp = 400;              // High threshold that is used in the Water Cycle
  int thresholdDown = 250;            // Low threshold that is used in the Water Cycle
  
// SETUP
void setup()
  {
    //mySerial.begin(9600);           // set up serial port for 9600 baud (speed)
    Serial.begin(115200);             // Set the Baud Rate
    delay(500);                       // wait for display to boot up
    
    // System Information
    Serial.println(F("Soil Moisture Code Test! No Control Signal"));
    Serial.println(F(""));
    delay(500);                       // wait for display to boot up
    
    // Soil Moisture setup
    pinMode(soilMoistureControl, OUTPUT);           // Sets Digital 4 pin as Output
}

// MAIN CODE
void loop() 
  {
      /////////////////////////////////////////////////////////////////////
      // Soil Moisture Sensor loop
      /////////////////////////////////////////////////////////////////////
      // Define sensorValue as the pin's analog value.
      int soilMoistureReading;
      digitalWrite(soilMoistureControl, HIGH);            // Switch Sensor On
      delay(1000);                                       // Wait for Sensor to charge
      soilMoistureReading = analogRead(soilMoisturePin);    // Take Sensor Reading
      delay(500);                                          // Wait to read Sensor
      digitalWrite(soilMoistureControl, LOW);             // Switch Sensor Off
      delay(1000);   
      
      // Print the Soil Moisture Level info.
      Serial.print("Soil Moisture Level: ");                // Print the header
      Serial.print(soilMoistureReading);                    // Print the analog reading
      Serial.println("\t");                                 // Advance to next sensor
      
}
