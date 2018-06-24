#include <Wire.h>
#include <ADXL345.h>
#include "SparkFunTMP102.h"

// Threshold definitions
#define AMBIENTTEMPTHRESHOLDMAX 90.0
// Address defintions of i2c sensors

#define AMBIENTTEMPSENSORADDRESS 0x48    // TMP102 I2C address

// Time definitions
#define CALIBRATIONTIME 15000

// Software Serial pin definitions
#define rxPin 6
#define txPin 2

// Chip select pin definitions
#define chipSelectRecieve 11
#define BAUDRATE 115200

// Initialize sensor objects
ADXL345 accel(ADXL345_STD);
TMP102 sensor0(0x48);

/* 
 *  Smoothing data values 
 */
const int numberOfReadings = 5;
float readingsAmbientTemp[numberOfReadings];

// final smoothened value
float smoothenedAmbientTemp = 0.0;

// totals for each for calculating smoothing
float totalAmbientTemp = 0.0; 

int readIndex = 0; //the read index for the number to iterate

int state = 1; //initially set state to IDLE
float A = 0.0; 
float ambientTemp = 0.0; //read in value for ambient temp

void setup() {
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  
  pinMode(chipSelectRecieve, INPUT);
  
  Serial.begin(BAUDRATE);
  Wire.begin();
  byte deviceID = accel.readDeviceID();
  sensor0.begin();
  sensor0.wakeup();
    /*
   * Initialize all values in each Array for the smoothing to 0
   */
   for(int thisReading = 0; thisReading < numberOfReadings; thisReading++){
      readingsAmbientTemp[thisReading] = 0.0;
   }
}


void loop() 
{

  
  unsigned long currentMillis = millis(); //timer
  A = String(accel.getY(),2).toFloat();
  ambientTemp = sensor0.readTempC();
  

  /*
   * Determining state for IDLE and READY determining whether the pod has passed the calibration period time (determined in milliseconds by CALIBRATIONTIME)
   * Also determined if each sensor can be read.
   */
  if (currentMillis > CALIBRATIONTIME) // On success, read() will return 1, on fail 0.
  {
    if(state != 0){ //if it hasn't set the state to error previously
      state = 2; //Set the state to READY
    }
  }
  else{
    if(state != 0){
      state = 1; //State set to IDLE if it doesn't read or timer isn't above the Calibration Period
    }
  }

  //--------------[ Smoothing Data ]------------------//

  //Set total to minus the read index for each sensor to make up for end data once read index is reset
  totalAmbientTemp = totalAmbientTemp - readingsAmbientTemp[readIndex];
  
  // read converted float data to array of each @ read index
  readingsAmbientTemp[readIndex] = ambientTemp;
  
  // add the reading to the total:
  totalAmbientTemp = totalAmbientTemp + readingsAmbientTemp[readIndex];
  
  // advance to the next position in the array:
  readIndex = readIndex + 1;
  // if we're at the end of the array...
  if (readIndex >= numberOfReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average for each of the sesnsors to get the smoothened value
  smoothenedAmbientTemp = totalAmbientTemp / numberOfReadings;

  //--------------/ End Smoothing Data /------------------//


  //--------------[ Ensure Data in Range ]------------------//
  /*  Check if the Temperatures or Pressure are not within the threshold and set the state to 0 if it isn't
   */
  if ((!isAmbientTempWithinRange(smoothenedAmbientTemp))){
    
    state = 0; // Set state to FAULT
    if(digitalRead(chipSelectRecieve) == HIGH){
      
      sendData(smoothenedAmbientTemp, A, state );
      
      while(digitalRead(chipSelectRecieve) == HIGH){
        //do nothing
      }
    
    } 
  }
  
   //--------------/ End Ensure Data in Range /------------------//


  /*
   * Reads if the pin for chip selection is set to HIGH and if so, sends data as requested.
   * This prevents over-reading in network with multiple Teensys and Flip and Clicks.
   */
  if(digitalRead(chipSelectRecieve) == HIGH){
    
  sendData(smoothenedAmbientTemp, A, state );
    while(digitalRead(chipSelectRecieve) == HIGH){
      //do nothing
    }
  }
  delay(555);
  
}
/* sendData
 *  @description 
 *          Sends the data over software serial to the Arduino Mega / Uno
 *  @param ambientTemp
 *          The ambient temperature from the Thermo 3 click sensor
 *  @param pressure
 *          The pressure value from the Barometer click sensor
 *  @param thermocoupleTemp
 *          The temperature from the thermocouple THERMO click sensor
 */
void sendData( float ambientTemp, float A, int state){
  String ambientTempString = String(ambientTemp,2);
  String AccelerationString = String(A,2);
  String stateString = String(state);
  Serial.print( ambientTempString + "," + AccelerationString + "," + stateString + "]");
}


  boolean isAmbientTempWithinRange(float ambientTemp){
  if ( (ambientTemp < AMBIENTTEMPTHRESHOLDMAX) ){ 
    return true;
  }
  else{
    return false;
  }
}

