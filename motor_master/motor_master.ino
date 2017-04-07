// Lee Truan
// Master control example

#include <Wire.h>
#include "SparkFunIMU.h"
#include "SparkFunLSM303C.h"
#include "LSM303CTypes.h"
#include <XBee.h>
#include <SoftwareSerial.h>

#define slave 0x01

uint8_t mSpeedL; // Set speed (x/255)
uint8_t mSpeedR;

bool mDirL; // false is forward true is reverse
bool mDirR;
int tries;
float headingDegrees;

int delay_;
bool timeM; // USed to debug compass
float headingAdj;
float beacon;
long lastRun;

LSM303C myIMU;

// xBee Startup

//SoftwareSerial xBEE(2, 3); // TX, RX lines
XBee xBee = XBee();
Rx16Response rx16 = Rx16Response();

#define samples 110
int temp, smoothData, rawData;
int timeToScan = 2000;
short currentHeading;

//Variable for i2c comms
uint8_t currHeadingI2c[2];

int resetRSSI = -1000; // Reset RSSI number

uint8_t payload[12];
int payload_size = 4;

//Structure to contain the readings from the beacon
struct{
  float heading;
  int signalStrength;
} readings[samples];

//Union for converting between byte[4] and float
union{
  float f;
  uint8_t b[4];
} heading_converter;
// end of xBee

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  pinMode(13, OUTPUT);
  pinMode(A0, INPUT);
  Serial.begin(9600);

  Serial1.begin(57600);
  xBee.setSerial(Serial1);

  // See documentation for more information on IMU instructions.
  if (myIMU.begin(
        MODE_I2C,
        MAG_DO_40_Hz,
        MAG_FS_16_Ga,
        MAG_BDU_ENABLE,
        MAG_OMXY_HIGH_PERFORMANCE,
        MAG_OMZ_HIGH_PERFORMANCE,
        MAG_MD_CONTINUOUS,
        ACC_FS_2g,
        ACC_BDU_ENABLE,
        ACC_X_ENABLE | ACC_Y_ENABLE | ACC_Z_ENABLE,
        ACC_ODR_100_Hz
      ) != IMU_SUCCESS)
  {
    Serial.println(F("Failed setup."));
    while (1);
  }

  mSpeedL = 100; // Set speed (x/255)
  mSpeedR = 100;
  
  mDirL = true; // false is forward true is reverse
  mDirR = false;
  tries = 0;
  headingDegrees = 0;
  
  delay_ = 250;
  timeM = false; // USed to debug compass
  headingAdj = 0;
  beacon = 0;

  delay(1000);
  //Serial.end();

  lastRun = millis();
}

/*
   The binary should be read RIGHT TO LEFT!
   Bit 1 = Motor 1 direction 0 foreward, 1 reverse
   Bit 2-4 = Motor 1 speed
   Bit 5 = Motor 2 direction 0 foreward, 1 reverse
   Bit 6-8 = Motor 2 speed
*/
void loop()
{
  long timer = millis();
  if (timer - lastRun > delay_)
  {
    if (timeM)
    {
      long time_ = millis();
      readCompass(4);
      Serial.println((String)F("TIME MS: ") + (String)(double)(millis() - time_));
    }
    else
      readCompass(4);

    Serial.println(beacon);
     
    if (analogRead(A0) < 200)
    {
      float deg = headingDegrees + headingAdj;
      deg -= beacon;
      deg += (deg < 0) ? 360: 0;
      if (deg > 10)
        contactSlave(0b01101110);
      else if (deg < -10)
        contactSlave(0b11100110);
      else
        contactSlave(0b11101110);
    }
    else if (analogRead(A1) < 20 && analogRead(A2) < 20 && tries < 4)
    {
      tries++;
      contactSlave(0b11111111);
    }
    else if (analogRead(A2) < 20)
    {
      tries = 0;
      contactSlave(0b11111111);
      delay(1000);
      contactSlave(0b11111110);
    }
    else
    {
      tries = 0;
      contactSlave(0b11101111);
      delay(1000);
    }
    lastRun = millis();
  }
}



void readCompass(int item)
{
  switch (item)
  {
      float value;
      float heading;
    case 0:
      value = myIMU.readAccelX();
      // Assume that if X is not activated then none are (poor assumption, but demo)
      if ( !isnan(value) )
      {
        Serial.print("\nAccelerometer:\n X = ");
        Serial.println(value, 4);
        Serial.print(" Y = ");
        Serial.println(myIMU.readAccelY(), 4);
        Serial.print(" Z = ");
        Serial.println(myIMU.readAccelZ(), 4);
      }
      break;
    case 1:
      value = myIMU.readGyroX();
      // Not supported by hardware, so will return NAN
      if ( !isnan(value) )
      {
        Serial.print(F("\nGyroscope:\n X = "));
        Serial.println(value, 4);
        Serial.print(" Y = ");
        Serial.println(myIMU.readGyroY(), 4);
        Serial.print(" Z = ");
        Serial.println(myIMU.readGyroZ(), 4);
      }
      break;
    case 2:
      value = myIMU.readMagX();
      if ( !isnan(value) )
      {
        Serial.print("\nMagnetometer:\n X = ");
        Serial.println(value, 4);
        Serial.print(" Y = ");
        Serial.println(myIMU.readMagY(), 4);
        Serial.print(" Z = ");
        Serial.println(myIMU.readMagZ(), 4);
      }
      break;
    case 3:
      value = myIMU.readTempC();
      if ( !isnan(value) )
      {
        Serial.print("\nThermometer:\n");
        Serial.print(" Degrees C = ");
        Serial.println(value, 4);
        Serial.print(" Degrees F = ");
        Serial.println(myIMU.readTempF(), 4);
      }
      break;
    case 4:
      heading = atan2(myIMU.readMagY(), myIMU.readMagX());
      if (heading < 0)
        heading += 2 * PI;
      headingDegrees = heading * 180 / M_PI;
      Serial.print("Theta: ");
      Serial.println(headingDegrees);
      break;
    default:
      Serial.println(F("Failed to enter in a vaild option."));
  }

  if (millis() % 5200 == 0)
    processBeacon();
}

void contactSlave(int data)
{
    Wire.beginTransmission(slave);
    Wire.write(data);
    Wire.endTransmission();
}

void Retrieve(int i){
  xBee.readPacket(10);    //Wait 50 to receive packet
  if (xBee.getResponse().isAvailable())     //Execute only if packet found
  {
    if (xBee.getResponse().getApiId() == RX_16_RESPONSE) 
    {
      xBee.getResponse().getRx16Response(rx16);
      //Store the transmitted data and RSSI
      for(int i = 0; i<4; i++) heading_converter.b[i] = rx16.getData(i);
      int currentRSSI = -rx16.getRssi();
      //Write to array
      readings[i].heading = heading_converter.f;
      readings[i].signalStrength = currentRSSI;
    }
  }else{
    readings[i].heading = 0;
    readings[i].signalStrength = resetRSSI;
    //Serial.println("Beacon problem");
  }
}

          


//Creates a heading through averaging the readings
int ProcessData(){
  int maxRSSI;
  unsigned long maxIndex = 0;
  maxRSSI = readings[0].signalStrength;
  
  //Find max RSSI value
  for (int i=1; i < samples; i++) {
    if (maxRSSI < readings[i].signalStrength) {
      maxRSSI = readings[i].signalStrength;
      maxIndex = i;
    }
  }
  //If there is no valid data
  if(maxRSSI == resetRSSI){
    return -1;
  }

  float headingx = 0;
  float headingy = 0;
  for(int i = 0; i < samples; i++)
  {
    if (readings[i].signalStrength == -1000 && readings[i].heading == 0)
    {
       Serial.println("this heading not included");
    }
    else
    {
      Serial.print(readings[i].heading);
      Serial.print("\t");
      Serial.println(readings[i].signalStrength);
      // Set magnitude of vector by signal strength
      headingx += readings[i].signalStrength * cos(readings[i].heading * PI / 180);
      headingy += readings[i].signalStrength * sin(readings[i].heading * PI / 180);
    }
  }
  
  float heading1 = atan2(headingy, headingx);
  if (heading1 < 0) heading1 += 2 * PI;
  heading1 = heading1 * 180 / PI;

  return (int) heading1;
}

void processBeacon()
{
  for(int i = 0;i<samples;i++){
    Retrieve(i);
    float propComplete = ((float)i)/(float)samples;
    delay(timeToScan/samples);
  }

  beacon = (ProcessData());
}

