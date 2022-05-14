// SendAntennaPosition - will read antenna position AZ/EL and send it to Xbee
#include <Wire.h>
//Compass library and initialization
#include "bmm150.h"
#include "bmm150_defs.h"
BMM150 bmm = BMM150();
float heading = 0;
float HeadingToDisplay = 0.0;

// Library for accelerometer
#include "LIS3DHTR.h"
LIS3DHTR<TwoWire> LIS; //IIC
#define WIRE Wire

// Library for LCD panel
#include "rgb_lcd.h"
rgb_lcd lcd;

// Define initial LCD colors
int colorR = 255;
int colorG = 0;
int colorB = 0;

const int ElevPin = A0;
int reading;
String ElString = "";
//---------------------------------------------------------------
void setup() 
{
  //Compass initialize

    if (bmm.initialize() == BMM150_E_ID_NOT_CONFORM) 
    {
      Serial.println("Chip ID can not read!");
      while (1);
    } 
    else 
      {
        Serial.println("Initialize done!");
      }
    
// Setup accelerometer
  LIS.begin(WIRE, LIS3DHTR_ADDRESS_UPDATED); //IIC init
  delay(100);
  LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);

//  analogReference(EXTERNAL); //External reference improves tilt accuracy
  // Initialize console port
  Serial.begin(9600); 
//  while(!Serial()); //waits for serial to be available
  Serial.println ("Starting up");

  //Initialize Xbee
  Serial1.begin(9600); // Xbee
//  while(!Serial1()); 
  Serial.println ("Xbee initialized");
  lcd.begin(16, 2);

// Initrialize compass
   Wire.begin(); // Start the I2C bus
//   compass = HMC5883L();
//   compass.SetScale(1.3);
//   compass.SetMeasurementMode(Measurement_Continuous);

}// setup
//-----------------------------------------------------------
//-----------------------------------------------------------
void displayHeading(float HeadingToDisplay)
{
  colorR = 0;
  colorG = 0;
  colorB = 255;
  lcd.setRGB(colorR, colorG, colorB);
  delay(500);
  lcd.setCursor(0, 0);
  lcd.print("Hdg="+String(HeadingToDisplay));
  lcd.setCursor(0, 1);
  lcd.print("Elev="+ElString);
}

//------------------------------------------------------------
void ReadCompass()
{
    bmm150_mag_data value;
    bmm.read_mag_data();

    value.x = bmm.raw_mag_data.raw_datax;
    value.y = bmm.raw_mag_data.raw_datay;
    value.z = bmm.raw_mag_data.raw_dataz;

    float xyHeading = atan2(value.x, value.y);
    float zxHeading = atan2(value.z, value.x);
    heading = xyHeading;

    if (heading < 0) {
        heading += 2 * PI;
    }
    if (heading > 2 * PI) {
        heading -= 2 * PI;
    }
    float headingDegrees = heading * 180 / M_PI;
    float xyHeadingDegrees = xyHeading * 180 / M_PI;
    float zxHeadingDegrees = zxHeading * 180 / M_PI;
    HeadingToDisplay = headingDegrees + 13; //Add declination

    Serial.print("Heading: ");
    Serial.println(headingDegrees);
} //ReadCompass

//--------------------------------------------
void readI2C_elevation ()
{
    if (!LIS)
  {
    Serial.println("LIS3DHTR didn't connect.");
    while (1)
      ;
    return;
  }
   //3 axis
   float elevReading = LIS.getAccelerationX() * 100.0;
   Serial.print("x:"); Serial.print(LIS.getAccelerationX()); Serial.print("  ");
   ElString = String(map(elevReading,0,100,0,90));
   Serial.println(ElString);
//  if (ElString.length() == 1) 
//  {
//    ElString = "0" + ElString; // pad to 2 characters
//  }
   
   //Serial.print("y:"); Serial.print(LIS.getAccelerationY()); Serial.print("  ");
   //Serial.print("z:"); Serial.println(LIS.getAccelerationZ());
}
//------------------------------------------------------------
//void ReadEl() 
//{
//  // Read the elevation pin
//  reading = analogRead (A0);
//  Serial.println ("EL="+String(reading)+" "+String(map(reading,510,613,0,90)));
//  //Serial.println ("EL="+(map(reading,413,613,-90,90)));
//  ElString = (String(map(reading,510,613,0,90)));
//  if (ElString.length() == 1) 
//  {
//    ElString = "0" + ElString; // pad to 2 characters
//  }
//  Serial1.println (ElString);
//}
//--------------------------------------------------------
void loop ()
{
  //ReadEl();
  readI2C_elevation();
  ReadCompass();
  displayHeading(HeadingToDisplay);
  delay (500);
}
