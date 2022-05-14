// SendAntennaPosition - will read antenna position AZ/EL and send it to Xbee
#include <Wire.h>
//Compass library and initialization
#include "bmm150.h"
#include "bmm150_defs.h"
BMM150 bmm = BMM150();
float heading = 0;

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
    
  // put your setup code here, to run once:
  analogReference(EXTERNAL); //External reference improves tilt accuracy
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
   compass = HMC5883L();
   compass.SetScale(1.3);
   compass.SetMeasurementMode(Measurement_Continuous);

}// setup
//-----------------------------------------------------------
//-----------------------------------------------------------
void displayHeading(DisplayHeading)
{
  colorR = 0;
  colorG = 0;
  colorB = 255;
  lcd.setRGB(colorR, colorG, colorB);
  delay(500);
//  lcd.setCursor(0, 0);
//  lcd.print("Hdg="+String(DisplayHeading));
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

    Serial.print("Heading: ");
    Serial.println(headingDegrees);

    delay(1000);
}
//------------------------------------------------------------
void ReadEl() 
{
  // Read the elevation pin
  reading = analogRead (A0);
  Serial.println ("EL="+String(reading)+" "+String(map(reading,510,613,0,90)));
  //Serial.println ("EL="+(map(reading,413,613,-90,90)));
  ElString = (String(map(reading,510,613,0,90)));
  if (ElString.length() == 1) 
  {
    ElString = "0" + ElString; // pad to 2 characters
  }
  Serial1.println (ElString);
  delay (1000);
}
//--------------------------------------------------------
void loop ()
{
  ReadEl();
  ReadCompass();
  displayHeading(heading);
}
