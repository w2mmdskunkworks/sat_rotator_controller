//Antenna rotator controller using relays and Grove connections

// Libraries
#include <Wire.h>
#include "rgb_lcd.h"
rgb_lcd lcd;

// Define debugging code
int DisplayJoystickValues = 0;
int PrintDebugMessages = 1;

// Define initial LCD colors
int colorR = 255;
int colorG = 0;
int colorB = 0;

// Pins for relay output
#define UpPin 4
#define DownPin 5
#define LeftPin 6
#define RightPin 7 

//Define pins for joystick
#define ParkPin 3
int VertPin = A0;
int HorizPin = A1;
// Variables into which to read the joystick value
int xPosition = 0;
int yPosition = 0;
int park = 0;

// Variables
int IsMovingV = 0; //1 if up, -1 if down
int IsMovingH = 0; //1 if right, -1 if left
int IsParking = 0; // 1 if parking; cancelled by joywstick move

void setup() {

// Define relay pin modes
  pinMode (UpPin, OUTPUT);
  pinMode (DownPin, OUTPUT);  
  pinMode (LeftPin, OUTPUT);
  pinMode (RightPin, OUTPUT);
  digitalWrite(UpPin,LOW);
  digitalWrite(DownPin,LOW);
  digitalWrite(LeftPin,LOW);
  digitalWrite(RightPin,LOW);

// Pin modes for joystick
  pinMode(VertPin, INPUT);
  pinMode(HorizPin, INPUT);
  pinMode (ParkPin, INPUT_PULLUP);

// Initialize serial ports
  Serial.begin(9600);

  // initialize the LCD
  // Blink red for 1 second, then white
  lcd.begin(16, 2);
  colorR = 255;
  colorG = 0;
  colorB = 0;
  lcd.setRGB(colorR, colorG, colorB);
  delay(500);
  colorR = 255;
  colorG = 255;
  colorB = 255;
  lcd.setRGB(colorR, colorG, colorB); 
  delay(500);
 // write HELLO message 
  lcd.setCursor(0, 0);
  lcd.print("Rotator     ");
  lcd.setCursor(0, 1);
  lcd.print("Controller    ");
  delay(1000);
  void clear(); 
 }

void Read_Joystick ()
{
  xPosition = analogRead(HorizPin);
  yPosition = analogRead(VertPin);
  park = digitalRead(ParkPin);
  if (PrintDebugMessages == 1)
  {
    Serial.println ("Park="+park);
    Serial.println ("Park="+String(park));
    Serial.println ("ParkRead="+String(digitalRead(ParkPin)));
  }
  
//Parking
  if (digitalRead(ParkPin) == 1)
  {    
    Serial.println ("Parking");
    IsParking = 1;
    colorR = 0;
    colorG = 0;
    colorB = 255;
    lcd.setRGB(colorR, colorG, colorB); // LCD blue when parking
  }

//Moving up
  if (yPosition > 600)
  {
// Green = moving    
    colorR = 0;
    colorG = 255;
    colorB = 0;
    lcd.setRGB(colorR, colorG, colorB); 
    lcd.setCursor(0, 0);
    lcd.print("Moving up       "); 
    IsMovingV = 1;
    IsParking = 0;
   }
  
//Moving down
  if (yPosition < 300)
  {
// Green = moving    
    colorR = 0;
    colorG = 255;
    colorB = 0;
    lcd.setRGB(colorR, colorG, colorB); 
    lcd.setCursor(0, 0);
    lcd.print("Moving down     "); 
    IsMovingV = -1;
    IsParking = 0;
   }

  //Moving right
  if (xPosition > 600)
  {
// Green = moving    
    colorR = 0;
    colorG = 255;
    colorB = 0;
    lcd.setRGB(colorR, colorG, colorB); 
    lcd.setCursor(0, 1);
    lcd.print("Moving right    "); 
    IsMovingH = 1;
    IsParking = 0;
   }

  //Moving left
  if (xPosition < 300)
  {
// Green = moving    
    colorR = 0;
    colorG = 255;
    colorB = 0;
    lcd.setRGB(colorR, colorG, colorB); 
    lcd.setCursor(0, 1);
    lcd.print("Moving left     "); 
    IsMovingH = -1;
    IsParking = 0;
   }

// Not moving vert
  if (( (yPosition-300) | (700 - yPosition)) >= 0)
  {
    lcd.setCursor(0, 0);
    lcd.print("Elevation stop  "); 
    IsMovingV = 0;
  }

// Not moving horiz
  if (( (xPosition-300) | (700 - xPosition)) >= 0)
  {
    lcd.setCursor(0, 1);
    lcd.print("Azimuth stop    "); 
    IsMovingH = 0;
  }

// Set color if not moving
  if (IsParking == 1)
    {
    colorR = 0;
    colorG = 0;
    colorB = 255;
    lcd.setRGB(colorR, colorG, colorB);  
    }
  else if ((abs(IsMovingH) + abs(IsMovingV)) == 0) //not moving
    {
    colorR = 255;
    colorG = 255;
    colorB = 255;
    lcd.setRGB(colorR, colorG, colorB);    
    }
  else
    { // moving
    colorR = 0;
    colorG = 255;
    colorB = 0;
    lcd.setRGB(colorR, colorG, colorB);   
    }

 // Debugging code
  if (DisplayJoystickValues > 0 )
    {
    lcd.setCursor(0, 0);
    lcd.print("Horiz=" + String(xPosition) + "       "); 
    lcd.setCursor(0, 1);
    lcd.print("Vert=" + String(yPosition) + "     ");   
    delay (3000);
    }
}
///////////////////////////////////////////////
void SwitchRelays ()
{
  if (PrintDebugMessages = 1)
  {
    Serial.println ("IsMovingH="+String(IsMovingH));
    Serial.println ("IsMovingV="+String(IsMovingV));
  }
  
  if (IsMovingH == 1)// Move left
  {
    digitalWrite (LeftPin,HIGH);
    if (PrintDebugMessages = 1) Serial.println ("Left relay engaged");
  }
  else 
  {
    digitalWrite (LeftPin,LOW);
  }
//--------------------------------
  if (IsMovingH == -1)// Move right
  {
    digitalWrite (RightPin,HIGH);
    if (PrintDebugMessages = 1) Serial.println ("Right relay engaged");
  }
  else 
  {
    digitalWrite(RightPin,LOW);
  }
//---------------------------------
   if (IsMovingV == 1)// Move up
  {
    digitalWrite (UpPin,HIGH);
    if (PrintDebugMessages = 1) Serial.println ("Up relay engaged");
  }
  else 
  {
    digitalWrite(UpPin,LOW);
  }
//--------------------------------  
   if (IsMovingV == -1)// Move down
  {
    digitalWrite (DownPin,HIGH);
    if (PrintDebugMessages = 1) Serial.println ("Down relay engaged");
  }
    else 
  {
    digitalWrite (DownPin,LOW);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  Read_Joystick();
  SwitchRelays ();
//  delay(3000);
}
