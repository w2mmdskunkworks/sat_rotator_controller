//Antenna rotator controller using relays and Grove connections

// Libraries
#include <Wire.h>
#include "rgb_lcd.h"
rgb_lcd lcd;

// radio drivers
// Code is from Arduino Cookbook page 461
//Connector wires are black, white, antenna, red.  White goes to pin 11
//#include <VirtualWire.h>
//byte message[VW_MAX_MESSAGE_LEN];// buffer for incoming messages
//byte msgLength = VW_MAX_MESSAGE_LEN;// size of the message

// Define debugging code
int DisplayJoystickValues = 0;
int PrintDebugMessages = 1;
int PrintRadioMessages = 0;

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
#define ParkPin 12
int VertPin = A14;
int HorizPin = A15;
// Variables into which to read the joystick value
int xPosition = 0;
int yPosition = 0;
int park = 0;

//Xbee joystick input variables
int XbeeVert;
int XbeeHoriz;
int XbeeTimeout = 0;
int XbeeControl = 0; // 1 if receiving data from Xbee

//Xbee position input variables
int AzIn = 0;
int ElIn1 = 0;
int ElIn2 = 0;
int ElIn3 = 0;
int ElIn4 = 0;
int ElNo = 0;
int AzNo = 0;

// Variables
int IsMovingV = 0; //1 if up, -1 if down
int IsMovingH = 0; //1 if right, -1 if left
int IsParking = 0; // 1 if parking; cancelled by joystick move
int RadioLoopCounter = 0;// Counter for radio command timeout

void setup() {
  Serial.println ("Starting");
  // Define relay pin modes
  pinMode (UpPin, OUTPUT);
  pinMode (DownPin, OUTPUT);
  pinMode (LeftPin, OUTPUT);
  pinMode (RightPin, OUTPUT);
  digitalWrite(UpPin, LOW);
  digitalWrite(DownPin, LOW);
  digitalWrite(LeftPin, LOW);
  digitalWrite(RightPin, LOW);

  // Pin modes for joystick
  pinMode(VertPin, INPUT);
  pinMode(HorizPin, INPUT);
  pinMode (ParkPin, INPUT_PULLUP);

  // Initialize serial ports
  Serial.begin(9600);
  Serial1.begin(9600);//Xbee

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

  // set up the radio
  //vw_setup(2000); //bits per second
  //vw_rx_start(); // start the receive
  }
//-------------------------------------------
void ReadXbeePosition ()
{
 if (Serial1.available() >= 4)// 4 characters to get 2 from 2-digit el, plus CRLF
 {
  //Serial.println ("Serial available="+String(Serial1.available()));
  ElIn1 = Serial1.read(); // read first value
  while (ElIn1 < 20) // If first byte is CR or LF, read until it isn't
  {
    ElIn1 = Serial1.read(); 
  }
  ElIn2 = Serial1.read(); // read 2nd value
  ElIn3 = Serial1.read(); // read 3rd value // read CR
  ElIn4 = Serial1.read(); // read 4thvalue // read LF
  Serial.println ("Read "+String(ElIn1)+" "+String(ElIn2)+" "+String(ElIn3)+" "+String(ElIn4));
  ElNo = ((ElIn1 - 48) * 10) + ElIn2 - 48;
  Serial.println ("ElNo="+String(ElNo));
  ShowPosition (); 
 }// if available
// delay (1000);
} // void
//------------------------------------------
void ShowPosition ()
{
  lcd.setCursor(0, 0);
  lcd.print("Az="+String(AzNo)+"           ");
  lcd.setCursor(0, 1);
  lcd.print("El="+String(ElNo)+"           ");
}

//-------------------------------------------
void ReadXbeeJoystick()
{
 //Serial.println ("Reading Xbee");
 if (Serial1.available() > 3 )
  {
     if(Serial1.read() == 48)// Starting sequence - next line is Y value
     {
      XbeeControl = 1;
      if (PrintRadioMessages == 1)Serial.println ("Starting sequence");
      XbeeHoriz = Serial1.read();
      if (PrintRadioMessages == 1)Serial.println (String(XbeeVert));
      XbeeVert = Serial1.read();
      if (PrintRadioMessages == 1)Serial.println (String(XbeeHoriz));

      // Move antenna based on message received
     if (XbeeVert == 85) IsMovingV = 1;
     if (XbeeVert == 83) IsMovingV = 0;
     if (XbeeVert == 68) IsMovingV = -1;
     if (XbeeHoriz == 76) IsMovingH = 1;
     if (XbeeHoriz == 83) IsMovingH = 0;
     if (XbeeHoriz == 82) IsMovingH = -1;

     XbeeTimeout = 0; //Reset Xbee timeout counter
     }
  }
  else
  {    
        // Nothing heard from Xbee - if counter exceeded stop rotation
      XbeeControl = 0; //Set flag to change LCD color
      XbeeTimeout ++;
      //Serial.println ("Counter="+String(XbeeTimeout));
      if (XbeeTimeout > 50) 
      {
        IsMovingH=0;
        IsMovingV=0;
      } 
     //Serial.println("V="+String(XbeeVert)+" H="+String(XbeeHoriz)); // Debugging 
  }
  //else Serial.println ("Waiting for data");
}
//void ReadRadio ()
//{
//  Serial.println ("Waiting for packet");
//  if (vw_get_message(message, &msgLength))
//  {
//    Serial.print ("Got: ");
//    for (int i = 0; i < msgLength; i++)
//    {
//      Serial.write(message[i]);   
//    }
//    Serial.print ("message[0]=");
//    Serial.println(message[0]);
//    Serial.print ("message[1]=");
//    Serial.println(message[1]);
//  
//// Move antenna based on message received
//    if (message[0] == 85) IsMovingV = 1;
//    if (message[0] == 83) IsMovingV = 0;
//    if (message[0] == 68) IsMovingV = -1;
//    if (message[1] == 76) IsMovingH = 1;
//    if (message[1] == 83) IsMovingH = 0;
//    if (message[1] == 82) IsMovingH = -1;
//
//    RadioLoopCounter = 0; // Reset counter because we have received a data packet
//    Serial.println();
//    Serial.println ("IsMovingV="+String(IsMovingV));
//    Serial.println ("IsMovingH="+String(IsMovingH));
//    delay(1000);
//  }
//
//  RadioLoopCounter ++;
//  //Serial.println ("Loop counter="+String(RadioLoopCounter));
//  
//  if (RadioLoopCounter > 20000) // stop antenna
//  {
//    IsMovingV = 0;
//    IsMovingH = 0;
//    //Serial.println("Move timeout");
//  }
//}
//---------------------------------------------------------------------
void Read_Joystick ()
{
  xPosition = analogRead(HorizPin);
  yPosition = analogRead(VertPin);
  park = digitalRead(ParkPin);
  if (PrintDebugMessages == 1)
  {
    Serial.println ("Park=" + park);
    Serial.println ("Park=" + String(park));
    Serial.println ("ParkRead=" + String(digitalRead(ParkPin)));
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
    IsMovingV = 1;
    IsParking = 0;
  }

  //Moving down
  if (yPosition < 300)
  {
    IsMovingV = -1;
    IsParking = 0;
  }

  //Moving right
  if (xPosition > 600)
  {
    IsMovingH = 1;
    IsParking = 0;
  }

  //Moving left
  if (xPosition < 300)
  {
    IsMovingH = -1;
    IsParking = 0;
  }

  // Not moving vert
  if (( (yPosition - 300) | (700 - yPosition)) >= 0)
  {
    lcd.setCursor(0, 0);
    lcd.print("Elevation stop  ");
    IsMovingV = 0;
  }

  // Not moving horiz
  if (( (xPosition - 300) | (700 - xPosition)) >= 0)
  {
    lcd.setCursor(0, 1);
    lcd.print("Azimuth stop    ");
    IsMovingH = 0;
  }

// testing - can probably remove this code
//  // Set color if not moving
//  if (IsParking == 1)
//  {
//    colorR = 0;
//    colorG = 0;
//    colorB = 255;
//    lcd.setRGB(colorR, colorG, colorB);
//  }
//  else if ((abs(IsMovingH) + abs(IsMovingV)) == 0) //not moving
//  {
//    colorR = 255;
//    colorG = 255;
//    colorB = 255;
//    lcd.setRGB(colorR, colorG, colorB);
//  }
//  else
//  { // moving
//    colorR = 0;
//    colorG = 255;
//    colorB = 0;
//    lcd.setRGB(colorR, colorG, colorB);
//  }

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
  if (PrintDebugMessages == 1)
  {
    Serial.println ("IsMovingH=" + String(IsMovingH));
    Serial.println ("IsMovingV=" + String(IsMovingV));
  }

  if (IsMovingH == 1)// Move left
  {
    digitalWrite (LeftPin, HIGH);
    if (PrintDebugMessages == 1) Serial.println ("Left relay engaged");
    colorR = 0;
    colorG = 255;
    colorB = 0;
    lcd.setRGB(colorR, colorG, colorB);
    lcd.setCursor(0, 1);
    lcd.print("Moving left     ");
  }
  else
  {
    digitalWrite (LeftPin, LOW);
  }
  //--------------------------------
  if (IsMovingH == -1)// Move right
  {
    digitalWrite (RightPin, HIGH);
    if (PrintDebugMessages == 1) Serial.println ("Right relay engaged");
    colorR = 0;
    colorG = 255;
    colorB = 0;
    lcd.setRGB(colorR, colorG, colorB);
    lcd.setCursor(0, 1);
    lcd.print("Moving left     ");
  }
  else
  {
    digitalWrite(RightPin, LOW);
  }
  //---------------------------------
  if (IsMovingV == 1)// Move up
  {
    digitalWrite (UpPin, HIGH);
    if (PrintDebugMessages == 1) Serial.println ("Up relay engaged");
    // Green = moving
    colorR = 0;
    colorG = 255;
    colorB = 0;
    lcd.setRGB(colorR, colorG, colorB);
    lcd.setCursor(0, 0);
    lcd.print("Moving up       ");
  }
  else
  {
    digitalWrite(UpPin, LOW);
  }
  //--------------------------------
  if (IsMovingV == -1)// Move down
  {
    digitalWrite (DownPin, HIGH);
    if (PrintDebugMessages == 1) Serial.println ("Down relay engaged");
    colorR = 0;
    colorG = 255;
    colorB = 0;
    lcd.setRGB(colorR, colorG, colorB);
    lcd.setCursor(0, 0);
    lcd.print("Moving down     ");
  }
  else
  {
    digitalWrite (DownPin, LOW);
  }

// Change to white if not moving
  if ((abs(IsMovingH) + abs(IsMovingV)) == 0) //not moving
  {
// if Xbee control
//    colorR = 255;
//    colorG = 147;
//    colorB = 38;
    colorR = 255;
    colorG = 255;
    colorB = 255;
    lcd.setRGB(colorR, colorG, colorB);
    lcd.setCursor(0, 0);
    lcd.print("Elevat stopped  ");
    lcd.setCursor(0, 1);
    lcd.print("Azimuth stopped ");
  }  
}

void loop() 
{
  // put your main code here, to run repeatedly:
  //ReadXbeePosition();
  //ReadXbeeJoystick();
  Read_Joystick();
  //ReadRadio ();
  SwitchRelays ();
  //ReadRadio ();
  delay(1000);
}
