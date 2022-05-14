//Antenna rotator controller using relays and Grove connections
// Code de WB2MNF
// Integrating the code from KW56GP

// Define debugging code
int DisplayJoystickValues = 0;
int PrintDebugMessages = 1;
int PrintRadioMessages = 0;
int PrintADC = 0;
int PrintJoystickPosition =0;
int AzEl_debug_mode = 0;
int debug_mode = 1;

// Libraries
#include <Wire.h>
#include "rgb_lcd.h"
rgb_lcd lcd;

#include "EEPROM.h" // Include the EEPROM Library 
#include "ADS1115.h" // Include the ADS1115.h Library (Library Updated to fix errors) 
ADS1115 adc;
#include <SPI.h>   // Comes with Arduino IDE

// radio drivers
// Code is from Arduino Cookbook page 461
//Connector wires are black, white, antenna, red.  White goes to pin 11
//#include <VirtualWire.h>
//byte message[VW_MAX_MESSAGE_LEN];// buffer for incoming messages
//byte msgLength = VW_MAX_MESSAGE_LEN;// size of the message

//EPROM definitions
#define EEPROM_ID_BYTE 1 // EEPROM ID to validate EEPROM data location 
#define EEPROM_ID 55 // EEPROM ID Value 
#define EEPROM_AZ_CAL_0 2 // Azimuth Zero Calibration EEPROM location 
#define EEPROM_AZ_CAL_MAX 4 // Azimuth Max Calibration Data EEPROM 
#define EEPROM_EL_CAL_0 6 // Elevation Zero Calibration Data EEPROM location 
#define EEPROM_EL_CAL_MAX 8 // Elevation Max Calibration Data EEPROM location 
#define AZ_CAL_0_DEFAULT 0 // Preset the Azimuth Zero Calibration Point to 0 
#define AZ_CAL_MAX_DEFAULT 27000 // Preset the Azimuth Max Calibration Point to 27000 
#define EL_CAL_0_DEFAULT 0 // Preset the Elevation Zero Calibration Point to 0 
#define EL_CAL_MAX_DEFAULT 27000 // Preset the Elevation Max Calibration Point to 27000 

// NOTE - setting tolerance to 20 for testing - set to something like 3 for operation
#define AZ_Tolerance 20 // Set the Azimuth Accuracy Tolerance 
#define EL_Tolerance 20 // Set the Elevation Accuracy Toleance 

//variables for tracking code
byte inByte = 0; // incoming serial byte
byte serial_buffer[50]; // incoming serial byte buffer
int serial_buffer_index = 0; // The index pointer variable for the Serial buffer
int set_AZ; // Azimuth set value
int set_EL; // Elevation set value
int current_AZ; // Current Azimuth raw value
int current_EL; // Current Elevation raw value
String Serial_Send_Data; // Data to send to Serial Port
int AZ_0; // Azimuth Zero Value from EEPROM
int AZ_MAX; // Azimuth Max Value from EEPROM
int EL_0; // Elevation 0 Value from EEPROM
int EL_MAX; //Elevation Max Value from EEPROM
int AZ_Degrees; // mapped AZ ADC value to Degrees
int EL_Degrees; // mapped EL ADC value to Degrees
String Requested_AZ; // RS232 Requested Azimuth - M and short W command
String Requested_EL; //RS232 Requested Azimuth and Elevation - Full W command
int AZ_To; // Requested AZ Move
int EL_To; // Requested EL Move
int AZ_Distance; // Distance to move AZ
int EL_Distance; // Distance to move EL

// Define initial LCD colors
int colorR = 255;
int colorG = 0;
int colorB = 0;

// Pins for relay output
#define rotate_up 7//5
#define rotate_down 6//4
#define rotate_left 5//6
#define rotate_right 4//7

//Define pins for joystick
#define park_button 15 // changed from 12 to allow calibrate button
#define cal_button 10
int VertPin = 15;
int HorizPin = 14;
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
int IsPC_move = 1; // 1 if PC is controlling the az/el movement, 0 otherwise
int RadioLoopCounter = 0;// Counter for radio command timeout
//----------------------------------------------------------------
void setup() {
  Serial.println ("Starting");
  // Define relay pin modes
  pinMode (rotate_up, OUTPUT);
  pinMode (rotate_down, OUTPUT);
  pinMode (rotate_left, OUTPUT);
  pinMode (rotate_right, OUTPUT);
  digitalWrite(rotate_up, LOW);
  digitalWrite(rotate_down, LOW);
  digitalWrite(rotate_left, LOW);
  digitalWrite(rotate_right, LOW);

  // Pin modes for joystick
  pinMode(VertPin, INPUT);
  pinMode(HorizPin, INPUT);
  pinMode (park_button, INPUT_PULLUP);
  pinMode (cal_button, INPUT_PULLUP);

  // Initialize serial ports
  Serial.begin(9600);
  Serial1.begin(9600);//Xbee
  
  //Initialize ADC
  Wire.begin(); // join I2C bus
  adc.initialize(); // initialize ADS1115 16 bit A/D chip
  Wire.beginTransmission(0x48); // Begin direct ADC communication Wire.write(0x1);

// test if ADC is connected
  if (debug_mode) // Test connection in debug mode
    {
      delay (100);
     if (adc.testConnection ()) 
       {
         Serial.println("ADC connection valid");
       }
     else 
       {
         Serial.println("ADC connection NOT valid");
         lcd.begin(16, 2);
         colorR = 255;
         colorG = 0;
         colorB = 0;
         lcd.setRGB(colorR, colorG, colorB);
         lcd.setCursor(0, 0);
         lcd.print("ADC         ");
         lcd.setCursor(0, 1);
         lcd.print("Failed        ");
         delay (10000);
       }
    } 
  
// Connect to adc and send two bytes - Set Config Reg to all Ones
  Wire.write(0x7F); // MSB
  Wire.write(0xFF); // LSB
  Wire.endTransmission(); // End the direct ADC Communication
  adc.setMode(ADS1115_MODE_CONTINUOUS); // Set the ADC to free running conversion mode
  adc.setGain(ADS1115_PGA_6P144); // set the ADC gain to 6.144 Volt range, .0001875 Volts/step
  adc.setRate(ADS1115_RATE_475); // set ADC sample rate to 475 samples per second
  adc.setMultiplexer(ADS1115_MUX_P0_NG); // Set the ADC to AN0+ Vs ground Mode

// initialize Az-El
  set_AZ = -1; // Preset the Azimuth and Elevation Move Variables
  set_EL = -1;
  read_eeprom_cal_data(); // Read the Azimuth and Elevation Calibration Values from EEPROM } // End Setup Loop

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
//---------------------------------------------------------
void read_adc() // Function to read the ADC 
{
  if (PrintADC)
    {
      Serial.println("Read ADC Function ");
    }
  int RotorValue; // Variable to store the rotor value
  adc.setRate(ADS1115_RATE_475);
  // Set the ADC rate to 475 samples/sec
  adc.setGain(ADS1115_PGA_6P144);
  // Set the ADC gain to 6.144V
  adc.setMultiplexer(ADS1115_MUX_P0_NG);
  // Set the ADC to Channel 0 AN0+ Vs ground
  delay(10);
  // adc settling delay
  current_EL = adc.getDiff0();
  // Read ADC Channel 0
  if (PrintADC)
    {
      Serial.print("Current raw EL=");  
      Serial.println(current_EL);
    }
  adc.setMultiplexer(ADS1115_MUX_P1_NG);
  // Set the ADC to Channel 1 AN1+ Vs ground
  delay(10);
  // adc settling delay
  current_AZ = adc.getDiff1();
  // Read ADC Channel 1
  if (PrintADC)
    {
      Serial.print("Current raw AZ=");  
      Serial.println(current_AZ);
    }
}
//----------------------------------------
void loop() 
{
  // put your main code here, to run repeatedly:
  //Serial.println ("Cal button="+String(digitalRead(cal_button)));
  calibrate();
  check_serial();
  check_move();
  //ReadXbeePosition();
  //ReadXbeeJoystick();
  Read_Joystick();
  //ReadRadio ();
  SwitchRelays ();
  //ReadRadio ();
  delay(100);
}
//-----------------------------------------------------------------------------
void calibrate()
{
    if (digitalRead(cal_button) == 1)
    {   
      // Change LCD to orange
      colorR = 255;
      colorG = 0;
      colorB = 255;
      lcd.setRGB(colorR, colorG, colorB);
      // Prompt to move az and el to zero then press button
      lcd.setCursor(0, 0);
      lcd.print("Set Az/El zero  ");
      lcd.setCursor(0, 1);
      lcd.print("Then press cal  ");
      delay (1000);
      // prompt to move az and el to max then press button     
      while (digitalRead(cal_button) == 0)
        {
          delay (10);
        } // wait for button press
      // after button pressed
      set_0_az_cal();
      set_0_el_cal();
//      lcd.setCursor(0, 0);
//      lcd.print("Raw Az="+String(current_AZ));
//      lcd.setCursor(0, 1);
//      lcd.print("Raw El="+String(current_EL));
      delay (5000);
      lcd.setCursor(0, 0);
      lcd.print("Set Az/El max   ");
      lcd.setCursor(0, 1);
      lcd.print("Then press cal  ");
      delay (1000);
      while (digitalRead(cal_button) == 0) 
        {
          delay (10);
        } // wait for button press   
      set_max_az_cal();
      set_max_el_cal();   
//      lcd.setCursor(0, 0);
//      lcd.print("Raw Az="+String(current_AZ));
//      lcd.setCursor(0, 1);
//      lcd.print("Raw El="+String(current_EL));
      delay (5000);
      // record values in EPROM
      lcd.setCursor(0, 0);
      lcd.print("Cal recorded    ");
      lcd.setCursor(0, 1);
      lcd.print("in EPROM        ");
      delay (2000);
    }
}
//-----------------------------------------------------------------------------
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
  send_current_azel;
  lcd.setCursor(0, 0);
  lcd.print("Az="+String(AZ_Degrees)+"           ");
  lcd.setCursor(0, 1);
  lcd.print("El="+String(EL_Degrees)+"           ");
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
  park = digitalRead(park_button);
  if (PrintJoystickPosition == 1)
  {
    Serial.println ("xPosition="+String(xPosition));
    Serial.println ("yPosition="+String(yPosition));
    Serial.println ("Park=" + park);
    Serial.println ("Park=" + String(park));
    Serial.println ("ParkRead=" + String(digitalRead(park_button)));
  }
  //Parking
  //if (digitalRead(park_button) == 1)
//  {
//    Serial.println ("Parking");
//    IsParking = 1;
//    colorR = 0;
//    colorG = 0;
//    colorB = 255;
//    lcd.setRGB(colorR, colorG, colorB); // LCD blue when parking
//  }

  //Moving down
  if (yPosition > 600)
  {
    IsMovingV = -1;
    IsParking = 0;
    IsPC_move = 0; // if manual control stop auto move
    set_AZ = -1;
    set_EL = -1;
  }

  //Moving up
  if (yPosition < 300)
  {
    IsMovingV = 1;
    IsParking = 0;
    IsPC_move = 0; // if manual control stop auto move
    set_AZ = -1;
    set_EL = -1;
  }

  //Moving right
  if (xPosition > 600)
  {
    IsMovingH = 1;
    IsParking = 0;
    IsPC_move = 0; // if manual control stop auto move
    set_AZ = -1;
    set_EL = -1;    
  }

  //Moving left
  if (xPosition < 300)
  {
    IsMovingH = -1;
    IsParking = 0;
    IsPC_move = 0; // if manual control stop auto move
    set_AZ = -1;
    set_EL = -1;    
  }

// Not moving vert
  if (( (yPosition - 300) | (700 - yPosition)) >= 0)
  {
    if (IsPC_move == 0) //Only reset IsMoving if the PC isn't in control
      IsMovingV = 0;
  }

// Not moving horiz
  if (( (xPosition - 300) | (700 - xPosition)) >= 0)
  {
    if (IsPC_move == 0) //Only reset IsMoving if the PC isn't in control
      IsMovingH = 0;
  }
  // Debugging code
  if (PrintJoystickPosition > 0 )
  {
    lcd.setCursor(0, 0);
    lcd.print("Horiz=" + String(xPosition) + "       ");
    lcd.setCursor(0, 1);
    lcd.print("Vert=" + String(yPosition) + "     ");
    delay (1000);
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
    send_current_azel(); // get current position   
    digitalWrite (rotate_left, HIGH);
    if (PrintDebugMessages == 1) Serial.println ("Left relay engaged to "+String(AZ_Degrees));
    if (IsPC_move == 0) // Manual control
    {
      colorR = 0;
      colorG = 255;
      colorB = 0;
      lcd.setRGB(colorR, colorG, colorB);
      lcd.setCursor(0, 1);
      lcd.print("Moving left "+String(AZ_Degrees)+"         ");
    }
    else // PC control
     {
      colorR = 0;
      colorG = 0;
      colorB = 255;
      lcd.setRGB(colorR, colorG, colorB);
      lcd.setCursor(0, 1);
      lcd.print("Left " + String(AZ_Degrees) + ">"+String(AZ_To)+ "     ");
    }     
  }
  else // not moving left
  {
    digitalWrite (rotate_left, LOW);
  }
  //--------------------------------
  if (IsMovingH == -1)// Move right
  {
    digitalWrite (rotate_right, HIGH);
    send_current_azel(); // Update current position for LCD display
    if (PrintDebugMessages == 1) Serial.println ("Right relay engaged to "+String(AZ_Degrees));
    if (IsPC_move == 0) // Manual control
    {
      colorR = 0;
      colorG = 255;
      colorB = 0;
      lcd.setRGB(colorR, colorG, colorB);
      lcd.setCursor(0, 1);
      lcd.print("Moving right "+String(AZ_Degrees)+"     "+ "     ");
    }
    else // PC control
     {
      colorR = 0;
      colorG = 0;
      colorB = 255;
      lcd.setRGB(colorR, colorG, colorB);
      lcd.setCursor(0, 1);
      lcd.print("Right " + String(AZ_Degrees) + ">"+String(AZ_To)+"     "+ "     ");
    }  
  }
  else
  {
    digitalWrite(rotate_right, LOW);
  }
  //---------------------------------
  if (IsMovingV == 1)// Move up
  {
    digitalWrite (rotate_up, HIGH);
    send_current_azel(); // Update current position for LCD display
    if (PrintDebugMessages == 1) Serial.println ("Up relay engaged to "+String(EL_Degrees));
    // Green = moving
    if (IsPC_move == 0) // Manual control
    {
      colorR = 0;
      colorG = 255;
      colorB = 0;
      lcd.setRGB(colorR, colorG, colorB);
      lcd.setCursor(0, 0);
      lcd.print("Moving up "+String(EL_Degrees)+"     "+ "     ");
    }
    else // PC control
    {
      colorR = 0;
      colorG = 0;
      colorB = 255;
      lcd.setRGB(colorR, colorG, colorB);
      lcd.setCursor(0, 0);
      lcd.print("Up " + String(EL_Degrees) + ">"+String(EL_To)+ "     ");
   }  
  }
  else
  {
    digitalWrite(rotate_up, LOW);
  }
  //--------------------------------
  if (IsMovingV == -1)// Move down
  {
    digitalWrite (rotate_down, HIGH);
    send_current_azel(); // Update current position for LCD display
    if (PrintDebugMessages == 1) Serial.println ("Down relay engaged to "+String(EL_Degrees));
    if (IsPC_move == 0) // Manual control
    {
      colorR = 0;
      colorG = 255;
      colorB = 0;
      lcd.setRGB(colorR, colorG, colorB);
      lcd.setCursor(0, 0);
      lcd.print("Moving down " + String(EL_Degrees)+"     "+ "     ");
    }
    else // PC control
    {
      colorR = 0;
      colorG = 0;
      colorB = 255;
      lcd.setRGB(colorR, colorG, colorB);
      lcd.setCursor(0, 0);
      lcd.print("Down " + String(EL_Degrees) + ">"+String(EL_To)+ "     ");
    }   
  }
  else
  {
    digitalWrite (rotate_down, LOW);
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
    
    read_adc(); // Read the ADC // Map Azimuth to degrees
    AZ_Degrees = map(current_AZ, AZ_0, AZ_MAX, 0, 360);
    EL_Degrees = map(current_EL, EL_0, EL_MAX, 0, 90);

    if (PrintADC == 1)
    {
      Serial.println ("SwitchRelays code - reading az-el in next step");
      //send_current_azel; // get current position for display   
      Serial.println ("Az="+String(AZ_Degrees));
      Serial.println ("El="+String(EL_Degrees));
      Serial.println ("Raw AZ="+String(current_AZ));
      Serial.println ("Raw EL="+String(current_EL));
    }
    lcd.setRGB(colorR, colorG, colorB);
    lcd.setCursor(0, 0);
    lcd.print("El="+String(EL_Degrees)+"          ");
    lcd.setCursor(0, 1);
    lcd.print("Az="+String(AZ_Degrees)+"          ");
  }  
}
//--------------------------------------------------------------------
void read_eeprom_cal_data() // Function to Read the Azimuth and Elevation Calibration Data
{ if (EEPROM.read(EEPROM_ID_BYTE) == EEPROM_ID) // Verify the EEPROM has valid data
  {
    if (debug_mode) // If in Debug Mode Print the Calibration Values
    {
      Serial.println("Read EEPROM Calibration Data Valid ID");
      Serial.println((EEPROM.read(EEPROM_AZ_CAL_0) * 256) + EEPROM.read(EEPROM_AZ_CAL_0 + 1), DEC);
      Serial.println((EEPROM.read(EEPROM_AZ_CAL_MAX) * 256) + EEPROM.read(EEPROM_AZ_CAL_MAX + 1), DEC);
      Serial.println((EEPROM.read(EEPROM_EL_CAL_0) * 256) + EEPROM.read(EEPROM_EL_CAL_0 + 1 ), DEC);
      Serial.println((EEPROM.read(EEPROM_EL_CAL_MAX) * 256) + EEPROM.read(EEPROM_EL_CAL_MAX + 1), DEC);
    }
    AZ_0 = (EEPROM.read(EEPROM_AZ_CAL_0) * 256) + EEPROM.read(EEPROM_AZ_CAL_0 + 1);
    // Read the Azimuth Zero Calibration Value from EEPROM
    AZ_MAX = (EEPROM.read(EEPROM_AZ_CAL_MAX) * 256) + EEPROM.read(EEPROM_AZ_CAL_MAX + 1);
    // Read the Azimuth Maximum Calibration Value from EEPROM
    EL_0 = (EEPROM.read(EEPROM_EL_CAL_0) * 256) + EEPROM.read(EEPROM_EL_CAL_0 + 1);
    // Read the Elevation Zero Calibration Value from EEPROM
    EL_MAX = (EEPROM.read(EEPROM_EL_CAL_MAX) * 256) + EEPROM.read(EEPROM_EL_CAL_MAX + 1);
    // Read the Elevation Maximum Calibration Value from EEPROM
  }
  else {
    // initialize eeprom to default values if (debug_mode)
    {
      Serial.println("Read EEPROM Calibration Data Invalid ID - setting to defaults");
    }
    AZ_0 = AZ_CAL_0_DEFAULT;
    // Set the Calibration To Default Values
    AZ_MAX = AZ_CAL_MAX_DEFAULT;
    EL_0 = EL_CAL_0_DEFAULT;
    EL_MAX = EL_CAL_MAX_DEFAULT;
    write_eeprom_cal_data(); // Write the Default Values to EEPROM
  }
}
//---------------------------------------------------------------------------
void write_eeprom_cal_data() // Function to Write the Calibration Values to EEPROM
{
  if (debug_mode) {
    Serial.println("Writing EEPROM Calibration Data");
  }
  EEPROM.write(EEPROM_ID_BYTE, EEPROM_ID);
  // Write the EEPROM ID
  EEPROM.write(EEPROM_AZ_CAL_0, highByte(AZ_0));
  // Write the Azimuth Zero Calibration High Order Byte
  EEPROM.write(EEPROM_AZ_CAL_0 + 1, lowByte(AZ_0));
  // Write the Azimuth Zero Calibration Low Order Byte
  EEPROM.write(EEPROM_AZ_CAL_MAX, highByte(AZ_MAX));
  // Write the Azimuth Max Calibration High Order Byte
  EEPROM.write(EEPROM_AZ_CAL_MAX + 1, lowByte(AZ_MAX));
  // Write the Azimuth Max Calibration Low Order Byte
  EEPROM.write(EEPROM_EL_CAL_0, highByte(EL_0));
  // Write the Elevation Zero Calibration High Order Byte
  EEPROM.write(EEPROM_EL_CAL_0 + 1, lowByte(EL_0));
  // Write the Elevation Zero Calibration Low Order Byte
  EEPROM.write(EEPROM_EL_CAL_MAX, highByte(EL_MAX));
  // Write the Elevation Max Calibration High Order Byte
  EEPROM.write(EEPROM_EL_CAL_MAX + 1, lowByte(EL_MAX));
  // Write the Elevation Max Calibration Low Order Byte
}

//---------------------------------------------------------------------
void send_current_az() // Send the Current Azimuth Function
{
  read_adc();
  // Read the ADC // Map Azimuth to degrees
  if (debug_mode)
  {
    Serial.println(current_AZ);
  }
  AZ_Degrees = map(current_AZ, AZ_0, AZ_MAX, 0, 360);
  // Map the Current Azimuth to Degrees if (AZ_Degrees > 180)
  // Correction Since Azimuth Reading starts at Meter Center Point
//  {
//    AZ_Degrees = AZ_Degrees - 180;
//  }
//  else
//  {
//    AZ_Degrees = AZ_Degrees + 180;
//  }
  if (debug_mode)
  {
    Serial.print("AZ_Degrees=");    
    Serial.println(AZ_Degrees);
  } // Send it back via serial
  Serial_Send_Data = "";
  if (AZ_Degrees < 100) // pad with 0's if needed
  {
    Serial_Send_Data = "0";
  }
  if (AZ_Degrees < 10)
  {
    Serial_Send_Data = "00";
  }
  Serial_Send_Data = "+0" + Serial_Send_Data + String(AZ_Degrees);
  // Send the Azimuth in Degrees
  Serial.println(Serial_Send_Data);
  // Return value via RS-232 port
}
//----------------------------------------------------------------------------------
void send_current_azel() // Function to Send the Current Azimuth and Elevation
{
  read_adc(); // Read the ADC // Map Azimuth to degrees
  if (AzEl_debug_mode)
  {
    Serial.println ("Sending current AzEl");
    Serial.print("current_AZ=");    
    Serial.println(current_AZ);
  }
  AZ_Degrees = map(current_AZ, AZ_0, AZ_MAX, 0, 360);
  
  // Map the Current Azimuth to Degrees 
  //if (AZ_Degrees > 180)
  // Correction Since Azimuth Reading starts at Meter Center Point
//  {
//    AZ_Degrees = AZ_Degrees - 180;
//  }
//  else
//  {
//    AZ_Degrees = AZ_Degrees + 180;
//  }

  // Map Elevation to degrees
  if (AzEl_debug_mode)
  {
    Serial.println ("Sending Current AzEl");
    Serial.print("current_EL=");
    Serial.println(current_EL);
  }
  EL_Degrees = map(current_EL, EL_0, EL_MAX, 0, 90);
  // Map the Elevation to Degrees
  if (AzEl_debug_mode)
  {
    Serial.print("EL_Degrees=");
    Serial.println(EL_Degrees);
    Serial.print("AZ_Degrees=");
    Serial.println(AZ_Degrees);
  }
  // Send it back via serial
  Serial_Send_Data = "";
  if (AZ_Degrees < 100) // pad with 0's if needed
  {
    Serial_Send_Data = "0";
  }
  if (AZ_Degrees < 10)
  {
    Serial_Send_Data = "00";
  }
  Serial_Send_Data = "+0" + Serial_Send_Data + String(AZ_Degrees) + "+0";
  // Send the Azimuth part of the string if (EL_Degrees < 100)
  // pad with 0's if needed
  {
    Serial_Send_Data = Serial_Send_Data + "0";
  }
  if (EL_Degrees < 10)
  {
    Serial_Send_Data = Serial_Send_Data + "0";
  }
  Serial_Send_Data = Serial_Send_Data + String(EL_Degrees);
  // Send the Elevation Part of the String
  //Serial.println(Serial_Send_Data); // may have to uncomment
  // Return value via RS-232 port
}
//-------------------------------------------------------------------------
void set_max_az_cal()
// Set the Max Azimuth Calibration Function 
{
  if (debug_mode)
  {
    Serial.println("Calibrate Max AZ Function");
  }
  read_adc();
  // Read the ADC // save current az and el values to EEPROM - Zero Calibration 
  if (debug_mode)
  {
    Serial.print("current_AZ=");  
    Serial.println(current_AZ);
  }
  AZ_MAX = current_AZ;
  // Set the Azimuth Maximum Calibration to Current Azimuth Reading
  write_eeprom_cal_data();
  // Write the Calibration Data to EEPROM
  if (debug_mode)
  {
    Serial.println("Max Azimuth Calibration Complete");
  }
}
void set_max_el_cal() // Set the Max Elevation Calibration Function
{
  if (debug_mode)
  {
    Serial.print("current_EL=");    
    Serial.println("Calibrate EL Max Function");
  }
  read_adc();
  // Read the ADC // save current Azimuth and Elevation values to EEPROM - Zero Calibration
  if (debug_mode)
  {
 Serial.print("current_EL=");        
    Serial.println(current_EL);
  }
  EL_MAX = current_EL;
  // Set the Elevation Max Calibration to the Current Elevation Reading
  write_eeprom_cal_data();
  // Write the Calibration Data to EEPROM
  if (debug_mode)
  {
    Serial.println("Max Elevation Calibration Complete");
  }
}
//-----------------------------------------------------------------------
void rotate_az_ccw()
// Function to Rotate Azimuth CCW
{
  digitalWrite(rotate_left, HIGH);
  IsPC_move = 1;// Tell display that PC is controlling the move
  IsMovingH = 1;// Tell display that the rotator is moving
  // Set the Rotate Left Pin High
  digitalWrite(rotate_right, LOW);
  // Make sure the Rotate Right Pin is Low
  if (debug_mode)
    {
      Serial.println("Rotating CCW");
    }  
}
//--------------------------------------------
void rotate_az_cw()
// Function to Rotate Azimuth CW
{
  digitalWrite(rotate_right, HIGH);
  IsMovingH = 1;// Tell display that the rotator is moving
  IsPC_move = 1;// Tell display that PC is controlling the move
  // Set the Rotate Right Pin High
  digitalWrite(rotate_left, LOW);
  // Make sure the Rotate Left Pin Low
  if (debug_mode)
      {
        Serial.println("Rotating CW");
      }  
}
void rotate_el_up()
// Function to Rotate Elevation Up
{
  digitalWrite(rotate_up, HIGH);
  IsMovingV = 1;// Tell display that the rotator is moving
  IsPC_move = 1;// Tell display that PC is controlling the move
  // Set the Rotate Up Pin High
  digitalWrite(rotate_down, LOW);
  // Make sure the Rotate Down Pin is Low
  if (debug_mode)
      {
        Serial.println("Rotating up");
      }  
}
void rotate_el_down()
// Function to Rotate Elevation Up
{
  digitalWrite(rotate_down, HIGH);
  IsMovingV = -1;// Tell display that the rotator is moving
  IsPC_move = 1;// Tell display that PC is controlling the move
  // Set the Rotate Down Pin High
  digitalWrite(rotate_up, LOW);
  // Make sure the Rotate Up Pin is Low
  if (debug_mode)
      {
        Serial.println("Rotating down");
      }  
}
//------------------------------------------------
void az_rotate_stop()
// Function to Stop Azimuth Rotation
{
  digitalWrite(rotate_right, LOW);
  // Turn off the Rotate Right Pin
  digitalWrite(rotate_left, LOW);
  // Turn off the Rotate Left Pin
  IsMovingH = 0;// Tell display that the rotator is stopped
  if (IsMovingV == 0) // if not moving V then the PC control is over; otherwise PC is still controlling V
    IsPC_move = 0;// Tell display that PC is controlling the move
}
//------------------------------------------------------
void el_rotate_stop() // Function to Stop Elevation Rotation
{
  digitalWrite(rotate_up, LOW); // Turn off the Rotate Up Pin
  digitalWrite(rotate_down, LOW);
  IsMovingV = 0;// Tell display that the rotator is stopped
  if (IsMovingH == 0) // If still moving V PC is still controlling
    IsPC_move = 0;// Tell display that PC is controlling the move
  // Turn off the Rotate Down Pin
}
//------------------------------------------------------------
void rotate_to()
// Function to Rotate to Set Point
{
  if (debug_mode) 
  {
    Serial.println("M Command - Rotate Azimuth To Function");
  }
  // Decode Command - Format Mxxx - xxx = Degrees to Move to
   if (debug_mode) 
  {
    Serial.print("serial_buffer_index=");
    Serial.println(serial_buffer_index);
  }
  if (serial_buffer_index == 4) // Verify the Command is the proper length
  {
    if (debug_mode)
    {
      Serial.println("Value in [1] to [3]?");
    }
    Requested_AZ = (String(char(serial_buffer[1])) + String(char(serial_buffer[2])) + String(char(serial_buffer[3]))) ;
    // Decode the Azimuth Value
    AZ_To = (Requested_AZ.toInt());
    // AZ Degrees to Move to as integer
    if (AZ_To < 0) // Make sure we don't go below 0 degrees
    {
      AZ_To = 0;
    }
    if (AZ_To > 360) // Make sure we don't go over 360 degrees
    {
      AZ_To = 360;
    }
//    if (AZ_To > 180) // Adjust for Meter starting at Center
//    {
//      AZ_To = AZ_To - 180;
//    }
//    else
//    {
//      AZ_To = AZ_To + 180;
//    }
    if (debug_mode)
    {
      Serial.print("Requested_AZ=");
      Serial.println(Requested_AZ);
      Serial.print("AZ_To=");
      Serial.println(AZ_To);
    }
  } 
  //if (serial_buffer_index == 4)
  // set the move flag and start
  read_adc();
  // Read the ADC // Map it to degrees
  if (debug_mode)
  {
    Serial.print("current_AZ=");
    Serial.println(current_AZ);
  }
  AZ_Degrees = map(current_AZ, AZ_0, AZ_MAX, 0, 360);
  // Map the Azimuth Value to Degrees
  if (debug_mode)
  {
    Serial.print("AZ_Degrees=");      
    Serial.println(AZ_Degrees);
  }
  AZ_Distance = AZ_To - AZ_Degrees;
  if (debug_mode)
  {
    Serial.print("AZ_Distance=");
    Serial.println(AZ_Distance);
  }   
  // Figure out far we have to move set_AZ = AZ_To;
  if (abs(AZ_Distance) <= AZ_Tolerance)
    // No move needed if we're within the Tolerance Range
  {
    az_rotate_stop(); // Stop the Azimuth Rotation
    set_AZ = -1;
    // Turn off the Move Command
  }
  else
  {
    // Move Azimuth - figure out which way 
    if (AZ_Distance > 0)
    //We need to move CCW
    {
      rotate_az_ccw();
      // If the distance is positive, move CCW
    }
    else
    {
      rotate_az_cw();
      // Otherwise, move clockwise
    }
  }
}
//}
//------------------------------------------------------------------
void send_current_el()
// Function to Send the Current Elevation
{
  read_adc();
  // Read the ADC // Map it to degrees
  if (debug_mode)
  {
    Serial.print("current_EL=");    
    Serial.println(current_EL);
  }
  EL_Degrees = map(current_EL, EL_0, EL_MAX, 0, 90);
  // Map the Elevation Value to Degrees
  if (debug_mode)
  {
    Serial.print("EL_Degrees=");
    Serial.println(EL_Degrees);
    Serial.print("AZ_Degrees=");
    Serial.println(AZ_Degrees);
  }
  // Send it back via serial
  Serial_Send_Data = "";
  if (EL_Degrees < 100)
    // pad with 0's if needed
  {
    Serial_Send_Data = "0";
  }
  if (EL_Degrees < 10)
  {
    Serial_Send_Data = "00";
  }
  Serial_Send_Data = "+0" + Serial_Send_Data + String(EL_Degrees);
  // Send the Elevation String
  Serial.println(Serial_Send_Data);
  // Return value via RS-232 port
}
//--------------------------------------------------------------
void rotate_az_el_to() // Rotate Azimuth and Elevation to Set Point Function
{
  if (debug_mode)
  {
    Serial.println("W Command - Rotate Azimuth and Elevation To Function");
  }
  // Decode Command - Format Wxxx yyy where xxx = Azimuth to Move to yyy = Elevation to Move to
  if (debug_mode)
  {
    Serial.print("serial_buffer_index=");    
    Serial.println(serial_buffer_index);
  }
  if (serial_buffer_index == 8) // Verify the command is the proper length
  {
    if (debug_mode)
    {
      Serial.println("Value in [1] to [3]?");
    }
    Requested_AZ = (String(char(serial_buffer[1])) + String(char(serial_buffer[2])) + String(char(serial_buffer[3]))) ;
    // Decode the Azimuth portion of the command
    AZ_To = (Requested_AZ.toInt());
    // AZ Degrees to Move to as integer
    if (AZ_To < 0) // Don't allow moving below zero
    {
      AZ_To = 0;
    }
    if (AZ_To > 360) // Don't allow moving above 360
    {
      AZ_To = 360;
    }
//    if (AZ_To > 180) // Adjust for Azimuth starting at Center
//    {
//      AZ_To = AZ_To - 180;
//    }
//    else
//    {
//      AZ_To = AZ_To + 180;
//    }
    if (debug_mode)
    {
      Serial.println(Requested_AZ);
      Serial.println(AZ_To);
      Serial.println("Value in [5] to [7]?");
    }
    Requested_EL = (String(char(serial_buffer[5])) + String(char(serial_buffer[6])) + String(char(serial_buffer[7]))) ;
    // Decode the Elevation portion of the command
    EL_To = (Requested_EL.toInt());
    // EL Degrees to Move to as integer
    if (EL_To < 0) // Don't allow moving below zero
    {
      EL_To = 0;
    }
    if (EL_To > 90) // Don't allow moving above 90
    {
      EL_To = 90;
    }
    if (debug_mode)
    {
      Serial.println(Requested_EL);
      Serial.println(EL_To);
    }
    // set the move flag and start
    read_adc();
    // Read the ADC // Map it to degrees
    if (debug_mode) {
      Serial.print("current_AZ=");      
      Serial.println(current_AZ);
    }
    AZ_Degrees = map(current_AZ, AZ_0, AZ_MAX, 0, 360);
    // Map the Azimuth Value to Degrees
    if (debug_mode)
    {
      Serial.print("AZ_Degrees=");      
      Serial.println(AZ_Degrees);
    }
    AZ_Distance = AZ_To - AZ_Degrees;
    // Figure how far to move Azimuth
    set_AZ = AZ_To;
    EL_Degrees = map(current_EL, EL_0, EL_MAX, 0, 90);
    // Map the Elevation Value to Degrees
    if (debug_mode) 
    {
      Serial.print("EL_Degrees=");      
      Serial.println(EL_Degrees);
    }
    EL_Distance = EL_To - EL_Degrees;
    // Figure how far to move Elevation
    set_EL = EL_To;
    // Set Azimuth
    if (abs(AZ_Distance) <= AZ_Tolerance)
      // No move needed if we're within tolerance range
    {
      az_rotate_stop(); // Stop the Azimuth Rotation
      set_AZ = -1; // Turn off the Azimuth Move Command
    }
    else
    {
      // Move Azimuth - figure out which way
      if (AZ_Distance > 0) //We need to move CW
      {
        rotate_az_cw();
        // Rotate CW if positive
      }
      else
      {
        rotate_az_ccw(); // Rotate CCW if negative
      }
    }
    // Set Elevation
    if (abs(EL_Distance) <= EL_Tolerance)
      // No move needed if we're within tolerance range
    {
      el_rotate_stop();
      // Stop the Elevation Rotation
      set_EL = -1;
      // Turn off the Elevation Move Command
    }
    else
    {
      // Move Elevation - figure out which way
      if (EL_Distance > 0)
        //We need to move CW
      {
        rotate_el_up();
        // Rotate Up if positive
      }
      else
      {
        rotate_el_down();
        // Rotate Down if negative
      }
    }
  }
}
//----------------------------------------------------------------
void set_0_az_cal() // Set Azimuth Zero Calibration
{
  if (debug_mode)
  {
    Serial.println("Calibrate Zero Function");
  }
  read_adc();
  // Read the ADC // save current az and el values to EEPROM - Zero Calibration
  if (debug_mode)
  {
    Serial.println(current_EL);
    Serial.println(current_AZ);
  }
  AZ_0 = current_AZ;
  // Set the Azimuth Zero Calibration to current position
  write_eeprom_cal_data();
  // Write the Calibration Data to EEPROM
  if (debug_mode)
  {
    Serial.println("Zero Azimuth Calibration Complete");
  }
}
//------------------------------------------------------------------------
void set_0_el_cal() // Set the Elevation Zero Calibration
{
  if (debug_mode)
  {
    Serial.println("Calibrate Zero Function");
  }
  read_adc();
  // Read the ADC
  // save current az and el values to EEPROM - Zero Calibration
  if (debug_mode)
  {
    Serial.println(current_EL);
    Serial.println(current_AZ);
  }
  EL_0 = current_EL;
  // Set the Elevation Zero Calibration to current position
  write_eeprom_cal_data();
  // Write the Calibration Data to EEPROM
  if (debug_mode)
  {
    Serial.println("Zero Elevation Calibration Complete");
  }
}
//---------------------------------------------------------------------
void check_move()
// Check to see if we've been commanded to move
{
  Serial.println ("Starting check_move");
  Serial.println ("set_AZ="+String(set_AZ));
  Serial.println ("set_EL="+String(set_EL));
  delay (2000);
  if (set_AZ != -1 || set_EL != -1)
  {
    // We're moving - check and stop as needed
    read_adc();
    // Read the ADC
    // Map AZ to degrees
    AZ_Degrees = map(current_AZ, AZ_0, AZ_MAX, 0, 360); 
    // Map the Current Azimuth reading to Degrees
    // Map EL to degrees
    EL_Degrees = map(current_EL, EL_0, EL_MAX, 0, 90);  
    // Map the Current Elevation to Degrees
    if (debug_mode)
    {
      Serial.println("Check_move");
      Serial.print("Moving current EL_Degrees=");
      Serial.println(EL_Degrees);
      Serial.print("Moving current AZ_Degrees=");
      Serial.println(AZ_Degrees);
    }
    if (set_AZ != -1)
      // If Azimuth is moving
    {
      AZ_Distance = set_AZ - AZ_Degrees;
      Serial.println ("AZ distance="+String(AZ_Distance));
      // Check how far we have to move
      if (abs(AZ_Distance) <= AZ_Tolerance)
        // No move needed if we're within the tolerance range
      {
        az_rotate_stop(); // Stop the Azimuth Rotation
        set_AZ = -1; // Turn off the Azimuth Move Command
      }
      else
      {
        // Move Azimuth - figure out which way
        if (AZ_Distance > 0) //We need to move CW
        {
          rotate_az_cw(); // Rotate CW if positive
        }
        else
        {
          rotate_az_ccw(); // Rotate CCW if negative
        }
      }
    }
    if (set_EL != -1) // If Elevation is moving
    {
      EL_Distance = set_EL - EL_Degrees;
      // Check how far we have to move
      if (abs(EL_Distance) <= EL_Tolerance)
      {
        // No move needed if we're within tolerance range
        el_rotate_stop(); // Stop the Elevation Rotation
        set_EL = -1; // Turn off the Elevation Move Command
      }
      else
      {
        // Move Azimuth - figure out which way
        if (EL_Distance > 0)
          //We need to move CW
        {
          rotate_el_up();
          // Rotate Up if positive
        }
        else
        {
          rotate_el_down(); // Rotate Down if negative
        }
      }
    }
  }
}


//----------------------------------------------------------------------------
void check_serial() // Function to check for data on the Serial port
{
  if (Serial.available() > 0) // Get the Serial Data if available
  {
    inByte = Serial.read(); // Get the Serial Data
    // You may need to comment out the following line if your PC software
    // will not communicate properly with the controller
    // SatPC32 wants the command echoed, Ham Radio Deluxe does not
    Serial.print(char(inByte));
    // Echo back to the PC
    if (inByte == 10) // ignore Line Feeds
    {
      return;
    }
    if (inByte != 13) // Add to buffer if not CR
    {
      serial_buffer[serial_buffer_index] = inByte;
      if (debug_mode) // Print the Character received if in Debug mode
      {
        Serial.print("Received = ");
        Serial.println(serial_buffer[serial_buffer_index]);
      }
      serial_buffer_index++; // Increment the Serial Buffer pointer
    }
    else
    {
      // It's a Carriage Return, execute command
      if ((serial_buffer[0] > 96) && (serial_buffer[0] < 123))
        //If first character of command is lowercase, convert to uppercase
      {
        serial_buffer[0] = serial_buffer[0] - 32;
      }
      switch (serial_buffer[0])
      {
        // Decode first character of command
        case 65: // A Command - Stop the Azimuth Rotation
          if (debug_mode) {
            Serial.println("A Command Received");
          }
          az_rotate_stop();
          break;

        case 66:
          // B Command - Send the Current Elevation to the PC
          if (debug_mode) {
            Serial.println("B Command Received");
          }
          send_current_el();
          // Call the Send Current Elevation Function
          break;
          
        case 67:
          // C - return current azimuth if (debug_mode) // Return the Buffer Index Pointer in Debug Mode
          {
            Serial.println("C Command Received");
            Serial.println(serial_buffer_index);
          }
          if ((serial_buffer_index == 2) & (serial_buffer[1] == 50))
          {
            if (debug_mode) {
              Serial.println("C2 Command Received");
            }
            send_current_azel(); // Return Azimuth and Elevation if C2 Command
          }
          else
          {
            send_current_az(); // Return Azimuth if C Command
          }
          break;

        case 68: // D - Rotate Elevation Down Command
          if (debug_mode)
          {
            Serial.println("D Command Received");
          }
          rotate_el_down();
          // Call the Rotate Elevation Down Function
          break;
          
        case 69: // E - Stop the Elevation Rotation
          if (debug_mode)
          {
            Serial.println("E Command Received");
          }
          el_rotate_stop(); // Call the Elevation Rotation Stop Function
          break;
          
        case 70: // F - Set the Max Calibration
          if (debug_mode)
          {
            Serial.println("F Command Received");
            Serial.println(serial_buffer_index);
          }
          if ((serial_buffer_index == 2) & (serial_buffer[1] == 50))
            // Check for F2 Command
          {
            if (debug_mode) {
              Serial.println("F2 Command Received");
            }
            set_max_el_cal(); // F2 - Set the Max Elevation Calibration
          }
          else
          {
            set_max_az_cal();
            // F - Set the Max Azimuth Calibration
          }
          break;
          
        case 76: // L - Rotate Azimuth CCW
          if (debug_mode) {
            Serial.println("L Command Received");
          }
          rotate_az_ccw();
          // Call the Rotate Azimuth CCW Function
          break;
          
        case 77: // M - Rotate to Set Point
          if (debug_mode)
          {
            Serial.println("M Command Received");
          }
          rotate_to(); // Call the Rotate to Set Point Command
          break;
          
        case 79:
          // O - Set Zero Calibration
          if (debug_mode)
          {
            Serial.println("O Command Received");
            Serial.println(serial_buffer_index);
          }
          if ((serial_buffer_index == 2) & (serial_buffer[1] == 50))
            // Check for O2 Command
          {
            if (debug_mode)
            {
              Serial.println("O2 Command Received");
            }
            set_0_el_cal();
            // O2 - Set the Elevation Zero Calibration
          }
          else {
            set_0_az_cal();
            // O - Set the Azimuth Zero Calibration
          }
          break;
          
        case 82: // R - Rotate Azimuth CW
          if (debug_mode)
          {
            Serial.println("R Command Received");
          }
          rotate_az_cw(); // Call the Rotate Azimuth CW Function
          break;
          
        case 83: // S - Stop All Rotation
          if (debug_mode)
          {
            Serial.println("S Command Received");
          }
          az_rotate_stop(); // Call the Stop Azimith Rotation Function
          el_rotate_stop(); // Call the Stop Elevation Rotation Function
          break;
          
        case 85: // U - Rotate Elevation Up
          if (debug_mode)
          {
            Serial.println("U Command Received");
          }
          rotate_el_up(); // Call the Rotate Elevation Up Function
          break;
          
        case 87: // W - Rotate Azimuth and Elevation to Set Point
          if (debug_mode)
          {
            Serial.println("W Command Received");
          }
          rotate_az_el_to();
          // Call the Rotate Azimuth and Elevation to Set Point Function
          break;
      }
      serial_buffer_index = 0;
      // Clear the Serial Buffer and Reset the Buffer Index Pointer
      serial_buffer[0] = 0;
    }
  }
}



// end of previous program
//void check_radio()   /****** LOOP: RUNS CONSTANTLY ******/
//{
//  if ( myRadio.available()) // Check for incoming data from transmitter
//  {
//    while (myRadio.available())  // While there is data ready
//    {
//      myRadio.read( &dataReceived, sizeof(dataReceived) ); // Get the data payload (You must have defined that already!)
//    }
//    // DO something with the data, like print it
//    Serial.print("Data received = ");
//    Serial.println(dataReceived);
//
//// Value received is positive - azimuth motion
//   if (dataReceived < 500 && dataReceived > 50) {digitalWrite(rotate_left, HIGH);}
//   if (dataReceived > 700) {digitalWrite(rotate_right, HIGH);}
//   if (dataReceived > 500 && dataReceived < 700) // stop rotation
//    { 
//    digitalWrite(rotate_left, LOW);
//    digitalWrite(rotate_right, LOW);  
//    }     
//
//// Value received is negative - elevation motion
//   if (dataReceived > -500 && dataReceived < -50) {digitalWrite(rotate_down, HIGH);}
//   if (dataReceived < -700) {digitalWrite(rotate_up, HIGH);}
//   if (dataReceived < -500 && dataReceived > -700) // stop rotation
//    { 
//    digitalWrite(rotate_up, LOW);
//    digitalWrite(rotate_down, LOW);  
//    }
//    
//  } //END Radio available
//}//--(end main loop )---
