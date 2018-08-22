
#include "Adafruit_MAX31855.h"
#include <LiquidCrystal.h>
#include "PID_v1.h"
//#include <EEPROM.h>

int thermoCLK = 16;
int thermoDO = 17;
int thermoCS1 = 18;
int thermoCS2 = 19;

// Initialize the Thermocouples
Adafruit_MAX31855 thermocouple1(thermoCLK, thermoCS1, thermoDO);
Adafruit_MAX31855 thermocouple2(thermoCLK, thermoCS2, thermoDO);

int BedSW = 3;   //Red LED GND activates
int PumpSW = 11;  //Blue LED GND activates

double SetPsw;
double SetBsw;

double Setpoint;
double Input;
double Output;
double Poutput;
 
// pid tuning parameters
double Kp = 100.0;
double Ki = 1.0;
double Kd = 5.0;
 
//// EEPROM addresses for persisted data
//const int SpAddress = 0;
//const int KpAddress = 8;
//const int KiAddress = 16;
//const int KdAddress = 24;
 
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

byte dot[8] = {
  B00000,
  B00000,
  B01110,
  B11111,
  B01110,
  B00000,
  B00000,
};

byte degree[8] = {
  B00110,
  B01001,
  B00110,
  B00000,
  B00000,
  B00000,
  B00000,
};

byte target[8] = {
  B00100,
  B01110,
  B10101,
  B11111,
  B10101,
  B01110,
  B00100,
};

byte leftarrow[8] = {
  B00000,
  B00100,
  B01100,
  B11111,
  B01100,
  B00100,
  B00000,
};

byte rightarrow[8] = {
  B00000,
  B00100,
  B00110,
  B11111,
  B00110,
  B00100,
  B00000,
};

//byte uparrow[8] = {
//  B00100,
//  B01110,
//  B11111,
//  B00100,
//  B00100,
//  B00100,
//  B00100,
//};

byte downarrow[8] = {
  B00100,
  B00100,
  B00100,
  B00100,
  B11111,
  B01110,
  B00100,
};

byte negitiveB[8] = {
  B11111,
  B10011,
  B10101,
  B10011,
  B10101,
  B10011,
  B11111,
};

byte negitiveP[8] = {
  B11111,
  B10011,
  B10101,
  B10011,
  B10111,
  B10111,
  B11111,
};

boolean EditMode = false; //Edit mode vs Run mode, default (false) is Run mode

boolean Benable = false; //On/Off switch for Bed, default (false) is off
boolean Penable = false; //On/Off switch for Pump, default (false) is off

boolean Evalue = false; //Bed/Pump flag to indicate which value is being edited, default (false) is Bed

double Bvalue = 25.0;
double Pvalue = 25.0;

double AdjAmt = 5;

double BedTempC = 0;
double PumpTempC = 0;

unsigned long Timeout = 10000;
unsigned long CurTime = 0;

// define some values used by the panel and buttons
int lcd_key     = 0;
int adc_key_in  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

// read the buttons function
int read_LCD_buttons()
{
 adc_key_in = analogRead(0);      // read the value from the sensor 
 // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
 // we add approx 50 to those values and check to see if we are close
 if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
 if (adc_key_in < 50)   return btnRIGHT;  
 if (adc_key_in < 250)  return btnUP; 
 if (adc_key_in < 450)  return btnDOWN; 
 if (adc_key_in < 650)  return btnLEFT; 
 if (adc_key_in < 850)  return btnSELECT;  

 return btnNONE;  // when all others fail, return this...
}
  
void setup() {
  
  //Serial.begin(9600);
  
  // Create special charators
  lcd.createChar(0, dot);
  lcd.createChar(1, degree);
  lcd.createChar(2, target);
  lcd.createChar(3, leftarrow);
  lcd.createChar(4, rightarrow);
  lcd.createChar(5, downarrow);
//  lcd.createChar(6, uparrow);
  lcd.createChar(7, negitiveB);
  lcd.createChar(8, negitiveP);
  
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  
  // Initiate PID
  myPID.SetTunings(Kp,Ki,Kd);
  myPID.SetMode(AUTOMATIC);
  
  // Intro splash screen on display
  lcd.setCursor(0, 0);
  lcd.print("Bioprinter");
  lcd.setCursor(0, 1);
  lcd.print("Temp Control");
  
  // wait for MAX chip to stabilize
  //delay(500);
  
  //SetPsw = thermocouple2.readInternal();
  
  // Pause splash screen for 2 secs
  delay(2000);
  
  // Clear screen
  lcd.clear();
  
}

void loop() {

    //Reading Bed Thermocouple
    
     //Setpoint = thermocouple1.readInternal();
     Setpoint = Bvalue;                                  // Set the PID target to adjusted Bed target

     double BedTempC = thermocouple1.readCelsius();      // Read Bed thermocouple
     
     Input = BedTempC;
     
     myPID.Compute();                                    // Caclulate how high to turn Bed on
     
     if (Benable)
     {
       analogWrite(BedSW, Output);                          // Turn Bed on this amount
     }
     else
     {
       analogWrite(BedSW, 0);                          // Turn Bed off
     }

     //Reading Pump Thermocouple
     
     //SetPsw = thermocouple2.readInternal() + 2.0;
     SetPsw = Pvalue;                                    // Set the PID target to adjusted Pump target
     
     double PumpTempC = thermocouple2.readCelsius();     // Read Bed thermocouple
     
     if (Penable)
     {
       if(PumpTempC > SetPsw)                              // Temp above limit
       {
         //digitalWrite(PumpSWPumpSW,LOW);
         //digitalWrite(PumpSW,HIGH);
         Poutput = 255;                                      // Set Pump output to max
         analogWrite(PumpSW, Poutput);                      // Turn Pump on
       }
       else                                                // Temp below limit
       {
         //digitalWrite(PumpSW,HIGH);
         //digitalWrite(PumpSW,LOW);
         Poutput = 0;                                    // Set Pump output to min
         analogWrite(PumpSW, Poutput);                      // Turn Pump off
       }
     }
     else
     {
       analogWrite(PumpSW, 0);                      // Turn Pump off
     }
  
   lcd_key = read_LCD_buttons();                         // Read the buttons
   
   switch (lcd_key)                                      // Depending on which button was pushed
   {
     case btnRIGHT:       
         if (EditMode)                                  // If in Edit Mode, select Pump value for editing, reset edit timer
         {
           Evalue = true;
           CurTime = millis();
         }
         else                                           // If not in Edit Mode, Enter Edit Mode and start edit timer
         {
           EditMode = true;
           CurTime = millis();
         }
         break;
       
     case btnLEFT:
         if (EditMode)                                  // If in Edit Mode, select Bed value for editing, reset edit timer
         {
           Evalue = false;
           CurTime = millis(); 
         }
         else                                            // If not in Edit Mode, Enter Edit Mode and start edit timer
         {
           EditMode = true;
           CurTime = millis();
         }
       break;
       
     case btnUP:
         if (EditMode)                                  // If in Edit Mode, raise selected value by amount, reset edit timer
         {
           if (Evalue)                                  // Pump value to edit
           {
             Pvalue = Pvalue + AdjAmt;
             CurTime = millis();
           }
           else                                         // Bed value to edit
           {
             Bvalue = Bvalue + AdjAmt;
             CurTime = millis();
           }
         }
         else                                            // If not in Edit Mode, toggle enable Bed
         {
           if (Benable)
           {
             Benable = false;
           }
           else
           {
             Benable = true;
           }
         }
       break;
       
     case btnDOWN:
         if (EditMode)                                  // If in Edit Mode, lower selected value by amount, reset edit timer
         {
           if (Evalue)                                  // Pump value to edit
           {
             Pvalue = Pvalue - AdjAmt;
             CurTime = millis();
           }
           else                                         // Bed value to edit
           {
             Bvalue = Bvalue - AdjAmt;
             CurTime = millis();
           } 
         }
         else                                            // If not in Edit Mode, toggle enable Pump
         {
           if (Penable)
           {
             Penable = false;
           }
           else
           {
             Penable = true;
           }
         }
       break;
       
     case btnSELECT:
         if (EditMode)                                   // If in Edit Mode, select button exits
         {
           EditMode = false;
         }
         else                                            // If not in Edit Mode, Enter Edit Mode and start edit timer
         {
           EditMode = true;
           CurTime = millis();
         }
       break;
       
       case btnNONE:
       
         if (EditMode)                                   // While in Edit Mode, exit when times up
         {
           if (millis() > CurTime + Timeout)
           {
             EditMode = false;
           }
         } 
         break; 
   }

//Start of display code
  if (EditMode)               //In edit mode for changing values
  {
    lcd.clear();              //Clear screen
    lcd.setCursor(0, 0);      //First line
    
    lcd.print(" Bed ");
    lcd.write(byte(2));       //Target symbol
    lcd.print(" ");
    if (Evalue)               //Which value is set to be changed
    {
      lcd.write(byte(4));     //Right arrow pointing to Pump
    }
    else
    {
      lcd.write(byte(3));     //Left arrow pointing to Bed
    }
    lcd.print(" Pump ");
    lcd.write(byte(2));       //Target sumbol
    
    lcd.setCursor(0, 1);      //Second Line
    
    lcd.print(Bvalue,1);
    lcd.write(byte(1));       //Degree symbol
    lcd.print("C");
    lcd.setCursor(9, 1);
    lcd.print(Pvalue,1);
    lcd.write(byte(1));       //Degree symbol
    lcd.print("C");
  } 
  else
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    if (Benable)              //Bed is enabled
      {
        lcd.write(byte(7));   //Negitive B charactor
      }
    else
      {
        lcd.print("B");       //Regular B charactor
      }
    if (Output > 0)         //Bed is active
      {
        lcd.write(byte(5));   //Down arrow
      }
    else
      {
        lcd.print(" ");       //No down arrow, just blank space
      }
    lcd.print(BedTempC,1);
    lcd.write(byte(1));       //Degree symbol
    lcd.print("C ");
    lcd.write(byte(2));       //Target symbol
    lcd.print(Bvalue,1);
    lcd.write(byte(1));       //Degree symbol
    lcd.print("C");
    
    lcd.setCursor(0, 1);
    if (Penable)              //Pump is enabled
      {
        lcd.write(byte(8));   //Negitive P charactor
      }
    else
      {
        lcd.print("P");      //Regular P charactor
      }
    if (Poutput > 0)         //Pump is active
      {
        lcd.write(byte(5));   //Down arrow
      }
    else
      {
        lcd.print(" ");       //No up arrow, just a blank space
      }
    lcd.print(PumpTempC,1);
    lcd.write(byte(1));       //Degree symbol
    lcd.print("C ");
    lcd.write(byte(2));       //Target symbol
    lcd.print(Pvalue,1);
    lcd.write(byte(1));       //Degree symbol
    lcd.print("C");
  }
  
   delay(500);                //Extra time at end of loop
}
