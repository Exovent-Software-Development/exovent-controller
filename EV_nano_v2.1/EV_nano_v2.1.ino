// Re-do arduino code for Exovent testing
// NJR 09-06-21
// --------------------------------------
// This code is written for an Arduino Nano

// Input hardware: A0 to A3 = 4 potentiometers for general input
// Input A4 and A5 are for the I2C interface for the LCD display
// A6 = pressure transducer for feedback, working with an MPX5010DP differential pressure Tx
// Input D2 for cycling
// Input D3 for On off of pressure control
// Output D6 for the valve servo control
// Output D7 reserved for pump motor speed control


#include <Wire.h>                    // Library for I2C Comms
#include <LiquidCrystal_I2C.h>       // Library for LCF
#include <Servo.h>                   // Library for Servo (for valve)


// Declarations ---------------------------------------
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);    // Set up LCD object
Servo servo;                                               // Set up servo object

//Set up analog pins
int lowPin = A3;
int highPin = A2;
int freqPin = A1;
int inExPin = A0;
int pressurePin = A6;

//Set up digital pins
int onOffPin = 3;
int cyclingPin = 2;

//Variables controlled by potentiometers
long int period = 0;
long int periodOpen = 2000;
int inExVal = 0;            
long int lowVal = 0;
long int highVal = 50;

//Variables that are involved with positioning the valve
//long int valvePos = 0;
long int posMax = 130;
long int posMin = 160;
int valveMax = 90;
int valveMin = 175;
long int desPos = 150;

//Variables involved with timing
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long switchTime = 0;
unsigned long debounce = 500;
int BPM = 10;

//Variables involved with pressure
long int pressure = 0;
long int pDiffMax = 0;
long int pDiffMin = 0;
long int lowPress = 0;

bool Cycling = false; //Ventilator defaults to off.
bool VenOn = false;

void setup() {
  pinMode(freqPin, INPUT);
  pinMode(inExPin, INPUT);
  pinMode(lowPin, INPUT);
  pinMode(highPin, INPUT);
  pinMode(pressurePin, INPUT);
  pinMode(onOffPin, INPUT_PULLUP);
  pinMode(cyclingPin, INPUT_PULLUP);
  
  Serial.begin(115200);
  servo.attach(5); //Servo motor connected to pin 9.

 // Initiate the LCD:
  lcd.init();
  lcd.backlight();  
  lcd.setCursor(0, 0); // Set the cursor on the third column and first row.
  lcd.print("Steer Energy");
  lcd.setCursor(0, 1); //Set the cursor on the third column and the second row (counting starts at 0!).
  lcd.print("Exovent Controller");
  delay(2000);

  lowPress = (map(analogRead(pressurePin), 41, 962, 0, 1000)) + 20; // map pressure from 41 to 0 and from 962 to 1000
  posMax = 130;
  posMin = 160;
  setupDisplay();
}
//-------------------------------------------------------------------------------

void setupDisplay(){
  lcd.setCursor(0,0);
  lcd.print("pMin:    ");
  lcd.print("pMax:  ");
  lcd.setCursor(0,1);
  lcd.print(" BPM:     ");
  lcd.print("I/E:  ");
  lcd.print("%   ");
  lcd.setCursor(0,2);
  lcd.print("ValvePos:   ");
  lcd.setCursor(0,3);
  lcd.print("p:  ");

}

/*--------------------------------------------------------------------------------------
  Main Programme
-------------------------------------------------------------------------------------*/

void loop() {
  currentTime = millis(); 
  Switch();
  Mapping();
  ReadOut();
  if(VenOn == true){
    Move();
    if((currentTime - previousTime) >= period) //Runs once every cycle
    {
      AdjustPos();
      previousTime = currentTime;
    }
  }
}

/*---------------------------------------------------------------------------
  Function that controls the switch to turn everything on/off.
---------------------------------------------------------------------------*/
void Switch(){
  if(digitalRead(onOffPin) == LOW)
  {
    VenOn = true;
    if(digitalRead(cyclingPin) == LOW)
    {
      Cycling = true;
    }
    else
    {
      Cycling = false;
    }
  }
  else
  {
    VenOn = false;
    servo.write(valveMin);
    posMin = valveMin;
    posMax = valveMin;
    pDiffMax = 0;
    pDiffMin = 0;
  }
}

/*----------------------------------------------------------------------------
 This function prints multiple variables to the serial monitor
 Useful for seeing the exact values of the potentiometers and debugging
-----------------------------------------------------------------------------*/
void ReadOut(){
  //lcd.clear();
  
  Serial.print(inExVal);
  Serial.print("  ");
  Serial.print(period);
  Serial.print("  ");
  Serial.print(lowVal/10);
  Serial.print("  ");
  Serial.print(highVal/10);
  Serial.print(analogRead(pressurePin));
  Serial.print("  ");
  Serial.println(pressure/10);
  Serial.print("  ");
  Serial.print(desPos);
  Serial.print("  ");
  Serial.print(posMax);
  Serial.print("  ");
  Serial.print(posMin);
  Serial.print("  ");
  Serial.println(pDiffMin);
// --- LCD Row 1  
  lcd.setCursor(5,0);
  lcd.print(lowVal/10);
  if(lowVal/10 < 10){
    lcd.print(" ");
  }
  lcd.setCursor(14,0);
  lcd.print(highVal/10);
  if(highVal/10 < 10){
    lcd.print(" ");
  }
// LCD Row 2  
  lcd.setCursor(5,1);
  lcd.print(BPM);
  lcd.setCursor(14,1);
  lcd.print(inExVal);
// LCD Row 3
  lcd.setCursor(9,2);
  lcd.print(servo.read());
  if(servo.read() < 100){
    lcd.print(" ");
  }
  if(servo.read() < 10){
    lcd.print(" ");
  }
// LCD Row 4
  lcd.setCursor(2,3);
  lcd.print(pressure/10);
  if(pressure/10 < 10){
    lcd.print(" ");
  }
  lcd.setCursor(8,3);
  if(VenOn == false){
    lcd.print("Vent  Off  ");    
  }
  else if(Cycling == true){
    lcd.print("Cycling    ");
  }
  else{
    lcd.print("Match PMin");
  }
}

/*--------------------------------------------------------------------
  Function maps analog values to the values that are more useful
--------------------------------------------------------------------*/
void Mapping(){
  period = map(analogRead(freqPin),0,950,6000,2000);  //freqPin mapped to 2000 - 10000 milliseconds
  if(period < 2000){period = 2000;}
  period = (period/10)*10;
  
  BPM = map(period,6000,2000,10,30);
  
  inExVal = map(analogRead(inExPin),0,950,20,50);     //inExPin mapped to a 20 - 50 % on to off
  if(inExVal > 50){inExVal = 50;}
  periodOpen = (inExVal*period)/100;
  
  pressure = map(analogRead(pressurePin), 41, 962, 0, 1000); //pressure mapped to 0 - 100 millibar
  
  highVal = map(analogRead(highPin),0,950,50,500); //highPin mapped to 5 - 50 millibar
  if(highVal > 500){highVal = 500;}
  
  lowVal = map(analogRead(lowPin),0,950,0,150); //lowPin mapped to 0 - 15 millibar
  if(lowVal > 150){lowVal = 150;}
  
  if(Cycling == false || highVal < lowVal)
  {
    period = 250;
  }
}

/*--------------------------------------------------------------------
  Function controls the stepping of the stepper motor
  Also controls whether its an in breath or out breath
--------------------------------------------------------------------*/
void Move() {
  //On the in breath, the valve is told to go towards the max position calculated in AdjustPos function
  if(Cycling == true && highVal > lowVal)
  {
    if((currentTime - previousTime) <= periodOpen)
    {
      desPos = posMax;
      pDiffMax = highVal - pressure;
    }
    else
    {
      desPos = posMin;
      pDiffMin = pressure - lowVal;
    }
  }
  //On the out breath, the valve is told to go towards the min position calculated in AdjustPos function
  else
  { 
    posMax = posMin;
    pDiffMax = 0;
    desPos = posMin;
    pDiffMin = pressure - lowVal;
  }
  
  //Moves servo to desired position
  if(!(servo.read() == desPos))
  {
    servo.write(desPos);
  }
}

/*----------------------------------------------------------------------------------------------------
  Function adjusts the values of posMax and posMin (where the valve aims for on the in and out breath)
  Values are adjusted relative to the difference between the desired pressure and the actual pressure
  for the max position and min position last breathing cycle.
  Also makes sure posMax and posMin don't go above 500 or below 0 respectively.
----------------------------------------------------------------------------------------------------*/
void AdjustPos() {
  posMax -= map(pDiffMax,-500,500,-45,45);
  if(posMax < valveMax) {posMax = valveMax;}
  if(posMax > valveMin) {posMax = valveMin;}
  posMin += map(pDiffMin,-500,500,-45,45);
  if(posMin > valveMin) {posMin = valveMin;}
  if(posMin < valveMax) {posMin = valveMax;}
}
