//#include <DueTimer.h> //or use "DueTimer.h"
#include "DueTimer.h"

////////////////////////////////////////////////////
// Used Inputs/Output pins of the Arduino Due board (3.3 Volts)

const int pinDir = 12; // Direction (Output PIN)
const int pinPwm = 3; // PWM (Output PIN)
const int pinBrake = 9; // Brake (Output PIN)

const int pinCurrent = 54; // Current sensor (Input PIN)

const int pinSensorA = 5; // Encoder-A (Input PIN)
const int pinSensorB = 6; // Encoder-B (Input PIN)

const int pinPot = 56; // Potentiometer (Input PIN)

////////////////////////////////////////////////////
// Used variable for storing positive or negative increments of encoder
volatile long int ticks = 0; // Pulse counter variable

////////////////////////////////////////////////////
///// Some physical parameters of the DC motor /////
const double J = 6.8e-3 ; //Kg.m^2 
const double Ke = 0.47 ; // Nm/A
const double R = 3.33 ; // Ohm
const double Te = 0.010 ; //Chosen sampling time

///// Variables that you will define in this exercise:
volatile double V = 0; //Applied motor voltage

void setMotorVoltage(double voltage)
{
  // Conversion V => PWM :
  int pwmval = abs(constrain(voltage, -5.0, 5.0) * 4095.0 / 5.0);  

  //establishing sign of voltage for PWM direction
  if (voltage > 0) 
    digitalWrite(pinDir, HIGH);
  else
    digitalWrite(pinDir, LOW);

  analogWrite(pinPwm, pwmval); //Write PWM amplitude
}

////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200); // Configuration of the Serial device with baudrate 115200.
  analogWriteResolution(12); // Best resolution for Arduino-Due (12 bits)
  analogReadResolution(12); // Best resolution for Arduino-Due (12 bits)
  
// setup PINs mode
  pinMode(pinDir, OUTPUT);
  pinMode(pinPwm, OUTPUT);
  pinMode(pinBrake, OUTPUT);
  pinMode(pinSensorA, INPUT_PULLUP);
  pinMode(pinSensorB, INPUT_PULLUP);
  pinMode(pinPot, INPUT);
  pinMode(pinCurrent, INPUT);
  
  digitalWrite(pinBrake, LOW); //Disable brake during this exercise.
  
// Interruptions from Encoders
  attachInterrupt(digitalPinToInterrupt(pinSensorA), sensorAInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinSensorB), sensorBInterrupt, CHANGE);

//Used for real-time sampling 
  Timer3.attachInterrupt(process).start(Te*1e6); //We use here Te (microsec)
}

////////////////////////////////////////////////////
// This is the main loop, print values in this function
void loop() {
  
  // print ticks counter
  Serial.print(ticks);
  Serial.print(" ");

  Serial.print(V);
  Serial.print(" ");
  
  //// print your variables here, for instance:
  //Serial.print( );
  
  Serial.println();
  delay(100);
  
}

////////////////////////////////////////////////////
void process()
{
// put your main code here, to run repeatedly in real-time:
V = 0;
// read potentiometer value and apply a proportionnal PWM value  
  int pot = analogRead(pinPot); //Example of reading the potentiometer value
y = ticks;
x1 = -4.81416156394158*x1_last + 0.00952771454195941*x2_last + 0.00711955896455351*x3_last + 0.0010*V + 5.81416156394158*y;
x2 = -756.929317695746*x1_last + 0.907053871121761*x2_last + 1.40113449145750*x3_last + 0.197757721017529*V + 756.929317695746*y;
x3 = -544.446481344029*x1_last + 0*x2_last + x3_last + 0*V + 544.446481344029*y;

// Compute output voltage 
// For eample : set the voltage proportionaly to the potentiometer value
  V = 5.*(pot-2048)/2048.;

  // set the motor voltage
  setMotorVoltage(V);

}

////////////////////////////////////////////////////
void sensorAInterrupt()
{
  // Called when encoder A changes its value
  int A = digitalRead(pinSensorA) ;
  int B = digitalRead(pinSensorB) ;

  if (A == B) ticks += 1 ;
  else ticks -= 1;
}


void sensorBInterrupt()
{
  // Called when encoder B changes its value
  int A = digitalRead(pinSensorA) ;
  int B = digitalRead(pinSensorB) ;

  if (B != A) ticks += 1 ;
  else ticks -= 1;
}
