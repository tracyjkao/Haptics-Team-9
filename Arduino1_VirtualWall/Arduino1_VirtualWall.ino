// HAPTIC PROJECT TEAM 9
// ARDUINO #1
//
// This Arduino renders the virtual wall sensation of the steering wheel based on the
// position of the vehicle and also actates the left vibration motor.
// This Arduino receives and transmits information to and from Arduino #1.
//
// Inputs (from Arduino #2):
//    - KartXPosition (mapped from 0 to 255)  // to go to the Virtual Wall
//    - Vibration Command to Left Motor       // to go to the Fan
// Outputs (to Arduino #2):
//    - Steer (0 or 1)
//
// COM PORT: # COM 5
//

// Libraries used
#include <math.h>
#include <String.h>

// Important Constants
#define PI 3.1415926535897932384626433832795

// #define ENABLE_VIRTUAL_WALL


// Left Vibration Motor Pins
int vibrLCommandPin = 9; // receives vibration command from Arduino 2's Pin 9
int vibrLPwmPin = 6; // Left Vibration Motor
int vibrLDirPin = 7;

// Position Pin Declarations
// int posPin = A5;    // Pin to receive Position Input from Arduino 2's Pin 3
int posPin = A1;

// Steering Pin Declarations
int steerPin = 13;  // Pin to output steering information to Arduino 2's Pin 13

// Arduino-Arduino Communication Variables
int steersignal = 0;    // Range 0 to 255 (to send)
int xposnorm = 0;       // Range 0 to 255 (received)
float xposition = 0;    // Range -1 to 1 (transformed)
float road_middle = 0;  // Based on range -1 to 1 (preset)
float frequency = 490 ;   // Hz

// Motor Pin declares
int pwmPin = 5;         // PWM output pin for motor 1
int dirPin = 8;         // direction output pin for motor 1
int sensorPosPin = A2;  // input pin for MR sensor
int fsrPin = A3;        // input pin for FSR sensor

// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading

// --------------------------------------------------------------
// change for calibration
// --------------------------------------------------------------
double updatePos_mid = 167;
// --------------------------------------------------------------

int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;
double OFFSET = 980;
double OFFSET_NEG = 15;
//double x_wallr = 200;
//double x_walll = -400;

double k = 3;            // constant associated with the virtual wall

// Kinematics variables
volatile double xh;           // position of the handle [m]
double theta_s = 0;      // Angle of the sector pulley in deg
double dxh;              // Velocity of the handle
double xh_prev;          // Distance of the handle at previous time step
double xh_prev2;
double dxh_prev;
double dxh_prev2;
double dxh_filt;         // Filtered velocity of the handle
double dxh_filt_prev;
double dxh_filt_prev2;

//Constants for velocity filtering
double ao = 2500;    //wn^2 (where wn=50)
double a1 = 100;     //2*zeta*wn (where zeta=1)
double T = 0.0006; //sample period [sec]
double A = 1 / ((2 * ao*T*T) + 2 * a1 * T + 4);
double B = 4 - 2 * a1 * T + ao * T * T;
double C = 8 - 2 * ao * T * T;
double D = ao * T * T;

// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor

// --------------------------------------------------------------
// Setup function
// --------------------------------------------------------------

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // --------------------------------------------------------------
  //  Virtual Wall Pin Setup
  // --------------------------------------------------------------
  // Set PWM frequency
  setPwmFrequency(pwmPin, 1);

  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);       // set FSR sensor pin to be an input

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A

  // Initialize motor
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction

  // Initialize position valiables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);
  flipNumber = 0;


  // ------------------------------------------------------------
  //  Left Vibration Motor Pin Setup
  // ------------------------------------------------------------
  pinMode(vibrLCommandPin, INPUT);  // receives vibration command from Arduino 2
  pinMode(vibrLPwmPin, OUTPUT);     // L Vibration Motor PWM
  pinMode(vibrLDirPin, OUTPUT);     // L Vibration Motor Dir

  digitalWrite(vibrLDirPin, LOW);
  analogWrite(vibrLPwmPin, 0);
  
  
  // -------------------------------------------------------------
  //  Kart X Position Pin Setup
  // -------------------------------------------------------------
  pinMode(posPin, INPUT);

  // -------------------------------------------------------------
  // Kart Steering Pin Setup
  // ------------------------------------------------------------
  pinMode(steerPin, OUTPUT);  
  digitalWrite(steerPin, LOW);
}


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{

  //*************************************************************
  //*** Section 1. Compute position in counts *******************
  //*************************************************************

  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);

  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;

  // Keep track of flips over 180 degrees
  if ((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if (lastRawDiff > 0) {       // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    flipped = false;
  }
  updatedPos = rawPos + flipNumber * OFFSET; // need to update pos based on what most recent offset is


  //*************************************************************
  //*** Section 2. Compute position in meters *******************
  //*************************************************************

  // ADD YOUR CODE HERE
  // Define kinematic parameters you may need
  double rh = 0.1;   //[m]
  // print updatedPos via serial monitor
  // Serial.println(updatedPos);

  double ts = (updatedPos - updatePos_mid) / 6.127; // steeringwheel calibration function
  double ts_rad = ts * PI / 180;
  double xh = ts_rad * rh; // Compute the position of the handle (in meters) based on ts (in radians)
  Serial.println(xh);

  // Calculate the velocity dxh of the handle
  // Also add any filter you would like here, you may use the provided one below
  dxh = (double)(xh - xh_prev) / 0.001;

  // Calculate the filtered velocity of the handle using a second-order low pass filter (derived from anolog H(s)=ao/(s^2+a1s+ao))
  //dxh_filt = A*(-B*dxh_filt_prev2 + C*dxh_filt_prev + D*(dxh + 2*dxh_prev + dxh_prev2));

  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  // dxh_filt = .9*dxh + 0.1*dxh_prev;

  // Record the position and velocity
  xh_prev2 = xh_prev;
  xh_prev = xh;

  dxh_prev2 = dxh_prev;
  dxh_prev = dxh;

  //dxh_filt_prev2 = dxh_filt_prev;
  //dxh_filt_prev = dxh_filt;

  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******
  //*************************************************************

  // Define kinematic parameters you may need
  double rp = 0.004191;   //[m]
  double rs = 0.1905;   //[m]

  //*************************************************************
  //******************* Rendering Algorithms ********************
  //*************************************************************

  // Forces algorithms
#ifdef ENABLE_VIRTUAL_WALL
  xposition = xh;
  if (xh > -1.3 && xh < 1.3)
  {
    //    force = -exp(k)/10*(xh-road_middle);
    force = -k * exp(2 * abs(xposition - road_middle)) * (xposition - road_middle) / abs(xh - road_middle);
    //      force = 0;
  }
  else
  {
    force = 0;
  }

#endif

  double Tp = (force * rh * rp) / rs; // Compute the require motor pulley torque (Tp) to generate that force
  //*************************************************************
  //*** Section 4. Force output  ********************************
  //*************************************************************

  // Determine correct direction for motor torque
  if (force > 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp) / 0.0183);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {
    duty = 1;
  } else if (duty < 0) {
    duty = 0;
  }
  output = (int)(duty * 255);  // convert duty cycle to output signal
  analogWrite(pwmPin, output); // output the signal

  // ***********************************************************
  // *** Section 5. Write Steering informatiion to Arduino 2 *****
  // ***********************************************************
  // Serial.println(xh);
  writeSteerToArduino2(xh);

  // ************************************************************
  // *** Section 6. Check Command for Actuating L Vibration Motor
  // ************************************************************
  checkVibrateLeft();


}

// --------------------------------------------------------------
// Function to set PWM Freq
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

// ------------------------------------------------------------
// Vibrate Left Vibration Motor
// ------------------------------------------------------------
void checkVibrateLeft()
{
  if (digitalRead(vibrLCommandPin) == HIGH) {
    // vibrate left vibration motor
    digitalWrite(vibrLDirPin, HIGH);
    analogWrite(vibrLPwmPin, 100);  // Actuate the vibration
    // delay(10);
  } else
  // if (digitalRead(vibrLCommandPin) == 0
  {
    digitalWrite(vibrLDirPin, LOW);
    analogWrite(vibrLPwmPin, 0); // Stop the vibration
    // delay(10);
  }
}

// -------------------------------------------------------------
// Receive Kart X Position from Arduino 1
// -------------------------------------------------------------
void readforXPos()
{
  // Read the XPos from Arduino 2
  xposnorm = analogRead(posPin);              // range is 0 to 255
  xposition = ((xposnorm) / 255 * 2) - 1;   // range is -1 to 1
  // xposition will be fed into the Virtual Wall
}


// --------------------------------------------------------------
// Send steering information to Arduino 2
// --------------------------------------------------------------
void writeSteerToArduino2(double xh)
{
  // digitalImplementation
  if (xh < 0){
    digitalWrite(steerPin, HIGH); // HIGH signal to turn left
  } else if (xh > 0) {
    // if xh > 0
    digitalWrite(steerPin, LOW); // lOW signal to turn right
  } else {
    // undefined for xh = 0;
    // continue whatever it was on
  }
  // Analog signal:
  // steersignal = (xh - (-1)) / (1 - (-1)) * 255;   // steer signal is from 0 to 255
  // analogWrite(steerPin, steersignal);     // Send steer signal to Arduino 2
}

// Outputs the duty cycle associated ith the PWM signal read (in percent)
// range is 0 to 1
//float analogReadPWM(int pin)
//{
//  float pwm_value = pulseIn(pin, HIGH); // length of time rectangular pulse is on
//  float period = 1 / frequency;
//  float dutycycle = pwm_value / period;
//  return dutycycle;
//}

//float analogReadPWM(int inputPin)
//{
//  unsigned long highTime = pulseIn(inputPin, HIGH);
//  unsigned long lowTime = pulseIn(inputPin, LOW);
//  long cycleTime = highTime + lowTime;
//  float dutycycle = (float) highTime / (float) cycleTime;
//  return dutycycle;
//  
//}
