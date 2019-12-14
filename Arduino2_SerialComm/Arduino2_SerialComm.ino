// HAPTIC PROJECT TEAM 9
// ARDUINO #2
//
// This Arduino receives and transmits information to and from Unity (Serial Communication).
// It actuates the right vibration motor and the fan.
// Inputs (from Unity)
//    - KartSpeed
//    - KartXPosition
//    - VibrationCommand to Right Motor on This Arduino
//        (from Arduino 1)
//    - Steer (0 or 1)
// Outputs (to Unity)
//    - Steer
//    - Acceleration/BrakeCommand
//         (to Arduino1)
//    - KartXPosition (mapped from 0 to 255)
//    - Vibration Command to Left Motor on Arduino 1
//
// COM PORT: # COM 4
//

// Libraries Used
#include <math.h>
#include <SerialCommand.h>
#include <String.h>



// Pin Declarations
// PWM Enabled D/O Pins: 3,9,10,11
// DO NOT USE PIN 4.
// VibrationFeature
int vibrRPwmPin = 5; // Motor 1   // Right Vibration Motor
int vibrRDirPin = 8; // Motor 1
int vibrLCommandPin = 9; // Send to Arduino 2 Pin 9 to Actuate Vibr L Motor

// Acceleration Control
int acceleratePin = 2;    // Right Button
int brakePin = 12;        // Left Button
int acceloutPin = 10;  // to LED
int brakoutPin = 11;    // to LED

// Fan Pin Declarations
int fanDirPin = 7;
int fanPwmPin = 6;

// Communication with Arduino 1
int posToArduino1Pin = 3;         // Position Information Arduino 1's A5
int steerFromArduino1Pin = 13;    // Receive Steer from Arduino 1's Pin 10

// Unity Communication Variables
int speedchange = 0;
int topSpeed = 50; // top speed of the kart allowable, per user setting.
float kartspeed = 0;
float kartspeednorm = 0;
int speedsignal = 0;
float xposition = 0;
float xpositionnorm = 0; // normalized to range -1 to 1
float roadcenter = 16;
float roadlborder = 11;
float roadrborder = 21;
int steer = 0;
float steerforUnity = 0;
int acceleration  = 0;


// Serial Communication
SerialCommand sCmd;

void setup() {
  // put your setup code here, to run once:

  // Set up serial communication
  Serial.begin(9600);

  // Unity-Related Setup
  pinMode(acceleratePin, INPUT_PULLUP);
  pinMode(brakePin, INPUT_PULLUP);
  pinMode(acceloutPin, OUTPUT);
  pinMode(brakoutPin, OUTPUT);
  pinMode(vibrLCommandPin, OUTPUT);
  pinMode(vibrRPwmPin, OUTPUT);
  pinMode(vibrRDirPin, OUTPUT);
  pinMode(posToArduino1Pin, OUTPUT);
  pinMode(steerFromArduino1Pin, INPUT);
  pinMode(fanDirPin, OUTPUT);
  pinMode(fanPwmPin, OUTPUT);

  digitalWrite(acceloutPin, LOW);
  digitalWrite(brakoutPin, LOW);
  digitalWrite(vibrLCommandPin, LOW);
  digitalWrite(vibrRDirPin, HIGH);
  analogWrite(vibrRPwmPin, 0);
  digitalWrite(posToArduino1Pin, LOW);
  // digitalWrite(fanDirPin, LOW);
  // analogWrite(fanPwmPin, 0);

  while (!Serial);

  // Unity Commands Recognized
  sCmd.addCommand("L", vibrateLeft);  // Turn on the left vibration motor
  sCmd.addCommand("R", vibrateRight); // Turn on the right vibration motor
  sCmd.addCommand("O", vibrateOff);   // Turn off both vibration motors.
  sCmd.addCommand("B", vibrateBoth);  // Turn on both vibration motors to signal going backwards
  sCmd.addCommand("E", gameEnd);      // End of the game
  // sCmd.addCommand("S", actuateFan);     // Get the speed information and send it to Arduino1
  // sCmd.addCommand("P", gotPosition);  // Get the postion information and send it to Arduino 1
  sCmd.addDefaultHandler(error);      // Default Handler
}

void loop() {
  // put your main code here, to run repeatedly:

  // Receive input from Unity
  if (Serial.available() > 0)
    sCmd.readSerial();

  // If receives corresponding command, will send speed and position information to Arduino1

  // Send Acceleration Button Press Information to Unity
  checkAccel(); // send acceleration/brake information

  // Retreive Steer information from Arduino 1 and Send to Unity
  checkSteer(); // send steering information

  //delay(0);
}

// Turn on the Left Vibration Motor
void vibrateLeft()
{
  // vibrate the vibration motor on the left side of the wheel

  digitalWrite(vibrLCommandPin, HIGH);
  // turn off the vibration motor on the right side of the wheel
  digitalWrite(vibrRDirPin, LOW);
  analogWrite(vibrRPwmPin, 0);
}

// Turn on the Right Vibration Motor
void vibrateRight()
{
  // vibrate the vibration motor on the right side of the wheel
  digitalWrite(vibrRDirPin, HIGH);
  analogWrite(vibrRPwmPin, 50);

  // turn off the vibration motor on the left side of the wheel
  digitalWrite(vibrLCommandPin, LOW);
}

// Turn off the vibration motor on both sides of the wheel
void vibrateOff()
{
  // set both vibration motors to low
  digitalWrite(vibrLCommandPin, LOW);
  digitalWrite(vibrRDirPin, LOW);
  analogWrite(vibrRPwmPin, 0);
}

// Turn on both vibration motors to signal that the car is going backwards.
void vibrateBoth()
{
  // set both vibration motors to High
  digitalWrite(vibrLCommandPin, HIGH);
  digitalWrite(vibrRDirPin, HIGH);
  analogWrite(vibrRPwmPin, 50);
}

//// Get the speed information and actuate the fan accordingly.
//void actuateFan()
//{
//  // Read speed information
//
//  String s = sCmd.next();
//  String t = sCmd.next();
//
//  // while s.equals("S"){
//  // s = sCmd.next();
//  //}
//  //  convert information to float
//  kartspeed = s.toFloat();
//  //check that the signal is the speed information
//  // if ((kartspeed >= 0) && (t.equals("T")))
//  if (kartspeed >= 0)
//  {
//    kartspeednorm = kartspeed * 1.0 / topSpeed;
//    speedsignal = (int) kartspeednorm * 255;
//    // send speed information to the Arduino1
//    digitalWrite(fanDirPin, HIGH);
//
//    Serial.println("fan command has been received.");
//    analogWrite(fanPwmPin, speedsignal);
//    //if we read the wrong signal
//  } else {
//  //disregard signal
//  }
//}

// Get the position information and send it to Arduino 1 for the range (0 to 255)
//void gotPosition()
//{
//  String p = sCmd.next(); // get next information
//  String q = sCmd.next(); // get next information
//
//  // convert information to float
//  xposition = p.toFloat();
//
//  // Check for the termination signal
//  if (q.equals("Q")) {
//    // Normalize position to range 0 to 255
//    constrain(xposition, roadlborder, roadrborder);
//    xpositionnorm = map(xposition, roadlborder, roadrborder, 0, 255);
//    // Normalize position to range -1 to 1
//    // xpositionnorm = (xposition - roadcenter) / (roadrborder - roadlborder);
//
//    // Send position to Arduino1
//    analogWrite(posToArduino1Pin, xpositionnorm);
//  }
//  // if we read the wrong signal
//  else
//  {
//    // do nothing; move on
//  }
//}

// Default handler
void error()
{
  // do nothing, move on
}

// Check user input for acceleration/braking,
// and send the appropriate signal to Unity.
void checkAccel()
{
  // If user presses button to accelerate
  if (digitalRead(acceleratePin) == LOW) {
    // Send signal to Unity to Accelerate
    Serial.println("/+1/");
    // Indicate with LED
    digitalWrite(acceloutPin, HIGH);

    acceleration++;
    acceleration *= 4.6;
    if (acceleration > 100) {
      analogWrite(fanPwmPin, acceleration);
      delay(50);
    } else {
      analogWrite(fanPwmPin, 0);
      delay(50);
    }

  } else {
    // do nothing
    // Serial.println("NONE");
    digitalWrite(acceloutPin, LOW);
  }

  // If user presses button to deccelerate
  if (digitalRead(brakePin) == LOW) {
    // Send signal to Unity to Deccelerate
    Serial.println("/-1/");
    // Indicate with LED
    digitalWrite(brakoutPin, HIGH);

    acceleration--;
    acceleration *= 0.5;
    if (acceleration > 100) {
      analogWrite(fanPwmPin, acceleration);
      delay(50);
    } else {
      analogWrite(fanPwmPin, 0);
      delay(50);
    }


  } else {
    // do nothing
    // Serial.println("NONE");
    digitalWrite(brakoutPin, LOW);
  }
}

// Retreives steering information from Arduino 1
// and sends the appropriate signal to Unity
void checkSteer() {
  steer = digitalRead(steerFromArduino1Pin); // int be of range 0 to 1023
  if (steer == LOW)  // float be of range -1 to 1
  {
    Serial.println("L");  // Signal to steer right; by 1
  }
  else {            // if signal is HIGH
    Serial.println("R");  // Signal to steer left;  by -1
  }
}

// Signals the end of the game/ turns all actuators off
void gameEnd()
{
  // Turn off the right vibration motor
  digitalWrite(vibrRDirPin, LOW);
  analogWrite(vibrRPwmPin, 0);
  // Turn off the left vibration motor
  digitalWrite(vibrLCommandPin, LOW);
  // Turn off the fan.
  analogWrite(fanPwmPin, 0);
}
