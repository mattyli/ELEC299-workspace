/**
 * @file servomotor.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca)
 * @brief Arduino program to command the Hitec HS422 servomotor.
 * @version 1.0
 * @date 2021-12-15
 *
 * @copyright Copyright (c) 2021
 *
 * Based on the example code by BARRAGAN <http://barraganstudio.com>, modified
 * 2013-11-08 by Scott Fitzgerald http://www.arduino.cc/en/Tutorial/Sweep.
 * Uses the Servo library https://www.arduino.cc/reference/en/libraries/servo/.
 *
 * Updated by Matthew Li and Brandon Rutherford (March 31st 2022)
 * 
 */

// Use this library to command servomotors
#include <Servo.h>


Servo hs422servo;                       // Create a servo object to control a servo
int angle = 0;                          // Variable to store the servo position [degrees]
int servoPin = 12;                      // Variable to hold the servo pin

#define OPEN 1
#define CLOSE -1

void setup()
{
    hs422servo.attach(servoPin);        // Attach the servo on pin 12 to the Servo object
}


void rotate(Servo servo, int angle, int state, int arc=180){
    for (angle = 0*state; angle <= arc*state; angle += (1*state)){
        servo.write(angle);
        delay(50);
    }
}

// angle = 0 + (state*180)

void loop()
{
    
    for (angle = 0; angle <= 180; angle += 2)           // For angle 0 to 180 [degrees]
    {
         hs422servo.write(angle);                        // Tell servo to go to position "angle"
         delay(50);                                      // delay half the time before, making it faster by 2x
    } 
    
    for (angle = 180; angle >= 0; angle -= 2)           // For angle 180 to 0 [degrees]
    {
         hs422servo.write(angle);                        // Tell servo to go to position "angle"
         delay(50);                                      // Wait a specified time [ms]
    }

    //rotate(hs422servo, angle, OPEN);
    //rotate(hs422servo, angle, CLOSE);
}
