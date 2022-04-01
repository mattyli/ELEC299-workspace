#include <Servo.h>

/**
 * @brief Arduino program written by Matthew Li and Brandon Rutherford, builds upon elements of Professor Joshua Marshall's code.
 * 
 * Program will enable a gripper to pick up a flag, hold it, then release it.
 * We added a variety of helper functions to improve modularity and increase readability
 * 
 */

// SETUP BLOCK
// constant definitions
#define UP      180                                 // figure out if these are the correct angles
#define DOWN    90                                  // LEVEL with the ground
#define OPEN    0
#define CLOSE   180
#define FORWARD 90
#define LEFT    0
#define RIGHT   180

Servo servoXY;                                      // servo object to control the XY plane servo
Servo servoZ;                                       // servo object to control the z direction servo
Servo gripper;                                      // servo object to control the gripper

int angleXY = 0;                                    // Variable to store the servo position [degrees]
int angleZ = 0;                                     // storing angle position of servoZ
int gripperAngle = 0;                               // stores the gripper angle

// TEMPORARY PIN ASSIGNMENTS, make sure they are digital pins
int servoPinXY = 3;                                 // Variable to hold the servo pin (PIN 2)
int servoPinZ = 2;                                  // Z servo pin 2
int gripperPin = 13;                                // gripper servo pin 13

// (13, 3, 2) <-- only availble pins from today

// END OF SETUP

// HELPER FUNCTIONS
void rotateServo(Servo servo, int servoAngle, int position, int speed=1){
    int currentPOS = servo.read();                      // use .read() to get the current servo position

    if (currentPOS < position){
        for (servoAngle = currentPOS; servoAngle <= position; servoAngle += speed){
        servo.write(servoAngle);
        delay(50);
        }
    }// rotate CW

    else if (currentPOS > position){
        for (servoAngle = currentPOS; servoAngle >= position; servoAngle -= speed){
        servo.write(servoAngle);
        delay(50);
        }
    }// rotate CCW

    else{}                                              // don't do anything, current position = next position
}// updated rotateServo()

void makeGripper(int pos)
{
    rotateServo(gripper, gripperAngle, pos);            // close or fully open the gripper
}

void rotateVertical(int pos)                            // another parameter, theta, how much you want to rotate
{
    rotateServo(servoZ, angleZ, pos);                   // code to rotate the gripper
}

void rotateHoriztonal(int pos)
{
    rotateServo(servoXY, angleXY, pos);// fix
}

// code to reset the state of the gripper
void reset()
{
    makeGripper(OPEN);                              // making the gripper open
    rotateVertical(DOWN);                           // making the vertical axis down
    rotateHoriztonal(FORWARD);                      // rotate the gripper array so it faces forward                          
}// forward facing, horizontal, open claws

// END OF HELPER FUNCTIONS

// actual program implementation
void activity2()
{
    reset();
    makeGripper(OPEN);                              // open for 5 seconds
    delay(5000);                                    // delay for 5000 ms = 5 s
    makeGripper(CLOSE);                             // close the gripper
    delay(15000);                                   // delay for 15000 ms = 15 s
    rotateVertical(UP);                             // rotating it up
    delay(5000);                                    // hold for 5 seconds
    rotateVertical(DOWN);                           // rotate it back down
    delay(5000);                                    // pause
    makeGripper(OPEN);                              // open the gripper, release the flag
}

void setup()
{
    servoXY.attach(servoPinXY);                     // Attach the servo on pin 12 to the Servo object
    servoZ.attach(servoPinZ);                       // attach the Z pin to pin ==>
    gripper.attach(gripperPin);                     // attach
}

void loop()
{
    delay(1000);
    activity2();
    delay(1000);
}
