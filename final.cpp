/**
 * SETUP FOR SHARP SENSORS
 */
#include <Servo.h>

const byte SHARP_PIN_FRONT  = 0;    // ANALOG PINS
const byte SHARP_PIN_LEFT   = 1;
const byte SHARP_PIN_RIGHT  = 2;

// Variable to store the proximity measurement
int sharp_val       = 0;
int sharpValLeft    = 0;
int sharpValRight   = 0;

double estimatedDistanceFront   = 0.0;
double estimatedDistanceLeft    = 0.0;
double estimatedDistanceRight   = 0.0;
double threshold = 22.0;                    // the distance at which to stop the robot

double analogToDistance(int analogValue){
    double distance;
    float analogValueF  = (float)analogValue;                // typecast 
    double quadTerm     = pow(analogValueF, 4);
    double tripleTerm   = pow(analogValueF, 3);
    double doubleTerm   = pow(analogValueF, 2);
    
    distance = (0.00000002 * quadTerm) - (0.00002 * tripleTerm) + (0.0099 * doubleTerm) - (2.0354*analogValueF + 178.39);
    // ^ based on the regression completed in Lab 3

    return distance/-10;
}
/**
 * END OF SETUP BLOCK
 */

/**
 * SETUP FOR SERVOS
 */

// SETUP BLOCK
// constant definitions - these constant hold the angles corresponding to the named states
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
// Maybe change this
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
}// opening/ closing the gripper

void rotateVertical(int pos)                            // another parameter, theta, how much you want to rotate
{
    rotateServo(servoZ, angleZ, pos);                   // code to rotate the gripper
}// rotating the vertical servo

void rotateHoriztonal(int pos)
{
    rotateServo(servoXY, angleXY, pos);// fix
}// rotating the horizontal servo

// code to reset the state of the gripper
void reset()
{
    makeGripper(OPEN);                              // making the gripper open
    rotateVertical(DOWN);                           // making the vertical axis down
    rotateHoriztonal(FORWARD);                      // rotate the gripper array so it faces forward                          
}// forward facing, horizontal, open claws

/**
 * SETUP FOR THE ROVER
 * pin assignments for the encoders and controllers
 */
// Motor driver PWM pin
const byte E1 = 6;
const byte E2 = 5;

// Motor driver direction pin
const byte M1 = 7;              // E1 6, M1 7
const byte M2 = 4;              // E2 5, M2 4

// Motor PWM command variable [0-255]
byte u = 0;

// Left wheel encoder digital pins
const byte SIGNAL_A = 8;        // SIG A (Green)
const byte SIGNAL_B = 9;        // SIG B (Yellow)

// Right wheel encoder digital pins
const byte SIGNAL_C = 10;       // SIG A2 (Green)
const byte SIGNAL_D = 11;       // SIG B2 (Yellow)

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks = 0;
volatile long encoder_ticksR = 0;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;
long t_distance = 0;
long t_start = 0;
long t_current = 0;

// This function is called when SIGNAL_A goes HIGH
void decodeEncoderTicksLeft()
{
    if (digitalRead(SIGNAL_B) == LOW)
    {
        encoder_ticks--;        // SIGNAL_A leads SIGNAL_B, so count one way
    }
    else
    {
        encoder_ticks++;        // SIGNAL_B leads SIGNAL_A, so count the other way
    }
}

void decodeEncoderTicksRight()
{
    if (digitalRead(SIGNAL_D) == LOW)
    {
        encoder_ticksR--;       // SIGNAL_A leads SIGNAL_D, so count one way
    }
    else
    {
        encoder_ticksR++;       // SIGNAL_D leads SIGNAL_C, so count the other way
    }
}

void driveForward()
{
    digitalWrite(M1, LOW);      // Drive forward (left wheels)
    analogWrite(E1, 150);       // Write left motors command

    digitalWrite(M2, HIGH);     // Drive forward (right wheels)
    analogWrite(E2, 150);       // Write left motors command
}

void driveBackward()
{
    digitalWrite(M1, HIGH);      // Drive backward (left wheels)
    analogWrite(E1, 150);       // Write left motors command
    digitalWrite(M2, LOW);     // Drive backward (right wheels)
    analogWrite(E2, 150);       // Write left motors command
}

void stopRover()
{
    digitalWrite(M1, HIGH);
    analogWrite(E1, 0);
    digitalWrite(M2, HIGH);
    analogWrite(E2, 0);
}// make the rover stop on an instant

// END OF SECOND BLOCKS

void setup()
{
    Serial.begin(9600);     // Open the serial port at 9600 bps
    servoXY.attach(servoPinXY);                     // Attach the servo on pin 12 to the Servo object
    servoZ.attach(servoPinZ);                       // attach the Z pin to pin ==>
    gripper.attach(gripperPin);                     // attach
}

void loop()
{
    // Read the sensor output (0-1023, or 10 bits)
    sharp_val  = analogRead(SHARP_PIN_FRONT);                   // change the pin to the appropriate one
    estimatedDistanceFront  = analogToDistance(sharp_val);      // value in CM
    // assuming rover is directly facing the flag
    if(estimatedDistanceFront <= threshold){
        t_distance = millis();          // will capture when it was detected since start of execution
        stopRover();                    // stop the rover
        delay(1000);                    
        reset();                        // reset the claws, ready to pick up the flag
        delay(500);
        makeGripper(CLOSE);             // gripper open when reset, close around the flag
        delay(500);     
        rotateVertical(UP);             // lift it up
        delay(500);
        t_start = millis();             // collect the time at this instant
        do{
            t_current = millis();       // record current time
            driveBackward();            
        }while(t_current - t_start <= t_distance);      // while the current time - the start is less than the time it took to get to the flag
        stopRover();                    // back where it started, stop the rover
        delay(500);                     
        rotateVertical(DOWN);           // let the flag drop back down
        delay(500); 
        makeGripper(OPEN);              // open the gripper, release the flag

    }
    else{
        driveForward();
    }
    delay(500);                                              // Delay for a bit before reading the sensor again
}
