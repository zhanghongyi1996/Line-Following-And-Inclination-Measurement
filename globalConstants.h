/**
 ******************************************************
         GLOBAL Define Constants and Constant Variables
 ******************************************************
*/
#ifndef _GLOBALCONSTANTS_H
#define _GLOBALCONSTANTS_H

#define FILTER_SAMPLE    3  // The number filter samples

// Pins Constants
#define L_PWM_PIN       10   // The pin for the left wheel power
#define L_DIR_PIN       16   // The pin for the left wheel direction
#define R_PWM_PIN        9   // The pin for the right wheel power
#define R_DIR_PIN       15   // The pin for the right wheel direction

#define BUZZER_PIN       6   // The pin for the builtin buzzer

#define YELLOW_LED      13   // The pin for the yellow LED
#define RED_LED         17   // The pin for the red LED
#define GREEN_LED       30   // The pin for the greeen LED

#define E1_A_PIN         7   // The E1 A pin for the wheel encoders
#define E1_B_PIN        23   // The E1 B pin for the wheel encoders
#define E0_A_PIN        26   // The E0 A pin for the wheel encoders

// Other Constants
#define BAUD_RATE      9600  // The baud rate for the serial trasfer
#define OCR3A_4HZ     15625  // Timer value 4Hz for the kinematics to sample at, set for prescaler of 256
#define OCR3A_40HZ     1562  // Timer value 40Hz for the kinematics to sample at, set for prescaler of 256
#define OCR3A_500HZ     125  // Timer value 500Hz for the kinematics to sample at, set for prescaler of 256
#define OCR3A_1KHZ       62  // Timer value 1KHz for the kinematics to sample at, set for prescaler of 256
/**
 ******************************************************
        Constant Variables that I have control over
 ******************************************************
*/
const bool  CLOCKWISE                = true;     // Rotate clockwise
const bool  COUNTER_CLOCKWISE        = false;    // Rotate ant-clockwise

const float DISTANCE_PER_TICK        = 0.157;    // Distance traveled in mm for one tick
const float WHEEL_SEPARATION         = 145;      // Wheel separtion in mm
const float WHEEL_RADIUS             = 35.4;     // Radius of the a Romi wheel
const float BEARING_TOLERANCE        = 0.000002; // The tolerance on the reading from the heading control to determin if is close to zero
const float BEARING_STRAIGHT_AHEAD   = 0.0;      // Stright ahead
const float BEARING_SCALLING         = 0.089;    // Used to scaled the heading PID controller output in power units for the motors

const int   _7_MM                    = 50;       // 5-7ish mm in encoder ticks 5 / DISTANCE_PER_TICK rounded (50) up 
const int   _40_MM                   = 240;      // 40mm in encoder tick  40 / DISTANCE_PER_TICK
const int   _100_MM                  = 636;      // 100mm in encoder tick 100 / DISTANCE_PER_TICK
const int   _1M                      = 1000;     // A meter in mm
const int   _1M_AND_HALF             = 1500;     // A meter and a half in mm

const int   STATE_INIT               = 0;        // The states for the FSM
const int   STATE_SCANNING           = 1;  
const int   STATE_ANGLE_DETECTION    = 3;
const int   STATE_DRIVE_FORWARD      = 4;
const int   STATE_DRIVE_TO_BASE      = 8;
const int   STATE_CALIBRATE          = 16;
const int   STATE_EXPERIMENT_1       = 18;
const int   STATE_EXPERIMENT_2       = 20;
const int   STATE_EXP1_RESET         = 25;
const int   STATE_RESET              = 32;
const int   STATE_SENSOR_CHANGE      = 34;  
const int   STATE_INVESTIGATION      = 64;
const int   STATE_END                = 128;
     
const int   _0_DEGS                  = 0;      // Stay on the spot
const int   _45_DEGS                 = 45;     // Turn angle - 45
const int   _90_DEGS                 = 90;     // Turn angle - 90
const int   _180_DEGS                = 180;    // Turn angle - 180
const int   _135_DEGS                = 135;    // Turn angle - 135

const int   TONE_SHORT_DURATION      = 150;     // Shortest duration for a tone
const int   TONE_LONG_DURATION       = 350;     // Short duration for a tone
const int   TONE_VOLUME              = 10;      // Volume for the buzzer

const int   _1_SEC                   = 1000;    // 1 second in milli seconds
const int   _2_SEC                   = 2000;    // 2 seconds in milli seconds
const int   _3_SEC                   = 3000;    // 3 seconds in milli seconds
const int   _100_MS                  = 100;     // 100 milli seconds in milli seconds
const int   _250_MS                  = 250;     // 250 milli seconds in milli seconds
const int   _500_MS                  = 500;     // 500 milli seconds in milli seconds

const int   ZERO                     = 0;       // Zero int
const int   STOP                     = 0;       // Stop velocity

const int   DRIVE_SPEED              = 27;      // Normal drive velocity
const int   DRIVE_SPEED_SLOW         = 15;      // Slow drive velocity
const int   DRIVE_SPEED_DOUBLE       = 50;      // Double'ish drive velocity
const int   DRIVE_SPEED_TOP          = 100;     // The max drive velocity

const int   WHEEL_FORWARD            = LOW;     // Wheel forward direction
const int   WHEEL_REVERSE            = HIGH;    // Wheel reverse direction
const int   MAX_NEGATIVE_WHEEL_POWER = -255;    // Max negative wheel power
const int   MAX_POSITIVE_WHEEL_POWER = 255;     // Max positive wheel power

const long  _100_MILLI_SECONDS       = 100000;  // 100 milli seconds for the micros function. 100 milliseconds = 0.1 second
const long  _50_MILLI_SECONDS        = 50000;   // 50 milli seconds for the micros function.   50 milliseconds = 0.05 second
const long  _20_MILLI_SECONDS        = 20000;   // 20 milli seconds for the micros function.   20 milliseconds = 0.02 second
const long  _5_MILLI_SECONDS         = 5000;    // 5 milli seconds for the micros function.     5 milliseconds = 0.005 second
const long  _2_MILLI_SECONDS         = 2000;    // 2 milli seconds for the micros function.     2 milliseconds = 0.002 second

const char SERIAL_CALIBRATE          = 'C';
const char SERIAL_EXP_1              = '1';
const char SERIAL_EXP_2              = '2';
const char SERIAL_EXP_DRIVE          = '3';
const char SERIAL_FLIP_FLOP_DEBUG    = 'D';
const char SERIAL_FLIP_FLOP_FILTER   = 'F';
const char SERIAL_FLIP_FLOP_SENSOR   = 'S';
const char SERIAL_STEP_EXP1_DISTANCE = 'B';

#endif
