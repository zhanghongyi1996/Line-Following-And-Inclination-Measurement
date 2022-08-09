/**
The robot implements the following states at different timing intervals. 
The initial sampling rate is 100ms.  

The robot kinematics are updated on Timer3 running at 500Hz i.e. every 2ms

FSM
^^^^
STATE_INIT - sampling rate 50ms, empty

STATE_SCANNING - sampling rate 50ms,  looking for an incline.

STATE_ANGLE_DETECTION -  sampling rate 50ms, sample and look for an change  in the angle

STATE_RESET -  sampling rate deafult, reset the internal variables of the anglefinder instance

STATE_EXPERIMENT_1 -  no sampling rate, runs the code once and then moves to end state. 
                      A serial input of 1 will trigger the experiment to run again

STATE_EXPERIMENT_1 -  sampling rate 50ms, runs the blocking code and repeats. 
                      A serial input of 2 will trigger the experiment to run activate.
                      
STATE_DRIVE_FORWARD - sampling rate 2ms, drive in a straight line on a  kinematics theta
                      bearing for a set distance scaning for an incline
                        
STATE_DRIVE_TO_BASE - sampling rate 2ms, drive in a straight line on a  kinematics theta
                      bearing for a set distance back to kinematics.x is zero, scaning for an incline
                        
STATE_CALIBRATE - sampling rate default, once shot state to call the sensor calibration routines
                  in different modes. A serial input of 3 will trigger the experiment to run activate.
                                   
STATE_INVESTIGATION - sampling rate configurable,  used to debug and investigate code and 
                      place the robot under ‘controlled’ conditions.
                      
STATE_END - sampling rate 2ms, power down state.

TODO and Bug List
^^^^^^^^^^^^^^^
TODO Need to make tolerances constant in angleFinder_c

*/
#include "globalConstants.h"

/**
  // Volatile Global variables used by Encoder ISR.
*/
volatile long  right_wheel_count_e1;        // Used by encoder to count the rotation
volatile bool  oldE1_A;                     // Used by encoder to remember prior state of A
volatile bool  oldE1_B;                     // Used by encoder to remember prior state of B

volatile long  left_wheel_count_e0;         // Used by encoder to count the rotation
volatile bool  oldE0_A;                     // Used by encoder to remember prior state of A
volatile bool  oldE0_B;                     // Used by encoder to remember prior state of B
          int  current_state = STATE_INIT;  // The operation state of the ROMI   
          bool debug_mode    = false;       // Global flag to flip flop the sensor debug mode
          bool filter_mode   = false;       // Global flag to slip flop the digital filter mode
          bool sensor_mode   = false;       // Global flag to slip flop the sensor scan mode
         float exp1_base_distance = 0.0;    // Used to set the position of the incline for exp 1, steps through 0.0, 40.0 and 80.0
          
#include "kinematics.h"
#include "isr.h"
#include "motor.h"
#include "pid.h"
#include "angleFinder.h"
#include "encoder.h"
#include "utils.h"

unsigned  long elapsed_time               = 0;           // Stores the elapsed_time in the main loop
unsigned  long last_ts                    = 0;           // Stores the general timestamp.
unsigned  long trigger_event_time         = 5000;        // The time in ms the next event should be processed

          long left_wheel_tick_snapshot   = 0;           // Records the left wheel tick snapshot used to calculate velocity          
          long right_wheel_tick_snapshot  = 0;           // Records the right wheel tick snapshot used to calculate velocity 
         
           int left_drive_power           = ZERO;        // Power to the left wheel
           int right_drive_power          = ZERO;        // Power to the right wheel
           int distance_to_drive          = _1M_AND_HALF;// Distance to drive default to 1.5 meters
           int number_of_sensor_readings  = 15;          // The default number of sensor reading for calibration
           int number_of_ex1_readings     = 5;           // the default number of experiment 1 readings
                  
         float velocity_left_wheel        = 0.0;         // mm per milli second
         float velocity_right_wheel       = 0.0;         // mm per milli second
       
         float Kp_heading                 = 0.08;        // Proportional gain for the heading
         float Kd_heading                 = 0.0;         // Derivative gain for the heading
         float Ki_heading                 = 0.0;         // Integral gain for the heading
        
           int execute_once               = 1;           // Flag to exeute code once
 
motor_c        left_motor;                               // Class for the left motor
motor_c        right_motor;                              // Class for the right motor

kinematics_c   robot_kinematics;                         // Kinematics for the robot

PID_c          heading_PID( Kp_heading, 
                            Ki_heading, 
                            Kd_heading );                // Controller for the heading

angleFinder_c  angle_finder;                             // Find the angle class


/*
 ******************************************************
             Core Functions
 ******************************************************
*/
/**
   The setup function runs once when you press reset or power the board
*/
void setup()
{
  // Initialize digital pins for the LEDs as an output.
  pinMode( YELLOW_LED, OUTPUT );
  pinMode( RED_LED,    OUTPUT );
  pinMode( GREEN_LED,  OUTPUT );

  // Initialize analaog pin for the buzzer as an ouput.
  pinMode( BUZZER_PIN, OUTPUT );

  // These two function set up the pin
  // change interrupts for the encoders.
  setupEncoder0();
  setupEncoder1();

  // Setup our timer3
  setupTimer3();

  //Start a serial connection
  Serial.begin( BAUD_RATE );
  
  // 7 beeps of the buzzer in the startup
  notificationSequence( 7 );
    
  Serial.println( "***RESET***" );
  
  // We set an intial timestamp value.
  // Note: there are 1,000 microseconds in a millisecond and 1,000,000 microseconds in a second.
  last_ts = micros(); 
  
  // Set up the motors pins
  left_motor.setPins( L_DIR_PIN, L_PWM_PIN );
  right_motor.setPins( R_DIR_PIN, R_PWM_PIN );

  // Set the time between events in milliseconds, Note 500000 = half a second
  trigger_event_time = _50_MILLI_SECONDS;
    
  // ***************************************************************************************************************************************
  // ***************************************************************************************************************************************
  // ********        Configuration for the Experiments
  // ***************************************************************************************************************************************
  // ***************************************************************************************************************************************

  angle_finder._sensor_height = 99.5;             // Sensor height
  angle_finder._debug_mode    = false;             // Switch on debug printing required to get results
  angle_finder._continuous    = false;             // True for sensor in continuous mode
  angle_finder._filter        = true;              // True for digital filter to be active
  
  // ----------------  Calibrate Experiment  -----------------------------
  // Set the state for the ROMI. Uncomment as required
  // Place the Romi into sampling the sensor through the 4 modes.
  number_of_sensor_readings = 50;                  // The number of samples to be sent to the serial port
  //current_state = STATE_CALIBRATE;

  // ----------------  Static Romi/ Static Incline Experiment       -------
  current_state = STATE_EXPERIMENT_1;
  angle_finder._exp1_base_distance = 0.0;          // The distance in mm from the base for the incline to be placed 0.0, 40.0, 80.0 in mm
  number_of_ex1_readings = 10;                     // The number of times in Experiment 1 the Romi will dectect the incline
  
  // ----------------  Static Romi/ Hand Moved Incline Experiment   -------
  //current_state = STATE_EXPERIMENT_2;
  angle_finder._exp2_inter_point_distance = 80.0; // The distance in mm from the first reading to the second reading

  // ----------------  Drive Experiment   --------------------------------
  
  //current_state = STATE_DRIVE_FORWARD;
  distance_to_drive = _1M;                  // The distance the romi will drive before turing around in mm

  // ***************************************************************************************************************************************
  // ***************************************************************************************************************************************
 
  //current_state = STATE_INVESTIGATION; Serial.println("Under Invesigation...");
  
  // Set up the global flag to be changed by the user on the serial link
  debug_mode = angle_finder._debug_mode;

  Serial.println( "Set Up Starting ..." );
  angle_finder.setup();
  angle_finder.calibrateSensorAngle();
  Serial.println( "Set Up Finished Entering Experiment Mode ..." );
  
} // End setup

/**
   The loop function runs over and over again forever
*/
void loop()
{
  // Get how much time has passed right now.
  // Note: there are 1,000 microseconds in a millisecond and 1,000,000 microseconds in a second.
  unsigned long time_now = micros() ; 
  
  // Work out how many microseconds have gone
  elapsed_time = time_now - last_ts;

  serial_line_read();
  
  // Trigger, if the target time has elapsed
  if ( elapsed_time > trigger_event_time ) {

      // Update time stamp
      last_ts =  micros();

      wheelVelocity();
      
      // Just in case it was changed on the serial interface
      angle_finder._debug_mode = debug_mode;
      
      // The FSM
      if ( current_state == STATE_EXPERIMENT_1 ) {
          delay ( _3_SEC );
          // Set the sampling rate up
          trigger_event_time = _2_MILLI_SECONDS;       
          experiment_1();
          current_state = STATE_END;
          
      } else if ( current_state == STATE_EXPERIMENT_2 ) {
        
          // Set the sampling rate up
          trigger_event_time = _20_MILLI_SECONDS;
          experiment_2();
             
      } else if ( current_state == STATE_CALIBRATE ) {

          sensorReadingTests();   
          current_state = STATE_END;
          
      }  else if ( current_state == STATE_SENSOR_CHANGE ) {
        
          // Set the sampling rate up
          trigger_event_time = _100_MILLI_SECONDS;
          
          angle_finder.readingTest(5, 
                                   angle_finder._debug_mode, 
                                   sensor_mode, 
                                   filter_mode );
                                   
          Serial.println("\nDone Changing Config... "); 
                                   
          current_state = STATE_END;
                  
      } /*else if ( current_state == STATE_ANGLE_DETECTION ) {
        
          // Set the sampling rate up
          trigger_event_time = _100_MILLI_SECONDS;
          angleDetected();       

      } */else if ( current_state == STATE_EXP1_RESET ) {

          // An slope has been detected so the scan is sat in 
          // true mode waiting for a change in environmental state
          if ( angle_finder.scan() == false ) {
              angle_finder.reset();
              current_state = STATE_EXPERIMENT_2;
              Serial.println("Exp 2 Ended ... ");
          }
         
      } else if ( current_state == STATE_DRIVE_FORWARD ) {
        
          // Set the sampling rate up
          trigger_event_time = _2_MILLI_SECONDS;
          
          bool flip = false;      
          if ( robot_kinematics.x > distance_to_drive ) {
              flip = true;
          }
          
          backAndForth(0, flip, CLOCKWISE, STATE_DRIVE_TO_BASE);      
         
      } else if ( current_state == STATE_DRIVE_TO_BASE ) {
        
          // Set the sampling rate up
          trigger_event_time = _2_MILLI_SECONDS;

          bool flip = false;          
          if ( robot_kinematics.x < 0 ) {
              flip = true;
          }
          
          backAndForth(PI, flip, COUNTER_CLOCKWISE, STATE_DRIVE_FORWARD); 
                         
      } else if ( current_state == STATE_END ) {

          // All stop, power down
          left_motor.drive( STOP );
          right_motor.drive( STOP );
         
      } else if ( current_state == STATE_INVESTIGATION ) {

          // Code which is under investigation.  Usefull in debugging
          // Just set the current_state = STATE_INVESTIGATION in the setup() function
                  
          codeUnderInvestigation();
      }  
              
   }  // End if (elapsed_time > trigger_event_time)
   
} // End loop

/**
 ******************************************************
             Function Section
 ******************************************************
*/
/**
   Function stud to investigate code and state change

   @param  None.

   @return None.

   @throws Nothing.
*/
void codeUnderInvestigation() {
  
  trigger_event_time = _100_MILLI_SECONDS;

  if (execute_once == 1 ) {
    
     angle_finder.CUT();

     execute_once = 2;
  }
  
} // End codeUnderInvestigation


/**
   Function encapsulate experiment 1 steps.  
   Calls the method 1 angle detection a set number of times.
   The incline is a base of the romi, i.e. right in front of it

   @param  None.
   
   @return None.

   @throws Nothing.
*/
void experiment_1() {
  
  int i = 1;   // loop counter
  
  Serial.println("Exp 1");
  Serial.print(number_of_ex1_readings);Serial.println(" times to execute");
  angle_finder.dumpStats();
  angle_finder._exp1_base_distance = exp1_base_distance;
  Serial.print("Exp 1 Pos Dis ...");Serial.println(exp1_base_distance);
  
  Serial.println("Angle Found (Degs)...In Milli Seconds ");
  
  // Loop for the number of readings
  while (i++ <= number_of_ex1_readings ) {
      long tic = millis();
      angle_finder.method1AngleFinder();
      long toc = millis() - tic;
      Serial.print(angle_finder.angle_of_incline * 180/PI ); 
      Serial.print("\t");
      Serial.println(toc); 
  }
  Serial.println("End of Exp 1\n");
} // End experiment_1

/**
   Function encapsulate experiment 2 steps.  
   Process the steps when an angle has been detected.
   It blocks the main thread of execution.  
   First take a reading, wait for a period of time.  Inform the 
   operator to move the incline and then take a second reading 
   before calculated the detected angle.
   
   @param  None.

   @return None.

   @throws Nothing.
*/
void experiment_2() {

  if ( angle_finder.scan() == true ) {
    
      Serial.println("Possible Incline Detected... ");
      
      // First beep to indicate found incline
      notificationSequence( 1 );
      delay( _3_SEC );
      
      notificationSequence( 2 );
      angle_finder._exp2_inter_point_distance = exp1_base_distance;
      Serial.print("First Reading Taken, move incline  ");
      Serial.println(angle_finder._exp2_inter_point_distance);
      
      angle_finder.sample();
    
      // The angle should be moved externally by the operator to position 2
      delay( _3_SEC + _3_SEC );
      notificationSequence( 3 );
      Serial.println("Second Reading Taken... ");  
      angle_finder.sample(); 
                       
      if ( angle_finder.angle_of_incline > 0.0 ) {
          Serial.print("Angle Has Been Found (Rads), (Degs) ... ");
          Serial.print(angle_finder.angle_of_incline);
          Serial.print(",");
          Serial.println(angle_finder.angle_of_incline * 180/PI);    
      } else {
          Serial.println("ERROR - Failed To Determin Angle! ");
      }
      Serial.println("Waiting for Incline To Be Removed... ");
      current_state = STATE_EXP1_RESET;
  } 
  
} // End experiment_2

/**
   Function to drive the romi on a bearing, rotate and change state

   @param  bearing , the bearing to follow
           filp, flag to indicate that the state should change
           roation, the direction to rotate the romi
           next_state, the next state to switch the FSM to

   @return None.

   @throws Nothing.
*/
void backAndForth(float bearing, bool flip, bool rotation, int next_state) {
  
  // Drive on Bearing
  driveOnBearing( bearing, DRIVE_SPEED );
  
  // Check for incline
  if ( angle_finder.scan() == true ) {
   
      // All stop, power down
      left_motor.drive( STOP );
      right_motor.drive( STOP );

      // First beep to indicate found incline
      notificationSequence( 1 );
      Serial.print("Incline Detected... ");

      // Reading 1
      angle_finder.sample();
               
      // Drive Forward 40 mm
      driveDistance( _40_MM );
               
       // All stop, power down
      left_motor.drive( STOP );
      right_motor.drive( STOP );  
      
      angle_finder._exp2_inter_point_distance = 40.0; 
      // Reading 2
      angle_finder.sample();

      float found_angle_deg = angle_finder.angle_of_incline * 180/PI;
      Serial.println(found_angle_deg);
      // Check
      if ( found_angle_deg > 14.0 ) {
          Serial.print("I Can't Drive ...");
          Serial.println(found_angle_deg);
          flip = true ;   
      } else {
          // Drive on Bearing
          driveOnBearing( bearing, DRIVE_SPEED_DOUBLE );

          // Drive for a period of time
          delay( _1_SEC );
          
          angle_finder.reset();
      }
  }  
  
  // Cannot do slope
  if ( flip == true ) {
    
      // Rotate on the spot for 180 deg right
      romiRotate( rotation, _180_DEGS );
      
      delay( _250_MS );
      
      // Flip state
      current_state = next_state;         
  }  

} // End backAndForth

/**
   Function to process the steps when an angel has been detected.
   It blocks the main thread of execution.  
   First take a reading, wait for a period of time.  Inform the 
   operator to move the incline and then take a second reading 
   before calculated the detected angle.
   
   @param  None.

   @return None.

   @throws Nothing.
*/
void angleDetected() {
  
  delay( _2_SEC );
  notificationSequence( 2 );
  Serial.println("First Reading Taken (move incline to point 2)... ");
  angle_finder.sample();

  // The angle should be moved externally by the operator to position 2
  delay( _2_SEC + _3_SEC );
  notificationSequence( 3 );
  Serial.println("Second Reading Taken... ");  
  angle_finder.sample(); 
                   
  if ( angle_finder.angle_of_incline > 0.0 ) {
      Serial.print("Angle Has Been Found (Rads) ... ");
      Serial.println(angle_finder.angle_of_incline);
      Serial.print("Angle Has Been Found (Degs) ... ");
      Serial.println(angle_finder.angle_of_incline * 180/PI);    
  } else {
      Serial.println("ERROR - Failed To Determin Angle! ");
  }

  current_state = STATE_RESET;
  Serial.println("Waiting for Incline To Be Removed... ");

} // End angleDetected

/**
   Function to repletely scan for an indication that in incline is detected

   @param  None.

   @return None.

   @throws Nothing.
*/
void scan() {   
  if ( angle_finder.scan() == true ) {
    
      Serial.println("Possible Incline Detected... ");
      // First beep to indicate found incline
      notificationSequence( 1 );
      delay( _2_SEC );
       
      current_state = STATE_ANGLE_DETECTION;  
  }  

} // End scan

/**
   Function to repletely call the sensor with different configuration 
   switches to obtain a sample set of readings

   @param  None.

   @return None.

   @throws Nothing.
*/
void sensorReadingTests() {
  
  // Snap shot the setting
  bool debug_on      = angle_finder._debug_mode;
  bool continuous_on = angle_finder._continuous;
  bool filter        = angle_finder._filter;
  
  angle_finder.calibrateSensorAngle();
  
  // Trigger sensor reading test
  // Paramters, sample size, debug on, continuous on, DF on
  Serial.println("\nReading based on  samples, not continuous, no digital filter. ");
  angle_finder.readingTest(number_of_sensor_readings, true, false, false );
  angle_finder._debug_mode = false; 
  angle_finder.calibrateSensorAngle();
          
  Serial.println("\nReading based on  samples, not continuous, with digital filter. ");
  angle_finder.readingTest(number_of_sensor_readings, true, false, true );
  angle_finder._debug_mode = false; 
  angle_finder.calibrateSensorAngle();  
       
  Serial.println("\nReading based on  samples, continuous, no digital filter. ");
  angle_finder.readingTest(number_of_sensor_readings, true, true, false );
  angle_finder._debug_mode = false; 
  angle_finder.calibrateSensorAngle();
          
  Serial.println("\nReading based on  samples, continuous, with digital filter. ");
  angle_finder.readingTest(number_of_sensor_readings, true, true, true );
  angle_finder._debug_mode = false; 
  angle_finder.calibrateSensorAngle();

  // Reset the settings and sensor state
  angle_finder._debug_mode = debug_on;
  angle_finder._continuous = continuous_on;
  angle_finder._filter     = filter;

  Serial.println( "\nResetting Sensor To State Before Tests ..." );
  angle_finder.calibrateSensorAngle();
  /*angle_finder.readingTest( 1, 
                            angle_finder._debug_mode, 
                            angle_finder._continuous, 
                            angle_finder._filter );*/
  
} // End sensorReadingTests

/**
   The function called by the ISR timer 3 to update the kinematics values.

   @param  None.

   @return void.

   @throws Nothing.
*/
void updateKinematics() {
  
   robot_kinematics.update();
   
} // End updateKinematics

/**
   Function to instruct the robot to drive forwards by the given distance.

   @param  distanceToDrive, the distance in wheel ticks for the robot
           to drive forwards.
                 
   @return void.

   @throws Nothing.
*/
void driveDistance (int distanceToDrive ) {
  
  int wheel_left_power  =  DRIVE_SPEED;
  int wheel_right_power =  DRIVE_SPEED;

  left_motor.drive( wheel_left_power );
  right_motor.drive( wheel_right_power );

  performDriveAction( distanceToDrive );
  
} // End driveDistance

/**
   Function to instruct the robot to rotate on it centre axis i.e. wheels 
   are revered by a certain amount in degrees

   @param  clockwise_direction, bool flag to indicate the true to rotate 
           clockwise else false for counter clock wise.
   @param  rotation_angle, the angel in degrees the romi should rotate.
   @param  break_online_found, bool flag to indicate if the function should 
           stop early if the line is detected before the full rotation.
           
   @return void.

   @throws Nothing.
*/
void romiRotate( bool  clockwise_direction, 
                 float rotation_angle ) {
                  
  long  rotation_exit         = 0;     // The calculated number of wheel ticks for the given angle
  float distance              = 0;     // Distance for wheel to travel on arc
  
  int wheel_left_power  =  DRIVE_SPEED;
  int wheel_right_power = -DRIVE_SPEED;

  if ( clockwise_direction == false ) {
    
      // Set the wheel power for counter clockwise spin
      wheel_left_power = -DRIVE_SPEED;
      wheel_right_power = DRIVE_SPEED;
  }

  // Calculate the number of ticks
  // For refference 45deg = 360, 90deg = 719 and 180deg = 1440
  // One tick cover 0.157 mm
  // Distance of wheel to travel s = PI*r*angle/180
  distance = ( PI * WHEEL_RADIUS * rotation_angle ) / _180_DEGS ;
  
  //  Requires a *2 to get the right results.  I think due to the wheels running in opposite directions
  rotation_exit = (distance / DISTANCE_PER_TICK) * 2;

  left_motor.drive( wheel_left_power );
  right_motor.drive( wheel_right_power );

  performDriveAction( rotation_exit );
  
}  // End romiRotate

/**
   Function to monitor the left wheel ticks so the drive action can be performed.
   The loop is block the main loop thread and will exit when the action has completed.

   @param  rotation_exit, the calculated number of wheel ticks for the action to end
   @param  break_online_found, bool flag to indicate if the function should 
           stop early if the line is detected before the full rotation.
           
   @return void.

   @throws Nothing.
*/
void performDriveAction (int  rotation_exit) {
                          
  bool preforming_action     = true;  // Flag to performa the action until complete
  long abs_difference        = 0;     // The difference in encoder ticks, but absolue value.
  long snapshot_wheel_count  = 0;     // Used to calaulcated when the action is complete
  
  snapshot_wheel_count = left_wheel_count_e0;
  
  while ( preforming_action == true ) {

      abs_difference = left_wheel_count_e0 - snapshot_wheel_count;

      // Abs so when the wheel goes backwards
      abs_difference = abs(abs_difference);
      
       // If fires if the angle rotaion reached
       if ( abs_difference > rotation_exit ) {

           // Stop the wheels
           left_motor.drive( STOP );
           right_motor.drive( STOP );
           preforming_action = false;
        }
   }
  
} // End performDriveAction

/**
   Function to drive the robot on a bearing.  Other functions control 
   when it comes to a stop.

   @param  bearing, the heading to mainatian in radians.
   @param  drive_power, the base drive power for the motors which is adjusted by 
           the error from the PID controller.
   
   @return void

   @throws Nothing.
*/
void driveOnBearing( float bearing , 
                     int   drive_power ) {
  
  float scalling_factor        = 0.0;   // For heading calculations to get a scalled value for the motors
  float heading_output         = 0.0;   // Stores the output from the heading PID controller
  int   left_power_adjustment  = ZERO;  // Delta power change value for the left wheel
  int   right_power_adjustment = ZERO;  // Delta power change value for the right wheel

  scalling_factor = (float)DRIVE_SPEED_TOP / BEARING_SCALLING;
  
  // Output_signal <----PID-- demand, measurement 
  heading_output = heading_PID.update( bearing, robot_kinematics.theta );

  // if ladder to nugde the robot one way or the other
  if ( heading_output > -BEARING_TOLERANCE && heading_output < BEARING_TOLERANCE ) {
    
      // Do noting on course.  Keep the power adjustments to zero
      left_power_adjustment  = ZERO;
      right_power_adjustment = ZERO;
      
  } else if ( heading_output < -BEARING_TOLERANCE ){
    
      // Negative error term nudge left drive power. Times by -1 to remove negative sign.
      left_power_adjustment  = (int)(scalling_factor * heading_output * -1);
      right_power_adjustment = ZERO;
      
  } else if ( heading_output > BEARING_TOLERANCE ) {
    
      // Positive error term, nudge right drive power
      left_power_adjustment   = ZERO;
      right_power_adjustment  = (int)(scalling_factor * heading_output);
  }   
  
  // Adjust the power on the motors based on the error
  left_drive_power  = drive_power + left_power_adjustment;
  right_drive_power = drive_power + right_power_adjustment;

  // Set the power on the motors
  right_motor.drive( right_drive_power );
  left_motor.drive( left_drive_power );

}  //End driveOnBearing

/**
   Function to maintain the global variables for the wheel velocity 
   in mm per mili second interval

   @param  None.

   @return None.

   @throws Nothing.
*/
void wheelVelocity() {
  // Velocity = distance / time or velocity = (change in encoder count) / elapsed time.
  // Wheel velocity  =   wheel change in encoder count / elapsed time

  float distance_travelled_left  = 0;   // Distance travelled by the left wheel in mm per milli second
  float distance_travelled_right = 0;   // Distance travelled by the right wheel in mm per milli second
  long  tick_difference_left     = 0;   // The number of encoder ticks diffence from the last reading for left wheel
  long  tick_difference_right    = 0;   // The number of encoder ticks diffence from the last reading for right wheel
  
  float elapsed_time_in_millis   = elapsed_time / (float)1000;
  
  tick_difference_left      =  left_wheel_count_e0  - left_wheel_tick_snapshot;
  tick_difference_right     =  right_wheel_count_e1 - right_wheel_tick_snapshot;
  
  left_wheel_tick_snapshot  =  left_wheel_count_e0;
  right_wheel_tick_snapshot =  right_wheel_count_e1;
  
  distance_travelled_left   =  tick_difference_left  * DISTANCE_PER_TICK; // Distance travelled in mm
  distance_travelled_right  =  tick_difference_right * DISTANCE_PER_TICK; // Distance travelled in mm

  //Serial.print(distance_travelled_left);Serial.print(",");Serial.println(distance_travelled_right);
  
  velocity_left_wheel       =  distance_travelled_left  / elapsed_time_in_millis;
  velocity_right_wheel      =  distance_travelled_right / elapsed_time_in_millis;

} // wheelVelocity
