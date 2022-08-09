/**
    File for until functions
*/
#ifndef _UTIL_h
#define _UTIL_h

/**
   Flashes the supplied LED

   @param  pin the pin on the board to activate.
   @param  delay_ms the rate at which the LED flashes.
   @param  inverse_power indicates if the power levels for the pins is invereted.

   @return void.

   @throws Nothing.

   @exceptsafe No checking implemented.
*/
void flashLed (int  pin,
               int  delay_ms,
               bool inverse_power) {

  int on  = HIGH;  // Set the power level for the on stae of pin
  int off = LOW;   // Set the power level for the off stae of pin

  if ( inverse_power == true) {
    
      // swap the on / off values
      on  = LOW;
      off = HIGH;
  }

  // turn the LED on
  digitalWrite( pin, on );

  // wait for a supplied period
  delay( delay_ms );

  // turn the LED off
  digitalWrite( pin, off );

  // wait for a supplied period
  delay( delay_ms );

} // End flashLed

/**
   Activate the buzzer on its pin at a given volume on an analog signal

   @param  volume for the buzzer.

   @return void.

   @throws Nothing.
*/ 
void playAnalogTone (int volume) {
 
  analogWrite( BUZZER_PIN, volume );
  delay( TONE_SHORT_DURATION );
  analogWrite( BUZZER_PIN, ZERO );

} // End playAnalogTone

/**
   Plays the buzzer a paramater number of times

   @param  times, the number of time the buuzzer it to chime.

   @return void.

   @throws Nothing.
*/ 
void notificationSequence (int times) {

  for ( int i= 1; i<= times; i++ ) {
      delay( TONE_SHORT_DURATION );
      playAnalogTone( TONE_VOLUME );
  }

} // End notificationSequence

/**
 * Read the serial line
 *
 * @param  None.
 * 
 * @return void.
 *
 * @throws Nothing.
 *
 * @exceptsafe No checking implemented.
  */
void serial_line_read() {
    
    //This line checks whether there is anything to read
    if ( Serial.available() ) {
    
        //This reads one byte
        char inChar = Serial.read(); 
        Serial.println(inChar);
        
        if ( inChar == SERIAL_CALIBRATE ){
            Serial.println("Entering the Calibration Mode..."); 
            current_state = STATE_CALIBRATE;
            return;
        }

        if ( inChar == SERIAL_EXP_1 ){
            Serial.println("Entering the Experiment 1 Mode..."); 
            current_state = STATE_EXPERIMENT_1;
            return;
        }

        if ( inChar == SERIAL_EXP_2 ){
            Serial.println("Entering the Experiment 2 Mode..."); 
            current_state = STATE_EXPERIMENT_2;
            return;
        }
        
        if ( inChar == SERIAL_EXP_DRIVE ){
            Serial.println("Entering the Drive Mode..."); 
            current_state = STATE_DRIVE_FORWARD;
            return;
        }

        if ( inChar == SERIAL_FLIP_FLOP_DEBUG ){
            Serial.println("Flip Flop Debug Mode ..."); 
            debug_mode = !debug_mode;
            return;
        } 

       if ( inChar == SERIAL_FLIP_FLOP_FILTER ){
            Serial.println("Flip Flop Filter Mode ..."); 
            filter_mode = !filter_mode;
            current_state = STATE_SENSOR_CHANGE;
            return;
        } 

       if ( inChar == SERIAL_FLIP_FLOP_SENSOR ){
            Serial.println("Flip Flop Sensor Mode ...");
            current_state = STATE_SENSOR_CHANGE; 
            sensor_mode = !sensor_mode;
            return;
        } 
        
        if ( inChar == SERIAL_STEP_EXP1_DISTANCE ){
            if (exp1_base_distance == 0.0)
            {
                exp1_base_distance = 40.0;
            } else if (exp1_base_distance == 40.0) {
                exp1_base_distance = 80.0;
            } else if (exp1_base_distance == 80.0) {
                exp1_base_distance = 0.0;
            }
            Serial.print("New Exp 1 Pos Dist ...");Serial.println(exp1_base_distance);
            return;
        }    
        
        Serial.println("ERROR - Unknown Command"); 
    }
    
} // End serial_line_read


#endif
