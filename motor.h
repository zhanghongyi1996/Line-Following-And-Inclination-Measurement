/**
   Class file for a motor
   A class to neatly contain commands for the motors, to take care 
   of +/- values, a min/max power value, & pin setup.
*/
#ifndef _MOTOR_H
#define _MOTOR_H

class motor_c {
  public:

    // Function Prototypes
    motor_c( );
    void setPins( const int direction_pin, const int power_pin );
    void drive( signed int power );
     
  private:
    int _direction_pin ;
    int _power_pin;
    
    // Function Prototypes
    void driveWheel ( int wheel_direction, signed int power );
    
};  // End of class definition.

/**
  The default constructor
*/
motor_c::motor_c(  ) {
  
}

/**
   Drive the supplied wheel in the direction.

   @param  direction_pin, the direction pin for the motor.
   @param  power, the power pin for the motor.

   @return void.

   @throws Nothing.
*/
void motor_c::setPins (const int direction_pin,
                       const int power_pin) {

    // Set direction for wheel
    this->_direction_pin = direction_pin ;

    // Send power pins, to motor drivers.
    this->_power_pin =  power_pin;

} // End motor_c:: driveWheel

/**
   Wheel drive for the wheel.

   @param  signed int, power to applied to the wheel.

   @return void.

   @throws Nothing.

   @exceptsafe Checks for range of entered values for power and converts to range.
               negagtive value inidates a reverse direction.
*/
void motor_c::drive( signed int power ) {

  // Set the wheel direction, defaulted to forward
  int wheel_direction = WHEEL_FORWARD;

  // If the power value is greater than Max neg and pos values set to corrspounding upper limit.
  if ( power < MAX_NEGATIVE_WHEEL_POWER ) {
    
      // Set power to max negative power.
      power = MAX_NEGATIVE_WHEEL_POWER;
    
  } else if ( power > MAX_POSITIVE_WHEEL_POWER ) {
    
      // Set power to max  power.
      power = MAX_POSITIVE_WHEEL_POWER;
  }

  // If a negative value then the wheel drive indicates reverse.
  if ( power < 0 ) {
    
      wheel_direction = WHEEL_REVERSE;

      // Note, a value closer to zero indcates more power a value of -255 indcates no power
      power = -power;
  }

  this->driveWheel( wheel_direction, power );

} // End motor_c::drive

/**
   Drive the supplied wheel in the direction.

   @param  wheel_direction, the direction of the wheel.
   @param  power, the power for the wheel.

   @return void.

   @throws Nothing.
*/
void motor_c::driveWheel (        int wheel_direction,
                           signed int power) {

    // Set direction for left wheel
    digitalWrite( this->_direction_pin, wheel_direction );

    // Send power PWM to pins, to motor drivers.
    analogWrite( this->_power_pin, power);

} // End motor_c:: driveWheel

#endif
