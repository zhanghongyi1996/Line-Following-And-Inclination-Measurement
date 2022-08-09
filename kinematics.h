/**
   Class file for the robot kinematics
   File currently encapsulates the update and a distance travelled  function
*/
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

class kinematics_c {
  public:
  
    // should be behind getters and setters
    float x                    = 0.0;   // Current x value, should be behind getters and setters
    float y                    = 0.0;   // Current y value, should be behind getters and setters
    float theta                = 0.0;   // Current heading in radians
    
    // Function Prototypes
    kinematics_c();                     // The default constructor 
    void update();                      // Update kinematics
    long distanceTravelled( );          // Gives a rough distance travelled
    
  private:
    float _theta_old           = 0.0;   // Old value of theta in radians
    float _x_old               = 0.0;   // Old value of x
    float _y_old               = 0.0;   // Old value of y
    long  _left_wheel_ticks    = 0;     // Records the number of ticks for the left wheel to calculate distance travelled 
    long  _right_wheel_ticks   = 0;     // Records the number of ticks for the right wheel to calculate distance travelled 
    
}; // End of class definition.

/**
  The default constructor
*/
kinematics_c::kinematics_c() {
  // Nothing to do  
} // End of default constructor.

/**
  Routine to update the ongoing kinematics

  @param  None
          
  @return void.

  @throws Nothing.
*/
void kinematics_c::update() {
  
  long  left_wheel_ticks_now  = 0;   // Working variable to record the value from the violate encoder variable
  long  right_wheel_ticks_now = 0;   // Working variable to record the value from the violate encoder variable
 
  float left_wheel_distance  = 0.0;  // The distance travelled by the left wheel since the last calculation 
  float right_wheel_distance = 0.0;  // The distance travelled by the right wheel since the last calculation 

  float distance             = 0.0;   // Working variable to calculate the distances when required
   
  // Snapshot the wheel encoders
  left_wheel_ticks_now  = left_wheel_count_e0;
  right_wheel_ticks_now = right_wheel_count_e1;
  
  // Calc distance traveled for each wheel in mm, is difference in ticks * distance for one tick
  left_wheel_distance  = (left_wheel_ticks_now - this->_left_wheel_ticks) * DISTANCE_PER_TICK;
  right_wheel_distance = (right_wheel_ticks_now - this->_right_wheel_ticks) * DISTANCE_PER_TICK;
  
  // Take right from left and divided by wheel seperation
  distance = (left_wheel_distance - right_wheel_distance) / WHEEL_SEPARATION;
  
  // Calc new heading theta = theta_old + (distance_left - distance_right)/wheel seperation
  this->theta = this->_theta_old + distance;

  distance = ( left_wheel_distance + right_wheel_distance ) / 2;
  
  // Calc new x = xold + d * cos(theta)
  this->x = this->_x_old + distance * cos( this->theta );
  
  // Calc new y = yold + d * sin(theta)
  this->y = this->_y_old + distance * sin( this->theta );
  
  // Snapshot old heading
  this->_theta_old = this->theta;
  
  // Snapshot wheel encoders
  this->_left_wheel_ticks  = left_wheel_count_e0;
  this->_right_wheel_ticks = right_wheel_count_e1;

  // Snapshot old x and y
  this->_x_old = this->x;
  this->_y_old = this->y;

  //  Debug Statements
  //  Serial.println( "x, y , theta" );
  //  Serial.print( this->x ); Serial.print( "," ); Serial.print( this->y ); Serial.print( "," ); Serial.println( this->theta );
  //  Serial.println( "" );
   
}  // kinematics_c::update

/**
  Rough distance caluclation based on average wheel ticks.

  @param  None
          
  @return long the distance covered in mm.

  @throws Nothing.
*/
long kinematics_c::distanceTravelled() {
  
  // Rough caluclation based on wheel ticks based on an average hence divide by 2
  long distance = (left_wheel_count_e0 + right_wheel_count_e1) / 2;
  distance = distance * DISTANCE_PER_TICK;
  
  return ( distance );  
   
} // End kinematics_c::distanceTravelled

#endif
