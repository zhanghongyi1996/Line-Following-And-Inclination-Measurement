/**
   Class file for a PID controller
*/
#ifndef _PID_h
#define _PID_h

class PID_c {
  public:

    PID_c(float P, float I, float D);               // Constructor, not order of P I & D arguments when calling.
    void  setGains(float P, float I, float D);      // This function updates the values of the gains
    void  reset();                                  // Useful to remove any intergral wind-up
    float update(float demand, float measurement);  // This update takes a demand and measurement.

    void printComponents(); //This function prints the individual components of the control signal and can be used for debugging

    float Kp_output = 0.0;
    float Ki_output = 0.0;
    float Kd_output = 0.0;

  private:

    // Control gains
    float Kp; //Proportional
    float Ki; //Integral
    float Kd; //Derivative


    // Values to store
    float output_signal       = 0.0;
    float last_error          = 0.0; //For calculating the derivative term
    float integral_error      = 0.0; //For storing the integral of the error
    unsigned long last_millis = 0;

};

/**
   Class constructor.

   @param  P, the proportional gain.
   @param  I, the integral gain.
   @param  D, the derivative.

   @return The class.
*/
PID_c::PID_c( float P, 
              float I, 
              float D)
{
  // Store the gains
  setGains( P, I, D );
  
  // Set last_millis
  reset();
}

/*
   This function prints the individual contributions to the total contol signal
   You can call this yourself for debugging purposes, or set the debug flag to true to have it called
   whenever the update function is called.
*/
void PID_c::printComponents() {
  
/*
  Serial.print( this->Kp_output,5 );
  Serial.print(", ");
  Serial.print( this->Kd_output,5 );
  Serial.print(", ");
  Serial.print( this->Ki_output,5 );
  Serial.print(", ");
  Serial.println( this->output_signal,5 );
 */ 
}


void PID_c::reset() {
  this->last_error      = 0;
  this->integral_error  = 0;
  this->Kp_output       = 0;
  this->Ki_output       = 0;
  this->Kd_output       = 0;
  this->last_millis     = millis();
}

/*
   This function sets the gains of the PID controller
*/
void PID_c::setGains(float P, float I, float D) {
  this->Kp = P;
  this->Kd = D;
  this->Ki = I;
}

/*
   This is the update function.
   This function should be called repeatedly.
   It takes a measurement of a particular quantity and a desired value for that quantity as input
   It returns an output; this can be sent directly to the motors,
   or perhaps combined with other control outputs
*/
float PID_c::update(float demand, float measurement) {
  
  // Calculate how much time (in milliseconds) has 
  // bassed since the last update call
  long time_now  = millis();
  long diff_time = time_now - this->last_millis;
  last_millis    = time_now;
  
  float time_delta = (float)diff_time;

  // Calculate error between demand and measurement.
  float error = demand - measurement;

  // This represents the error derivative
  float error_delta = (error - last_error) / (float)diff_time;
  last_error = error;

  // Integral term.
  integral_error = integral_error + (error * (float)diff_time);

  //Calculate P,I,D Term contributions.
  this->Kp_output = (-this->Kp) * error;
  this->Kd_output = (-this->Kd) * error_delta; 
  this->Ki_output = (-this->Ki) * integral_error; 

  //Add the three components to get the total output
  this->output_signal = this->Kp_output + this->Kd_output + this->Ki_output;

  // Pass the result back.
  return this->output_signal;
}

#endif
