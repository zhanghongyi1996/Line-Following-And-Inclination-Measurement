/**
   Class file for the angle finding routines
   File currently encapsulates the calibrate and calibrate
*/
#ifndef _ANGLEFINDER_H
#define _ANGLEFINDER_H

#include <Wire.h>
#include <VL6180X.h>

class angleFinder_c {
  public:
    // Public data
    float angle_of_incline                 = 0.0;    // The calculated angle of incline in radians
    bool  incline_detected                 = false;  // Flag to indicate if an incline has been detected

    //  Should be private but changed access scope for easy of testing
    bool   _debug_mode                     = false;   // If true then sesnor readings are sent to the serial port        
    bool   _continuous                     = false;   // Flag to set the continuous mode
    bool   _filter                         = false;   // Flag to switch on the digital filter 
    float  _sensor_height                  = 99.0;    // The height of the sensor in mm from the ground
    float  _exp1_base_distance             = 0.0;     // For exp 1 the distance from the base for the incline to be placed 0.0, 40.0, 80.0 in mm
    float  _exp2_inter_point_distance      = 100.0;   // For exp 2 the distance from point readings in mm

    // Function Prototypes
    angleFinder_c();                                  // The default constructor 
    void CUT();                                       // Code Under Test
    void setup();                                     // Setup the angle finder base variables
    void dumpStats();                                 // Prints the sensors stats to the serial port
    void calibrateSensorAngle();                      // Determine the sensor angle based on readings
    bool scan();                                      // Scan for an angle
    void sample();                                    // Sample the sensor to get a reading
    void reset();                                     // Reset the internal working variables
    void  method1AngleFinder();                       // Finds the angle based on method 1
    void readingTest( int  sample_size,          
                      bool debug_state, 
                      bool continuous_state, 
                      bool filter_state);             // Read test function                            

  private:
    // Function Prototypes
    void   method2AngleFinder();                      // Finds the angle based on method 2
    float  takeAReading( int sampleNumber );          // Samples a reading from the sensor
    
      VL6180X  _sensor;                               // Reference to the sensor
        bool   _first_sample               = true;    // Flag to indicate to take first sample reading
        
        float  _1st_reading                = 0;       // Old sensor reading
        float  _2nd_reading                = 0;       // New sensor reading
        float  _sensor_beam_length         = 0;       // The sensor beam length, the hypotenuse         

        float  _sensor_angle               = 0.0;     // The calculated angle of the sensor in radians
        float  _sensor_focal_point_length  = 0;       // The focal point in front of the romi where the sensor beam hits
        float  _L4                         = 0.0;     // Last reading adjacent side
        float  _L5                         = 0.0;     // This reading adjacent side
         
       float   _m[FILTER_SAMPLE]           = {0};   // The array of filter sensor readings
       float   _x[FILTER_SAMPLE]           = {0};   // The array of unfiltered sensor readings
      
       float   _a[FILTER_SAMPLE]           = { 1.0, 
                                             -1.64745998107698, 
                                              0.700896781188403}; // Filter Paramters
                                              
       float   _b[FILTER_SAMPLE]           = { 0.0133592000278565,
                                              0.0267184000557130, 
                                              0.0133592000278565}; // Filter Paramters
}; // End of class definition.

/**
  The default constructor
*/
angleFinder_c::angleFinder_c() {
  // Nothing to do  
} // End of default constructor.


/**
   The function samples  the sensor reading and calibrates the angle 
   of the sensor to the ground.

   @param  None.

   @return void.

   @throws Nothing.
*/
void angleFinder_c::setup( ) {
    
  // Init the sensor
  Wire.begin();

  if ( this->_continuous == true ) {
    
      this->_sensor.init();
      this->_sensor.configureDefault();
    
      // Reduce range max convergence time and ALS integration
      // time to 30 ms and 50 ms, respectively, to allow 10 Hz
      // operation (as suggested by Table 6 ("Interleaved mode
      // limits (10 Hz operation)") in the datasheet).
      this->_sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
      this->_sensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);

      // stop continuous mode if already active
      this->_sensor.stopContinuous();
  
      // in case stopContinuous() triggered a single-shot
      // measurement, wait for it to complete
      delay( _250_MS );
  
      // start interleaved continuous mode with period of 100 ms
      this->_sensor.startInterleavedContinuous( _100_MS );
      
  } else {
      this->_sensor.init();
      this->_sensor.configureDefault();
      this->_sensor.setTimeout( _500_MS );
  }
  
} // End angleFinder_c::setup

/**
   Dump the sensor stats to the serial port.

   @param  None.

   @return void.

   @throws Nothing.
*/
void angleFinder_c::dumpStats() {

  Serial.print("Continuous Mode... ");        Serial.println(this->_continuous); 
  Serial.print("Filter State... ");           Serial.println(this->_filter); 
  Serial.print("Sensor Height (mm)... ");     Serial.println(this->_sensor_height); 
  Serial.print("Sensor Angle (Rad)... ");     Serial.println(this->_sensor_angle); 
  Serial.print("Sensor Angle (Deg)... ");     Serial.println(this->_sensor_angle * 180/PI); 
  Serial.print("Sensor Beam Length (mm)... ");Serial.println(this->_sensor_beam_length); 
  Serial.print("Sensor Focal Point (mm)... ");Serial.println(this->_sensor_focal_point_length); 
  
} // End angleFinder_c::dumpStats

/**
   The function samples the sensor reading and calibrates the angle 
   of the sensor to the ground.

   @param  None.

   @return void.

   @throws Nothing.
*/
void angleFinder_c::calibrateSensorAngle() {
  
  int   sample_number   = 100;    // The number of reading to sample

  this->_sensor_beam_length = this->takeAReading( sample_number );

  // Update the sensor angle and focal point in front of Romi
  this->_sensor_angle = acos(this->_sensor_height / this->_sensor_beam_length);

  // Worked out by pythag.
  this->_sensor_focal_point_length = sqrt((   this->_sensor_beam_length * this->_sensor_beam_length) 
                                           - (this->_sensor_height * this->_sensor_height));

  this->dumpStats();
  
} // End angleFinder_c::calibrateSensorAngle

/**
   The function samples the sensor reading.

   @param  None.

   @return void.

   @throws Nothing.
*/
bool angleFinder_c::scan( ) {
  float working_value   = 0.0;  // Working variable
  int   sample_number   = 20;   // The number of reading in sample set
  
  working_value = this->takeAReading( sample_number );
 
  // If the reading if differnt including the tolerance of the scan beam length
  // might be an incline
  working_value = this->_sensor_beam_length - working_value;
 
  int tolerance = 7;   // need to made into a constant
  working_value = abs(working_value);
  incline_detected = false;
  if ( working_value > tolerance ) {     
      incline_detected = true;
  } 
  
  return incline_detected;
  
} // End angleFinder_c::scan

/**
   The function samples the sensor reading.

   @param  None.

   @return void.

   @throws Nothing.
*/
void angleFinder_c::sample( ) {
  float  working_value   = 0.0;  // Working variable
  int    sample_number   = 50;   // The number of reading in sample set
  
  working_value = this->takeAReading( sample_number );

  if (this->_first_sample == true) {
    
      // Take the first reading
      this->_1st_reading = working_value;
      this->_first_sample = false;
      
  } else {
    
      this->_first_sample = true;
      // Take the second reading
      this->_2nd_reading = working_value;

      // Calculate the angle
      this->method2AngleFinder();
      this->reset();           
  }
   
} // End angleFinder_c::sample


/**
   Calcaultes the angle based on method 1 having the incline at the 
   base of the robot
   Tan() = (L1 – x cos())/(L1tan() - L6) – 1/tan()
   
   @param  None.

   @return void.

   @throws Nothing.
*/
void angleFinder_c::method1AngleFinder(){
  
  float base_distance       = 0.0;      // The distance of the incline from the romi
  float working_value1      = 0.0;      // A working variable
  float working_value2      = 0.0;      // A working variable

  // Tan(alpha) = (L1-xcos(theta)) / (xsin(theta)-L6)
  base_distance = this->_exp1_base_distance;
  
  this->_1st_reading = this->takeAReading( 15 );
  //Serial.println(this->_1st_reading);
  
  // (L1-xcos(theta))
  working_value1 = this->_sensor_height - (this->_1st_reading * cos( this->_sensor_angle ));
  //Serial.println(working_value1);
  
  // (xsin(theta)-L6)
  working_value2 = this->_1st_reading * sin(this->_sensor_angle) - base_distance;
  //Serial.println(working_value2);

  working_value2 = atan2(working_value1, working_value2);
  
  //Serial.println(working_value2);
  
  this->angle_of_incline = working_value2;
  
/*
  Serial.println("");
  Serial.println("**************************************************************");
  Serial.print("_1stReading... "); Serial.println(this->_1st_reading);
  Serial.print("Angle ... ");      Serial.println(this->angle_of_incline);
  Serial.println("");
*/
}  // End angleFinder_c::method1AngleFinder

/**
   Calcaultes the angle based on method 3 having the distance travelled
   We know L1, L2, L3,  and   from the set up of the sensor. We need to solve .​
   We know the distance travelled (DT). Cosine and Sin rules can solve the top triangles and Cos for .  ​
   Red Triangle – Cosine rule to solve HDT using SR2, DT and   +90.​
   Red Angle  -solved sin rule.​
   Green  = 90 -  - ​
   Green Triangle - HGR solved cosine rule HDT, SR1 and green  ​
   Cos  = L5-L4 / HGR 
   
   @param  None.

   @return void.

   @throws Nothing.
*/
void angleFinder_c::method2AngleFinder(){
  
  float distance_travelled = 0.0;      // The distance travelled in mm in the time window set by member variable
  float HDT                = 0.0;      // The red triangle hyp
  float red_beta           = 0.0;      // The angle 
  float green_x            = 0.0;      // The angle opposite to the HGR
  float HGR                = 0.0;      // The green triangle hyp
  float _90_in_radians     = PI/2.0;   // 90 degress in radians
  float working_value      = 0.0;      // A working variable

  // Set the simulated distance mmoved
  distance_travelled = this->_exp2_inter_point_distance;
  
  // Solved with cosine rule c= Sqrt(a^2 + b^2 - 2.b.c.cos(theata))
  working_value = this->_2nd_reading * this->_2nd_reading;
  HDT = distance_travelled * distance_travelled;  
  // HDT = a^2 + b^2
  HDT = HDT + working_value;

  // wokingValue = 2.b.c.cos(theata)
  working_value = cos(this->_sensor_angle + _90_in_radians); 
  working_value = 2 * this->_2nd_reading * distance_travelled * working_value;

  HDT = HDT -  working_value;
  HDT = sqrt(HDT);

  // Solve with sin rule
  red_beta = (sin(this->_sensor_angle + _90_in_radians) / HDT) * this->_2nd_reading;
  red_beta = asin(red_beta);

  // Solve by summing angles
  green_x = _90_in_radians - this->_sensor_angle - red_beta;

  // Solved with cosine rule
  HGR = ((this->_1st_reading * this->_1st_reading) + (HDT * HDT));
  working_value = 2 * this->_1st_reading * HDT * cos(green_x);
  HGR = HGR - working_value;
  HGR = sqrt(HGR);

  // L4
  this->_L4 = cos(this->_sensor_angle) * ( (float)this->_sensor_beam_length - (float)this->_1st_reading);
  // L5
  this->_L5 = cos(this->_sensor_angle) * ( (float)this->_sensor_beam_length - (float)this->_2nd_reading);
  
  // Work the angle out
  this->angle_of_incline = (this->_L5 - this->_L4) / HGR;
  this->angle_of_incline = asin(this->angle_of_incline);

/*
  Serial.println("");
  Serial.println("**************************************************************");
  Serial.print("_1stReading... "); Serial.println(this->_1st_reading);
  Serial.print("_2ndReading... "); Serial.println(this->_2nd_reading);
  Serial.print("Delta... ");       Serial.println( this->_1st_reading - this->_2nd_reading);
  Serial.print("HDT... ");         Serial.println(HDT);
  Serial.print("redBeta (r)... "); Serial.println(red_beta);
  Serial.print("redBeta (d)... "); Serial.println(red_beta * 180/PI);
  Serial.print("greenX (r)... ");  Serial.println(green_x);
  Serial.print("greenX (d)... ");  Serial.println(green_x * 180/PI);
  Serial.print("HGR... ");         Serial.println(HGR);
  Serial.print("L4... ");          Serial.println(this->_L4);
  Serial.print("L5... ");          Serial.println(this->_L5);
  Serial.print("L4 L5 delta... "); Serial.println(this->_L5 - this->_L4);
  Serial.print("Angle ... ");      Serial.println(this->angle_of_incline);
  Serial.println("");
*/
}  // End angleFinder_c::method3AngleFinder

/**
   Takes a reading from the sensor based on the supplied sampling rate
   
   @param  sample_number, the number of time the sesor should be read.

   @return The averaged / filtered reading as a float

   @throws Nothing.
*/
float angleFinder_c::takeAReading( int sample_number ) {
  
  float working_value     = 0.0;  // Working variable
  long  running_total     = 0.0;  // Working variable
  int   number_of_samples = 0;
  
  if ( _debug_mode == true ) {
      Serial.print("Debug Mode Samples Count ... ");Serial.println( sample_number ); 
      Serial.print("Continuous Mode ... ");         Serial.println( this->_continuous ); 
      Serial.print("Digital Filter Mode ... ");     Serial.println( this->_filter ); 
  }
  // Sample readings and average
  for (int i = 1; i <= sample_number ; i++) {

      if ( this->_continuous == true ) {
          working_value = (float)this->_sensor.readRangeContinuousMillimeters();
      } else {
          working_value = (float)this->_sensor.readRangeSingleMillimeters();
      }
      
      if (this->_sensor.timeoutOccurred()) { 
          Serial.print(" TIMEOUT ");         
      } else {
                     
          if (this->_filter == true) {
            
              float read_value = working_value;
              
              if (i < FILTER_SAMPLE) {
                  this->_x[i] =  working_value; 
                  this->_m[i] =  working_value;   
              } else {

                  // Shuffle previous reading through array M and X 
                  this->_x[2] = this->_x[1];
                  this->_x[1] = this->_x[0];
                  this->_x[0] = working_value;
      
                  this->_m[2] = this->_m[1];
                  this->_m[1] = this->_m[0];
                  this->_m[0] = 0.0;  
                     
                  // Adjust current SR relative to array ‘a’ index 1 & 2
                  for (int k=1; k < FILTER_SAMPLE; k++ ) {
                      this->_m[0]=this->_m[0] - this->_a[k] * this->_m[0+k];                        
                  }
    
                  // Adjust current SR relative to array ‘b’ index 0,1 & 2 
                  for (int p=0; p < FILTER_SAMPLE; p++) {
                     this->_m[0] = this->_m[0] + this->_b[p] * this->_x[0+p];
                  }
              
                  working_value = this->_m[0];
              }
                             
          } else {
              number_of_samples++;
              running_total = running_total + working_value;  
          } 
          
          if ( _debug_mode == true ) {
               Serial.println( working_value );
          }       
      }
  }

  if (this->_filter == true) {
     if ( _debug_mode == true ) {
         Serial.print("Filtered... ");Serial.println(working_value); 
     }
  } else {
      working_value = (float)(running_total / number_of_samples);
  
      if ( _debug_mode == true ) {
          Serial.print("Average... ");Serial.println(working_value); 
      }
  }
  
  return working_value;
  
} // End angleFinder_c::takeAReading

/**
   Reset the internal variables
   
   @param  None

   @return None
   
   @throws Nothing.
*/
void angleFinder_c::reset( ) {
  
  this->_1st_reading = 0.0;
  this->_2nd_reading = 0.0;
  this->_L4 = 0.0;
  this->_L5 = 0.0;
  this->incline_detected = false;
  
} // End angleFinder_c::reset

/**
   Read test function to drive the sampling code 
   under differnt conditions
   
   @param  int   sample_size, the number of records to read from the sensor
           bool  debug_state, switch on the debug flag to dump out the readings
           bool  continuous_state, switch the mode to continuous
           bool  filter_state, switch on the digital filter

   @return None
   
   @throws Nothing.
*/
void angleFinder_c::readingTest(int  sample_size, 
                                bool debug_state, 
                                bool continuous_state, 
                                bool filter_state ) {

  this->_debug_mode = debug_state;
  this->_continuous = continuous_state;
  this->_filter     = filter_state;

  this->setup();

  delay ( _250_MS ); 
  
  this->takeAReading( sample_size );
  
} // End angleFinder_c::readingTest

/**
   Code Under Test
   
   @param None

   @return None
   
   @throws Nothing.
*/
void angleFinder_c::CUT( ) {
/*
 Serial.println("In the CUT");
float m[26] = {0};// filled with processed signals
float x[26] = {0};
int i=0;
int N=2;
float a[]={1 , -1.64745998107698, 0.700896781188403}; //design the parameter of your filter
float b[]={0.0133592000278565 , 0.0267184000557130 , 0.0133592000278565}; //design the parameter of your filter
this->setup();
delay( 200 );

while(i < 50){
  delay( 200 );
  //Serial.print(_sensor.readRangeContinuousMillimeters());

  // receive the signal and do operation on it
  x[i]= _sensor.readRangeContinuousMillimeters();
  if(i<N)
  {m[i]=x[i];
                 // Serial.print("i ... ");Serial.println(i);
                //  Serial.print("_m[i]");Serial.println(m[i]); 
  }
  else
  {
    for(int k=1;k<=N;k++)
       {
                /*   Serial.print("\ni ... ");Serial.println(i);
                  Serial.print("k ... ");Serial.println(k);
                  Serial.print("Before _m[i]");Serial.println(m[i]);                  
                  Serial.print("_a[k]");Serial.println(a[k]);                  
                  Serial.print("_m[i+k]");Serial.println(m[i-k]);    
          m[i]=m[i]-a[k]*m[i-k];
           //   Serial.print("after _m[i]");Serial.println(m[i]); 
     
        }
    for(int p=0;p<=N;p++)
       {m[i]=m[i]+b[p]*x[i-p];}
    //m[i]=(-1*(a[1]*m[i-1]+a[2]*m[i-2])+b[0]*x[i]+b[1]*x[i-1]+b[2]*x[i-2])/a[0]; main purpose for upper two 'for',signal processing
  }
  i=i+1;
  //
  if (_sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
  //See the result
  if(i>20)
  { 

    Serial.println("Print x Results");
    //data before process
      for(int k=0;k<i;k++)
    {
      Serial.println(x[k]);
    }
    //data after process
    Serial.println("Print m Results");
      for(int k=0;k<i;k++)
    {
      Serial.println(m[k]);
    }

    Serial.println("***************** DONE ****************");
    return;
  }
}
*/
} // End angleFinder_c::CUT

#endif
