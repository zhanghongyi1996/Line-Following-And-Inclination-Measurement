/**
   A file to hold the ISR routines
   File currently encapsulates routies to set up the timer 3 used to update the kinimtrics
*/ 
#ifndef _ISR_H
#define _ISR_H 

/**
  // Function Protypes
*/
void  updateKinematics();

/**
  Routine to setupt timer3 to run 

  @param  None.

  @return void.

  @throws Nothing.

  @exceptsafe No Checks.
*/
void setupTimer3() {
  
  // disable global interrupts
  cli();          

  // Reset timer3 to a blank condition.
  // TCCR = Timer/Counter Control Register
  TCCR3A = 0;     // set entire TCCR3A register to 0
  TCCR3B = 0;     // set entire TCCR3B register to 0

  // First, turn on CTC mode.  Timer3 will count up
  // and create an interrupt on a match to a value.
  // See table 14.4 in manual, it is mode 4.
  TCCR3B = TCCR3B | (1 << WGM32);

  // For a cpu clock precaler of 256:
  // Shift a 1 up to bit CS32 (clock select, timer 3, bit 2)
  // Table 14.5 in manual. 
  TCCR3B = TCCR3B | (1 << CS32);
  
  // set compare match register to desired timer count.
  // CPU Clock  = 16000000 (16mhz).
  // Prescaler  = 256
  // Timer freq = 16000000/256 = 62500
  // We can think of this as timer3 counting up to 62500 in 1 second.
  // compare match value = 62500 / 2 (we desire 2hz).

  // OCR3A = 31250; // 2hz
  // OCR3A = 15625; // 4hz
  // OCR3A = 1562;  // 40hz
  // OCR3A = 125;   // 500hz
  // OCR3A = 62;    // 1000hz

  OCR3A = OCR3A_500HZ;
    
  // enable timer compare interrupt:
  TIMSK3 = TIMSK3 | (1 << OCIE3A);

  // enable global interrupts:
  sei(); 

} // End setupTimer3



/**
  The ISR routine.
  The name TIMER3_COMPA_vect is a special flag to the compiler.  
  It automatically associates with Timer3 in CTC mode.
*/
ISR( TIMER3_COMPA_vect ) {
    
  // Call to update the ongoing kinematics calculations
  updateKinematics();

} // End ISR TIMER3_COMPA_vect

#endif
