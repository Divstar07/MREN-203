
// Encoder digital pins
 const byte SIGNAL_AL = 4;
 const byte SIGNAL_BL = 3;
 const byte SIGNAL_AR = 2;
 const byte SIGNAL_BR = 1;


 // Counter to keep track of encoder ticks [integer]
 volatile long encoder_ticksL = 0;
 volatile long encoder_ticksR = 0;
 long displacementL = 0;
 long displacementR = 0;
//ticks to rad conversion rate
 const double RPT = 2.0 * PI / 3000 ;

 // Counter to keep track of the last number of ticks [integer]
 long encoder_ticks_last = 0;
 long lastDisplacementL = 0;
 long lastDisplacementR = 0;
 
 // This function is called when SIGNAL_AL goes HIGH
 void decodeEncoderTicksLeft()
 {
 if (digitalRead(SIGNAL_BL) == LOW)
 {
 // SIGNAL_A leads SIGNAL_B, so count one way
 encoder_ticksL--;
 }
 else
 {
 // SIGNAL_B leads SIGNAL_A, so count the other way
 encoder_ticksL++;
 }
 }

  void decodeEncoderTicksRight()
 {
 if (digitalRead(SIGNAL_BR) == LOW)
 {
 // SIGNAL_A leads SIGNAL_B, so count one way
 encoder_ticksR++;
 }
 else
 {
 // SIGNAL_B leads SIGNAL_A, so count the other way
 encoder_ticksR--;
 }
 }

 void setup()
 {
 // Open the serial port at 9600 bps
 Serial.begin(9600);
 // Set the pin modes for the encoders
 pinMode(SIGNAL_AL, INPUT);
 pinMode(SIGNAL_BL, INPUT);
 pinMode(SIGNAL_AR, INPUT);
 pinMode(SIGNAL_BR, INPUT);

 // Every time SIGNAL_A goes HIGH, this is a pulse
 attachInterrupt(digitalPinToInterrupt(SIGNAL_AL), decodeEncoderTicksLeft, RISING);
 attachInterrupt(digitalPinToInterrupt(SIGNAL_AR), decodeEncoderTicksRight, RISING);

 // Print a message
 Serial.print("Program initialized.");
 Serial.print("\n");
 }

 void loop()
 {
//  ticks_last = encoder_ticks;


  displacementL = (double)encoder_ticksL * RPT;
  long speedL = displacementL - lastDisplacementL; 
  Serial.print("Left Wheel Angular Speed is: ");
  Serial.print(speedL);
  Serial.print ("Rad/s");
  Serial.print("\n");

  lastDisplacementL = displacementL;

/* -------------------------------------------- */

  displacementR = (double)encoder_ticksR * RPT;
  long speedR = displacementR - lastDisplacementR; 
  Serial.print("Right Wheel Angular Speed is: ");
  Serial.print(speedR);
  Serial.print ("Rad/s");
  Serial.print("\n");

  lastDisplacementR = displacementR; 
 // Short delay [ms]
 delay(1000);
}