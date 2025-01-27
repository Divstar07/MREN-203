
// Encoder digital pins
 const byte SIGNAL_AL = 4;
 const byte SIGNAL_BL = 3;
 const byte SIGNAL_AR = 2;
 const byte SIGNAL_BR = 1;

 //radius of wheels and track
 const float radius = 0.0625;
 const float track = 0.2775;

 int EA = 5; // Wheel PWM pin (must be a PWM pin)
 int I1 = 12; // Wheel direction digital pin 1
 int I2 = 11; // Wheel direction digital pin 2
 int EB = 6;
 int I3 = 10;
 int I4 = 9;

  float vTotal = 0;
  float  angleRate = 0;

 // Counter to keep track of encoder ticks [integer]
 volatile long encoder_ticksL = 0;
 volatile long encoder_ticksR = 0;
 long displacementL = 0;
 long displacementR = 0;

//enncoder ticks to radians conversion rate (rad/tick)
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




//called when Signal_AR goes high
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



//run motors 
void motors(int speedL, int speedR){ 
  digitalWrite(I1, LOW);
  digitalWrite(I2, HIGH);
  digitalWrite(I3, HIGH);
  digitalWrite(I4, LOW);

  analogWrite(EA, speedR);
  analogWrite(EB, speedL);
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

 pinMode(EA, OUTPUT);
 pinMode(I1, OUTPUT);
 pinMode(I2, OUTPUT);
 pinMode(EB, OUTPUT);
 pinMode(I3, OUTPUT);
 pinMode(I4, OUTPUT);

 // Every time SIGNAL_A goes HIGH, this is a pulse
 attachInterrupt(digitalPinToInterrupt(SIGNAL_AL), decodeEncoderTicksLeft, RISING);
 attachInterrupt(digitalPinToInterrupt(SIGNAL_AR), decodeEncoderTicksRight, RISING);

 // Print a message
 Serial.print("Program initialized.");
 Serial.print("\n");
 }

  void speedometer() {
      //left wheel readings
  displacementL = (double)encoder_ticksL * RPT;     //finds displacement 
  // long speedL = displacementL - lastDisplacementL;   //angular speed
  float velocityL = (displacementL - lastDisplacementL) * radius;                 //speed in m/s
  lastDisplacementL = displacementL;                //set last_displacement to current displacement 


  //right wheel readings 
  displacementR = (double)encoder_ticksR * RPT;
  float velocityR = (displacementR - lastDisplacementR) * radius; 
  lastDisplacementR = displacementR; 


  //calculate total translational velocity and angular rate 
  vTotal = 0.5 * (velocityL + velocityR);
  angleRate = (velocityR - velocityL) / track;

  // print results 
  Serial.print("Total Translational speed is: ");
  Serial.print(vTotal);
  Serial.print("m/s \n");

  Serial.print("The angular turning rate is: ");
  Serial.print(angleRate);
  Serial.print("Rad/s \n");


  }






 void loop()
 {
    int uL = 127;
    int uR = 127;
    motors(uL, uR);
    speedometer();

    //Short delay [ms]
    delay(1000);
    Serial.print("\n");
}



