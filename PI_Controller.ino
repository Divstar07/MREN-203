//--------------------------------------------- VARIABLES -----------------------------------------------
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
    float angleRate = 0;

  // Counter to keep track of encoder ticks [integer]
  volatile long encoder_ticksL = 0;
  volatile long encoder_ticksR = 0;

  //enncoder ticks to radians conversion rate (rad/tick)
  const double RPT = 2.0 * PI / 3000 ;

  // Counter to keep track of the last number of ticks [integer]
  long encoder_ticks_last = 0;
  float lastDisplacementL = 0;
  float lastDisplacementR = 0;

  //desired speeds and sum of all errors
  float V_desired = 0;
  float omega_desired = 4;
  float sumErrorR = 0;
  float sumErrorL = 0;

  //speeds
  float vNow = 0;
  float omegaNow = 0;




//--------------------------------------------- ENCODER FUNCTIONS ---------------------------------------
  void decodeEncoderTicksLeft(){
      if (digitalRead(SIGNAL_BL) == LOW){
    // SIGNAL_A leads SIGNAL_B, so count one way
        encoder_ticksL--;
      }
      else{
        // SIGNAL_B leads SIGNAL_A, so count the other way
        encoder_ticksL++;
      }
    }
    void decodeEncoderTicksRight(){
      if (digitalRead(SIGNAL_BR) == LOW){
        // SIGNAL_A leads SIGNAL_B, so count one way
        encoder_ticksR++;
        }
      else{
        // SIGNAL_B leads SIGNAL_A, so count the other way
      encoder_ticksR--;
        }
      }



//--------------------------------------------- MOTORS --------------------------------------------------
  void motors(short speedL, short speedR){ 

    if (speedL < 0 && speedR < 0) {  // backward
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
  } else if (speedL > 0 && speedR > 0) {  // forward
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
  } else if (speedL > 0 && speedR < 0) {  // turn left
    digitalWrite(I2, HIGH);
    digitalWrite(I1, LOW);
    digitalWrite(I4, HIGH);
    digitalWrite(I3, LOW);
  } else if (speedL < 0 && speedR > 0) {  // turn right
    digitalWrite(I2, LOW);
    digitalWrite(I1, HIGH);
    digitalWrite(I4, LOW);
    digitalWrite(I3, HIGH);
  
  }

    analogWrite(EA, abs(speedR));
    analogWrite(EB, abs(speedL));
    }

//--------------------------------------------- FIND VELOCITIES -----------------------------------------
  //returns current velocity of each wheel
  float findVelocityL() {
    float displacementL = (double)encoder_ticksL * RPT;     //finds displacement 
    float v = (displacementL - lastDisplacementL) * radius / 0.5; //find speed in m/s
    lastDisplacementL = displacementL;                      //store last displacement
    return v;                                               //return v 
    }   
  float findVelocityR() {
    float displacementR = (double)encoder_ticksR * RPT;     //finds displacement 
    float v = (displacementR - lastDisplacementR) * radius / 0.5; //find speed in m/s
    lastDisplacementR = displacementR;                      //store last displacement
    return v;                                               //return v
    } 

      //updates variables for current speed and turning
    void findSpeeds(float vL, float vR) {
        vNow = 0.5 * (vR + vL);
        omegaNow = (vR - vL) * (1.0 / track);
    }

    
    //returns velocity of each wheel required for desired speeds
  float requiredR() {
      float Vreq = V_desired + (omega_desired*track)/2;
      Serial.print("reqR = ");
      Serial.print(Vreq);
      Serial.print("\n");
      return Vreq;
    }
  float requiredL() {
      float Vreq = V_desired - (omega_desired*track)/2;
      Serial.print("reqL = ");
      Serial.print(Vreq);
      Serial.print("\n");
      return Vreq;
    }


// -------------------------------------------- FIND ERRORS ---------------------------------------------
  float findErrorL(float vL) { 
    float errorL = requiredL() - vL; //find error in velocity
    
      sumErrorL += errorL;             //add error to integral term
      // Serial.print("error summed L \n");
    
      Serial.print("errorL = ");
      Serial.print(errorL);
      Serial.print("\n");

    return errorL;                    //return error 
  }
  float findErrorR(float vR) { 
    float errorR = requiredR() - vR;
    
      sumErrorR += errorR;
      // Serial.print("error summed R \n");
    
    Serial.print("    errorR = ");
    Serial.print(errorR);
    Serial.print("\n");

    return errorR;
    
  }


// -------------------------------------------- CONTROLLER ----------------------------------------------
  short PController (float errorNow, float sumError, float kP, float kI) {
    short u = (short) (kP*errorNow + sumError*kI);  
    if (u > 255) {                  // bound u between (-255,255)
      return 255; 
    } else if (u < -255) {
      return -255;
    }
    return u;
  }

// -------------------------------------------- SET UP --------------------------------------------------
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


// -------------------------------------------- LOOP ----------------------------------------------------
 void loop()
 {

    //initialize variables for speed at current time interval
    float vL = findVelocityL();
    float vR = findVelocityR();
    Serial.print("VL = ");
    Serial.print(vL);
    Serial.print("    VR = ");
    Serial.print(vR);
    Serial.print("\n");

    findSpeeds(vL, vR);
    Serial.print("Velocity total is: ");
    Serial.print(vNow);
    Serial.print("       Turning rate is: ");
    Serial.print(omegaNow);
    Serial.print("\n");

    short uL, uR;
    //set uL, and uR and run motors at speed
   uL = PController(findErrorL(vL), sumErrorL, 220, 80);

   uR = PController(findErrorR(vR), sumErrorR, 220, 80);
   Serial.print("UL = ");
   Serial.print(uL);
   Serial.print("    UR = ");
   Serial.print(uR);
   Serial.print("\n");


    motors(uL,uR);

    //Short delay [ms]
    delay(500);
    Serial.print("\n");
  }

