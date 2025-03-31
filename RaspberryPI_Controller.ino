//--------------------------------------------- VARIABLES -----------------------------------------------
// Encoder digital pins
const byte SIGNAL_AL = 4;
const byte SIGNAL_BL = 3;
const byte SIGNAL_AR = 2;
const byte SIGNAL_BR = 1;

#define ERR_COUNT 5

//radius of wheels and track
const float radius = 0.0625;
const float track = 0.2775;

int EA = 5;   // Wheel PWM pin (must be a PWM pin)
int I1 = 12;  // Wheel direction digital pin 1
int I2 = 11;  // Wheel direction digital pin 2
int EB = 6;
int I3 = 10;
int I4 = 9;


float vTotal = 0;
float angleRate = 0;


// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticksL = 0;
volatile long encoder_ticksR = 0;

//enncoder ticks to radians conversion rate (rad/tick)
const double RPT = 2.0 * PI / 3000;

// Counter to keep track of the last number of ticks [integer]
long encoder_ticks_last = 0;
float lastDisplacementL = 0;
float lastDisplacementR = 0;


//holds desired speed values from pi
float strSpeeds[2] = { 0.0, 0.0 };
//desired speeds and sum of all errors

double sumErrorR = 0;
double sumErrorL = 0;
double lastErrorL = 0;
double lastErrorR = 0;
short uL, uR;
float k_p = 190.0;
float k_i = 30.0;
float k_d = 32.0;

//speeds
float vNow = 0;
float omegaNow = 0;

// Integral error accumulation
float error_array_L[ERR_COUNT];
float error_array_R[ERR_COUNT];
int error_array_counter = 0;



// -------------------------------------------- SET UP --------------------------------------------------
void setup() {
  // Open the serial port at 57600 bps
  Serial.begin(57600);
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

  init_err_array(error_array_L);  // Initialize error accumulation array
  init_err_array(error_array_R);
}


// -------------------------------------------- LOOP ----------------------------------------------------


void loop() {
  if (error_array_counter == (ERR_COUNT - 1)) {
    error_array_counter = 0;
  }

  String serial_buffer = "Default";

  strSpeeds[0] = 0.0;
  strSpeeds[1] = 0.0;

  serial_buffer = readSerial();
  seperateString(serial_buffer);

  float V_desired = strSpeeds[0];
  float omega_desired = strSpeeds[1];


  if (V_desired == 0.0 && omega_desired == 0.0) {
    motors(0, 0);
  } else {
    float vL = findVelocityL();
    float vR = findVelocityR();
    findSpeeds(vL, vR);

    float errorL = findErrorL(vL, V_desired, omega_desired);
    float errorR = findErrorR(vR, V_desired, omega_desired);

    error_array_L[error_array_counter] = errorL;
    error_array_R[error_array_counter] = errorR;

    // Accumulate integral error
    sumErrorL = accumulate(error_array_L);
    sumErrorR = accumulate(error_array_R);

    uL = PIDController(errorL, sumErrorL, lastErrorL, k_p, k_i, k_d);
    uR = PIDController(errorR, sumErrorR, lastErrorR, k_p, k_i, k_d);

    motors(uL, uR);
  }

  error_array_counter++;
}



//--------------------------------------------- ENCODER FUNCTIONS ---------------------------------------
void decodeEncoderTicksLeft() {
  if (digitalRead(SIGNAL_BL) == LOW) {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticksL--;
  } else {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticksL++;
  }
}

void decodeEncoderTicksRight() {
  if (digitalRead(SIGNAL_BR) == LOW) {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticksR++;
  } else {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticksR--;
  }
}



//--------------------------------------------- MOTORS --------------------------------------------------
void motors(short speedL, short speedR) {

  if (speedL == 0 && speedR == 0) {
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);

    analogWrite(EA, 0);
    analogWrite(EB, 0);
    return;
  } else {
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
  }

  analogWrite(EA, abs(speedR));
  analogWrite(EB, abs(speedL));
}

//--------------------------------------------- FIND VELOCITIES -----------------------------------------
//returns current velocity of each wheel
float findVelocityL() {
  float displacementL = (double)encoder_ticksL * RPT;            //finds displacement
  float v = (displacementL - lastDisplacementL) * radius / 0.5;  //find speed in m/s
  lastDisplacementL = displacementL;                             //store last displacement
  return v;                                                      //return v
}
float findVelocityR() {
  float displacementR = (double)encoder_ticksR * RPT;            //finds displacement
  float v = (displacementR - lastDisplacementR) * radius / 0.5;  //find speed in m/s
  lastDisplacementR = displacementR;                             //store last displacement
  return v;                                                      //return v
}

//updates variables for current speed and turning
void findSpeeds(float vL, float vR) {
  vNow = 0.5 * (vR + vL);
  omegaNow = (vR - vL) * (1.0 / track);
}


//returns velocity of each wheel required for desired speeds
float requiredR(float V_desired, float omega_desired) {
  float Vreq = V_desired + (omega_desired * track) / 2;
  return Vreq;
}


float requiredL(float V_desired, float omega_desired) {
  float Vreq = V_desired - (omega_desired * track) / 2;
  return Vreq;
}


// -------------------------------------------- FIND ERRORS ---------------------------------------------
float findErrorL(float vL, float V_desired, float omega_desired) {
  float errorL = requiredL(V_desired, omega_desired) - vL;  //find error in velocity
  // sumErrorL += errorL;                                      //add error to integral term
  return errorL;  //return error
}


float findErrorR(float vR, float V_desired, float omega_desired) {
  float errorR = requiredR(V_desired, omega_desired) - vR;
  // sumErrorR += errorR;
  return errorR;
}


// -------------------------------------------- CONTROLLER ----------------------------------------------
short PIDController(float errorNow, double &sumError, double &lastError, float kP, float kI, float kD) {
  // sumError += errorNow;                 // Integrate the error
  float dError = errorNow - lastError;  // Derivative of error
  lastError = errorNow;                 // Store the last error for the next iteration

  // Compute control output
  short u = (short)(kP * errorNow + kI * sumError + kD * dError);

  // Bound u between (-255, 255)
  if (u > 255) return 255;
  if (u < -255) return -255;
  return u;
}

// -------------------------------------------- SERIAL COM ----------------------------------------------
//reads string from pi
String readSerial() {
  String serial_buffer;
  // while serial is available read input into the buffer
  do {
    if (Serial.available() > 0) {
      serial_buffer = Serial.readStringUntil('\n');
      // Serial.println(serial_buffer);
    }
  } while (Serial.available() <= 0);

  return serial_buffer;
}


//converts pi string to float values

void seperateString(String string) {
  int index = string.indexOf(':');
  if (index != -1) {
    strSpeeds[0] = string.substring(0, index).toFloat();
    strSpeeds[1] = string.substring(index + 1).toFloat();
  }
}


void init_err_array(float error_array[]) {
  for (int i = 0; i < ERR_COUNT; i++) {
    error_array[i] = 0.0;
  }
}

float accumulate(float error_array[]) {
  float accumulator = 0.0;
  for (int i = 0; i < ERR_COUNT; i++) {
    accumulator += error_array[i];
  }

  return accumulator;
}
