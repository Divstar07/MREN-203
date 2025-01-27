/**
2 * @file PWM-motor-control.ino
3 * @author Joshua Marshall (joshua.marshall@queensu.ca)
4 * @brief Arduino program to drive one wheel motor through a motor driver.
5 * @version 2.0
6 * @date 2022-12-05
7 *
8 * @copyright Copyright (c) 2021-2022
9 *
10 */

 int EA = 5; // Wheel PWM pin (must be a PWM pin)
 int I1 = 12; // Wheel direction digital pin 1
 int I2 = 11; // Wheel direction digital pin 2
 int EB = 6;
 int I3 = 10;
 int I4 = 9;

 void setup()
 {
 // Configure digital pins for output
 pinMode(EA, OUTPUT);
 pinMode(I1, OUTPUT);
 pinMode(I2, OUTPUT);
 pinMode(EB, OUTPUT);
 pinMode(I3, OUTPUT);
 pinMode(I4, OUTPUT);
 }

 void loop()
 {
    int u = 191; // A variable for the motor PWM command [0-255]
    figureEight(u);
    // rotations(u);
 }


  // void Rotations(int u){
  //   turnCW(u); 
  //   delay(2000);
  //   stop();
  //   delay(750);
  //   turnCCW(u);
  //   delay(200);
  //   stop();
  //   delay(3000);

  // }

//turns all motors off
void stop() {
    digitalWrite(I1, LOW);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, LOW);

    analogWrite(EA, 0);
    analogWrite(EB, 0);

}
void start(int speedR, int speedL) {
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);

    analogWrite(EA, speedR);
    analogWrite(EB, speedL);

}

//Clockwise tank turn
 void turnCW(int speedL, int speedR) {
  
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
    
    analogWrite(EA, speedR);
    analogWrite(EB, speedL);

    delay(10);
 }

//counterclockwise tank turn 
 void turnCCW(int speedL, int speedR) {
  
    digitalWrite(I2, LOW);
    digitalWrite(I1, HIGH);
    digitalWrite(I4, LOW);
    digitalWrite(I3, HIGH);
    
    analogWrite(EA, speedR);
    analogWrite(EB, speedL);

    delay(10);
 }



 void figureEight(int u){
    // goes straight 
    start(u,u);
    delay(1000);
    stop();
    //turns in cricle half arc
    turnCCW(u,u/2);
    delay(1000);
    stop();
    //goes straight again
    start(u,u);
    delay(1000);
    stop();

    //turn again, Oppisite direction 
    turnCW(u/2,u);
    delay(1000);
    stop(); 
    
    //repeat
    //profit 




 }
