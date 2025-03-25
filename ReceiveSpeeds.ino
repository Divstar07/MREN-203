
String str;

void setup() {
  Serial.begin(9600);
}
void loop() {
   
  do {
    if (Serial.available() > 0) {
      str = Serial.readStringUntil('\n');
      Serial.print("You sent me: ");
      Serial.println(str);
    }
  } while (Serial.available() <= 0);
  float speeds[2] = {0.0, 0.0}; // V and W
  int j = 0; //index for speeds
  bool fraction = false; 

  // Serial.print("Turning Velocity is: ");
  // Serial.print(str[0]);
  // Serial.print(" + ");
  // Serial.println(str[2]);

  speeds[0] += ((float)str[0] - 48);
  speeds[0] += ((float)str[2] - 48) / 10;

  speeds[1] += ((float)str[4] - 48);
  speeds[1] += ((float)str[6] - 48) / 10; 
 
  // for (int i = 0; i < sizeof(str)/sizeof(str[0]); i++) { 
  //   if(str[i] == '.') {
  //     fraction = true;
      
  //   } else if(str[i] == ':') {
  //     j++; 
  //     fraction = false;
  //   } else {
  //     if(fraction) {
  //       speeds[j] += ((float)str[i] - 48) / 10;
  //     }else {
  //       speeds[j] += (float)str[i] - 48;
  //     }
  //     fraction = false;
  //   }
  // } 

  Serial.print("Forward Velocity is: ");
  Serial.println(speeds[0]);
  Serial.print("turning velocity is: ");
  Serial.println(speeds[1]);

}