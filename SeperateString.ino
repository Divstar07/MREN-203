
char  str[7] = "1.5:3.2";  

bool fraction = false; 

void setup() {
  Serial.begin(9600);
}
void loop() {
  float speeds[2] = {0.0, 0.0}; // V and W
  int j = 0; //index for speeds

  for (int i = 0; i < sizeof(str)/sizeof(str[0]); i++) { 
    if(str[i] == '.') {
      fraction = true;
      
    } else if(str[i] == ':') {
      j++; 
      fraction = false;
    } else {
      if(fraction) {
        speeds[j] += ((float)str[i] - 48) / 10;
      }else {
        speeds[j] += (float)str[i] - 48;
      }
      fraction = false;
    }
  } 

  

  Serial.print("forward velocity is: ");
  Serial.println(speeds[0]);

  Serial.print("turning velocity is: ");
  Serial.println(speeds[1]);

  delay(2500);


}