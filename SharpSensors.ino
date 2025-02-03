// Arduino analog input pin to which the Sharp sensor is connected
const byte SHARP_PIN = A5;

// Variables to store the proximity measurement
int sharp_val = 0; // integer read from analog pin
float sharp_range; // range measurement [cm]

float convertToCm(int num)
{
  float dist = 1578.9*pow(num, 0.82);
  return dist;
}

void setup()
{
// Open the serial port at 115200 bps
  Serial.begin(115200);
}

void loop()
{
  // Read the sensor output (0-1023, which is 10 bits and fits inside an Arduino int-type),→
  sharp_val = analogRead(SHARP_PIN);
  sharp_range = convertToCm(sharp_val);

  // Print all values
  Serial.print(sharp_range);
  Serial.print("\n");

  // Delay for a bit before reading the sensor again
  delay(500);
}
