// Arduino analog input pin to which the Sharp sensor is connected
const byte SHARP_PIN_FRONT = A5;
const byte SHARP_PIN_LEFT = A4;
const byte SHARP_PIN_RIGHT = A3;

// Variables to store the proximity measurement
int sharp_val_front = 0; // integer read from analog pin
float sharp_range_front; // range measurement [cm]

int sharp_val_left = 0;
float sharp_range_left;

int sharp_val_right = 0;
float sharp_range_right;

float convertToCm(int num)
{
  float dist = 1578.9*pow(num, 0.82) + 5;
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
  sharp_val_front = analogRead(SHARP_PIN_FRONT);
  sharp_range_front = convertToCm(sharp_val_front);

  sharp_val_left = analogRead(SHARP_PIN_LEFT);
  sharp_range_left = convertToCm(sharp_val_left);

  sharp_val_right = analogRead(SHARP_PIN_RIGHT);
  sharp_range_right = convertToCm(sharp_val_right);

  // Print all values
  Serial.print(sharp_range_left);
  Serial.print("\t |");
  Serial.print(sharp_range_front);
  Serial.print("\t |");
  Serial.print(sharp_range_right);
  Serial.print("\n");

  // Delay for a bit before reading the sensor again
  delay(500);
}
