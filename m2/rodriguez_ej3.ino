// Global variables definitions

int leds[]={9,10,11}; // Digital pins 9, 10 and 11
int measure = 0; // variable to store the value read by the sensor
int light = 0; // variable to set the value of the light depending on the measuremente

void setup() {
  // Initialize digital pins (9, 10 and 11) as an output. 
  pinMode(leds[0], OUTPUT);
  pinMode(leds[1], OUTPUT);
  pinMode(leds[2], OUTPUT);
  
  Serial.begin(9600); // To be able to monitorize
}

void loop() {
  lightUp(); // Call the private function
}

// Private function to monitor the execution
void monitoring() {
  Serial.println("___________");
  Serial.print("Measurement: ");
  Serial.println(measure);
  Serial.print("Light: ");
  Serial.println(light);
  delay(1000);
}

void lightUp() {
  measure=analogRead(0); // Get the value from the light sensor

  // Depending on the measurement, we set a value to global variable light according to the exercise
  if (measure >= 0 && measure <= 255) {
    light = 255;
  } else if (measure >= 256 && measure <= 511) {
    light = 191;
  } else if (measure >= 512 && measure <= 767) {
    light = 127;
  } else {
    light = 64;
  }
  
  monitoring(); // Call private function to monitor the execution once we have read the measurement from the light sensor
  for (int i=0; i < 3; i++){
    analogWrite(leds[i],light); // Setting light value to the pins
    delay(200);                 // Wait for a 200ms.
  }
}
