#include <Servo.h>

Servo servo;

const int inputLDR = A0;  // Analog input number 0

int ldrValue = 0; 
int angle = 0; 

void setup() {
  pinMode(inputLDR, INPUT);
  servo.attach(7); // Using PIN 7
}

void loop() {
  run();
  
  delay(100);
}

void run() {
  ldrValue = analogRead(inputLDR); // Read ldr sensor

  if (ldrValue > 650 && ldrValue <= 1024) { 
    angle = 180; // max angle
  } else if (ldrValue > 350 && ldrValue <= 650) { 
    angle = 90; // mid angle
  } else { 
    angle = 0; // min angle
  }
  
  servo.write(angle);
}
