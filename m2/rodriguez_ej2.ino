// Global variables definitions

int leds[]={5,6,7,LED_BUILTIN}; // Digital pins 5, 6, 7 y 13 (13 -> internal led)
int buttonState = 0; // button state variabnle, intialized as 0 (unpressed)

void setup() {
  // Initialize digital pin 4 as input and digital pins (5, 6, 7 and 13) as an output. 
  pinMode(4, INPUT);
  
  pinMode(leds[0], OUTPUT);
  pinMode(leds[1], OUTPUT);
  pinMode(leds[2], OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  blink(); // Call the private function
}

void blink()
{ 
  buttonState = digitalRead(4);       // Read the state of the button
  for (int i=0; i < 4; i++){          // Main loop to iterate all the pins set as output
    if (buttonState == HIGH) {        // If, button is pressed
      pinMode(leds[i],OUTPUT);
      digitalWrite(leds[i], HIGH);    // turn the LED on (HIGH is the voltage level)
      delay(200);                     // wait for a 200 ms.
      digitalWrite(leds[i], LOW);     // turn the LED off by making the voltage LOW
      delay(200);                     // wait for a 200 ms.
    }
  }  
}