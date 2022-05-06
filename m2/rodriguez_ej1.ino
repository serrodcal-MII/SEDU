// Global variables definitions

int leds[]={5,6,7,LED_BUILTIN}; // Digital pins 5, 6, 7 y 13 (13 -> internal led)

void setup() {
  // Initialize digital pins (5, 6, 7 and 13) as OUTPUT
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
  for (int i=0; i < 4; i++){       // Main loop to itarete leds
    pinMode(leds[i],OUTPUT);
    digitalWrite(leds[i], HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(200);                    // wait for a 200 ms.
    digitalWrite(leds[i], LOW);    // turn the LED off by making the voltage LOW
    delay(200);                    // wait for a 200 ms.
  }  
}