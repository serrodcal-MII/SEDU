#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#include <DHT.h>
#include <Servo.h>

// NOTA: después de la sesion sincrona, el profesor comento que se podia usar solo LDR y DHT, y solo un actuador, servo o motor dc. 
// En este caso se usa solo un actuador de tipo servo por simplicidad.

// DHT22
// Define DHT type and DHT pin 
#define DHTTYPE DHT22
#define DHTPIN 7

// Servo

Servo servo;

// define tasks
void TaskReadFromSerial( void *pvParameters ); // Get commands
void TaskWriteToSerial( void *pvParameters );
void TaskSensors( void *pvParameters );
void TaskActuators( void *pvParameters );

//define smaphore handlers
SemaphoreHandle_t sensorSem;

//define queue
QueueHandle_t sensorQueue;
QueueHandle_t actuatorQueue;

//define DHT sensor
DHT dhtSensor(DHTPIN, DHTTYPE);

// Define stucts
struct message 
{
  int ldr;  // LDR
  double humidity; // DHT
  double temperature; // DHT
};

// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect.
  }

  // initialize DHT sensor communication:
  dhtSensor.begin();

  servo.attach(9); // Using PIN 9
  servo.write(0); // Initialize

  sensorSem = xSemaphoreCreateBinary();

  sensorQueue = xQueueCreate( 5, sizeof( message ) );
  actuatorQueue = xQueueCreate( 5, sizeof( int ) );

  // Now set up tasks to run independently.

  xTaskCreate(
    TaskActuators
    ,  (const portCHAR *) "Actuators"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  4  // Priority, with 1 being the highest, and 4 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskWriteToSerial
    ,  (const portCHAR *) "WriteToSerial"   // A name just for humans
    ,  256  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 1 being the highest, and 4 being the lowest.
    ,  NULL );
  
  xTaskCreate(
    TaskSensors
    ,  (const portCHAR *) "Sensors"   // A name just for humans
    ,  512  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 1 being the highest, and 4 being the lowest.
    ,  NULL );

   xTaskCreate(
    TaskReadFromSerial
    ,  (const portCHAR *) "ReadFromSerial"
    ,  256  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskReadFromSerial( void *pvParameters )
{
  (void) pvParameters;

  int state = 1;
  int pos = 0;
  
  for (;;)
  {   
    if(Serial.available()>0){
      
      char c = Serial.read();
      if (c != '\n'){

        switch(state){
            case 1:
              if (c == '[') {
                state = 2;
              } else {
                Serial.print('[');
                Serial.print('E');
                Serial.println(']');
                state = 1;
              }
              break;
            case 2:
              if (c == 'S') {
                state = 7;
              } else if (c == 'A') {
                state = 3;
              } else {
                Serial.print('[');
                Serial.print('E');
                Serial.println(']');
                state = 1;
              }         
              break;
            case 3:
              if (c == ',') {
                state = 4;            
              } else {
                Serial.print('[');
                Serial.print('E');
                Serial.println(']');
                state = 1;
              }
              break;
            case 4:
              if (c == '0') { // single actuator -> servo
                state = 5;            
              } else {
                Serial.print('[');
                Serial.print('E');
                Serial.println(']');
                state = 1;
              }
              break;
            case 5:
              if (c == ',') {
                state = 6;            
              } else {
                Serial.println("[E]");
                state = 1;
              }
              break;
            case 6:
              if (isDigit(c)) {
                pos = c - '0'; // convert to int
                state = 8;            
              } else {
                Serial.print('[');
                Serial.print('E');
                Serial.println(']');
                state = 1;
              }
              break;
            case 7:
              if (c == ']') {
                xSemaphoreGive(sensorSem);
              } else {
                Serial.print('[');
                Serial.print('E');
                Serial.println(']');          
              }
              state = 1;       
              break;
            case 8:
              if (c == ']') {
                xQueueSend(actuatorQueue, &pos, portMAX_DELAY);  // Send value to other tasks
              } else {
                Serial.print('[');
                Serial.print('E');
                Serial.println(']');             
              }
              state = 1;       
              break;
            default:
              break;
        }
      }      
    }
    vTaskDelay( 250 / portTICK_PERIOD_MS ); 
  }
}

void TaskWriteToSerial( void *pvParameters )
{
  (void) pvParameters;

  for (;;)
  {   
    struct message current;
    if (xQueueReceive( sensorQueue, &current, portMAX_DELAY ) == pdTRUE) {
      uint8_t checksum = 0; // Equals to byte
      
      Serial.print('[');
      Serial.print('O');
      uint8_t ldr = (uint8_t) ( (float(current.ldr)/1024) * 255 ); // Equals to byte, between 0 and 255
      checksum = checksum + ldr;
      Serial.print(ldr);
      Serial.print(',');
      uint8_t temperature = (uint8_t) current.temperature; // Equals to byte
      checksum = checksum + temperature;
      Serial.print(temperature);
      Serial.print(',');
      uint8_t humidity = (uint8_t) current.humidity; // Equals to byte
      checksum = checksum + humidity;
      Serial.print(humidity);
      Serial.print(',');
      Serial.print(checksum);
      Serial.println(']');
    }
    
    vTaskDelay( 250 / portTICK_PERIOD_MS ); 
  }
  
}

void TaskSensors( void *pvParameters )
{
  (void) pvParameters;

  // LDR
  pinMode(A0, INPUT);
    
  for (;;)
  {
    xSemaphoreTake(sensorSem, portMAX_DELAY);

    struct message current;
     
    current.ldr = analogRead(A0); // LDR
    
    // Get temperature and humidity.
    current.temperature = dhtSensor.readTemperature();
    current.humidity = dhtSensor.readHumidity();
    
    xQueueSend(sensorQueue, &current, portMAX_DELAY);  // Send value to other tasks
    
    vTaskDelay(250 / portTICK_PERIOD_MS); 
  }
  
}

// NOTA: después de la sesion sincrona, el profesor comento que se podia usar solo LDR y DHT, y solo un actuador, servo o motor dc. 
// En este caso se usa solo un actuador de tipo servo por simplicidad.
void TaskActuators( void *pvParameters )
{
  (void) pvParameters;

  int pos = 0;
  int angle = 0;

  for (;;)
  {  

    if (xQueueReceive( actuatorQueue, &pos, portMAX_DELAY ) == pdTRUE) {
      if (pos == 0)
        angle = 0;
       else if (pos == 9)
        angle = 180;
       else
         angle = 18 * pos + 1;
      servo.write(angle);
    }
    
    vTaskDelay(250 / portTICK_PERIOD_MS); 
  }

}