#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#include <DHT.h>
#include <Wire.h>  // libreria I2C

#define MPU 0x68

// Conversion ratios
#define A_R 16384.0
#define G_R 131.0

// DHT22
// Define DHT type and DHT pin 
#define DHTTYPE DHT22
#define DHTPIN 7

// define tasks
void TaskReadFromSerial( void *pvParameters ); // Get commands
void TaskSensors( void *pvParameters );

//define smaphore handlers
SemaphoreHandle_t sensorSem;

//define queue
QueueHandle_t sensorQueue;

//define DHT sensor
DHT dhtSensor(DHTPIN, DHTTYPE);

// Define stucts
struct message 
{
  int ldr;  // LDR
  double humidity; // DHT
  double temperature; // DHT
  // IMU
  float accx;
  float accy;
  float gx;
  float gy;
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

  // IMU
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  sensorSem = xSemaphoreCreateBinary();

  sensorQueue = xQueueCreate( 5, sizeof( message ) );

  // Now set up tasks to run independently.
  
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
    ,  128  // Stack size
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

  for (;;)
  {   
    if(Serial.available()>0){
      
      char c = Serial.read();

      if (c == '1') {

        xSemaphoreGive(sensorSem);
    
        struct message current;
        xQueueReceive( sensorQueue, &current, portMAX_DELAY ); // Read value from other tasks

        uint8_t checksum = 0; // Equals to byte
    
        Serial.print('[');
        Serial.print('O');
        uint8_t ldr = (uint8_t) current.ldr; // Equals to byte
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
        uint8_t accx = (uint8_t) current.accx*100; // Equals to byte
        checksum = checksum + accx;
        Serial.print(accx);
        Serial.print(',');
        uint8_t accy = (uint8_t) current.accy*100; // Equals to byte
        checksum = checksum + accy;
        Serial.print(accy);
        Serial.print(',');
        uint8_t gx = (uint8_t) current.gx*100; // Equals to byte
        checksum = checksum + gx;
        Serial.print(gx);
        Serial.print(',');
        uint8_t gy = (uint8_t) current.gy*100; // Equals to byte
        checksum = checksum + gy;
        Serial.print(gy);
        Serial.print(',');
        Serial.print(checksum);
        Serial.println(']');
      
      }
      
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

    //IMU
    // MPU-6050 RAW values (unproccessed) from the accelerometer and the gyroscope in X, Y and Z axis
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    
    float Acc[2];
    float Gy[2];
    
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Ask for 0x3B, to get ax
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,6,true); // From 0x3B, request 6 registries
  
    ax=Wire.read()<<8|Wire.read();
    ay=Wire.read()<<8|Wire.read();
    az=Wire.read()<<8|Wire.read();
    
    // From raw values, let's calculate Axis X and Y using atan formula
    Acc[1] = atan(-1*(ax/A_R)/sqrt(pow((ay/A_R),2) + pow((az/A_R),2)))*RAD_TO_DEG;
    Acc[0] = atan((ay/A_R)/sqrt(pow((ax/A_R),2) + pow((az/A_R),2)))*RAD_TO_DEG;
  
    // Gyroscope
    Wire.beginTransmission(MPU);
    Wire.write(0x43); // Ask for 0x43, to get gx
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,4,true); // From 0x43, request 4 registries
    gx=Wire.read()<<8|Wire.read();
    gy=Wire.read()<<8|Wire.read();
    // From raw values, let's calculate angles
    Gy[0] = gx/G_R;
    Gy[1] = gy/G_R;

    current.accx = Acc[0];
    current.accy = Acc[1];
    current.gx = Gy[0];
    current.gy = Gy[1];
    
    xQueueSend(sensorQueue, &current, portMAX_DELAY);  // Send value to other tasks
    
    vTaskDelay(250 / portTICK_PERIOD_MS); 
  }
  
}