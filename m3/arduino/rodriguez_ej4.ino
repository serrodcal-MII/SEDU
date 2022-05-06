#include <DHT.h>
#include <Wire.h>  // libreria I2C

// LDR
const int inputLDR = A0;
int ldrValue = 0;

// Define the I2C IMU's address
#define MPU 0x68

// Conversion ratios
#define A_R 16384.0
#define G_R 131.0

// MPU-6050 RAW values (unproccessed) from the accelerometer and the gyroscope in X, Y and Z axis
int16_t ax, ay, az;
int16_t gx, gy, gz;
float Acc[2];
float Gy[2];

// DHT22
// Define DHT type and DHT pin 
#define DHTTYPE DHT22
#define DHTPIN 7

DHT dhtSensor(DHTPIN, DHTTYPE);

float humidity = 0.0;
float temperature = 0.0;

void setup() {
  // LDR
  pinMode(inputLDR, INPUT);
  Serial.begin(9600);
  
  // IMU
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  // DHT22
  dhtSensor.begin();
}

void loop() {

  if(Serial.available() > 0) {
    String input = Serial.readString( );
    if (input.equals("1\n")) {
      ldr();
  
      imu();
    
      dht();
    
      monitoring();
    }
  }

  delay(1000);
}

void monitoring() {

  Serial.println("___________");
  Serial.print("LDR: "); 
  Serial.println(ldrValue);

  Serial.println("IMU: ");
  Serial.print("  ax: ");
  Serial.print(ax);
  Serial.print(", ay: ");
  Serial.print(ay);
  Serial.print(", az: ");
  Serial.println(az);
  Serial.print("  gx: ");
  Serial.print(gx);
  Serial.print(", gy: ");
  Serial.print(gy);
  Serial.print(", gz: ");
  Serial.println(gz);
  Serial.print("  Acc: ");
  Serial.print(Acc[0]);
  Serial.print(", ");
  Serial.println(Acc[1]);
  Serial.print("  Gy: ");
  Serial.print(Gy[0]);
  Serial.print(", ");
  Serial.println(Gy[1]);
  
  Serial.println("DHT22: ");
  Serial.print("humidity: ");
  Serial.print(humidity);
  Serial.print(" %");
  Serial.print(", temperature: ");
  Serial.print(temperature);
  Serial.println(" ºC");
  
  Serial.println("");
}

void ldr() {
  ldrValue = analogRead(inputLDR);
}

void imu() {
  
  // Accelerometer
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
}

void dht() {
  humidity = dhtSensor.readHumidity();
  temperature = dhtSensor.readTemperature();
}