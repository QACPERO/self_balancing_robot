#include <Arduino.h>
#include <Wire.h> //essential for I2C communication
#include <MPU6050.h> //library for MPU6050 sensor
#include <I2Cdev.h> //I2C device library


//LN298N Motor Driver Pins
#define IN3 3 // Pin sterowania silnikiem B
#define IN4 4 // Pin sterowania silnikiem B
#define ENB 5 // Pin sterowania prędkością silnika B (PWM)
#define ENA 6 // Pin sterowania prędkością silnika A (PWM)
#define IN1 7 // Pin sterowania silnikiem A
#define IN2 8 // Pin sterowania silnikiem A

//MPU6050 Sensor Pins
#define INT 2 // Pin przerwania MPU6050
#define SCL A5 // Pin I2C SCL
#define SDA A4 // Pin I2C SDA

enum Direction {
  FORWARD,
  BACKWARD
};

enum Motor {
  MOTOR_A,
  MOTOR_B
};

struct IMUData {
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
};


//constants and variables

//complementrary filter variables
 //time interval between filter updates

struct ComplementaryFilter {
  float alpha;
  unsigned long lastUpdate;
  float dt; // 10 ms
  float filteredAngle;
  void initialize(float tau, float dtValue, float initialAngle){
    dt = dtValue;
    alpha = tau / (dt + tau); // calculate alpha based on time constant and dt
    lastUpdate = millis();
    filteredAngle = initialAngle;
  }

  void update(float newGyroAngle, float newAccAngle){ // angular speed from gyroscope and angle from accelerometer
    float currentTime = millis();
    float currentDt = (currentTime - lastUpdate) / 1000.0; // convert to seconds
    if(currentDt >= dt){
      // filtering

      float gyroAngle = newGyroAngle * currentDt;
      filteredAngle = alpha * (filteredAngle + gyroAngle) + (1 - alpha) * newAccAngle;

      lastUpdate = currentTime;
    }
    
  }
};


void setMotorSpeed(Motor motor, Direction direction, uint8_t speed); //setter function for motor speed and direction


// Create an MPU6050 object
MPU6050 mpu;
ComplementaryFilter compFilter;
IMUData imuData;

//variales
float tau = 0.5; //time constant for complementary filter /// 0.5 = mid /// 2.0 = smooth /// 0.1 = responsive
float dt = 0.01; //time interval for filter update in seconds
float initialAngle = 0.0; //initial angle for filter -> to be set after first reading



void setup() {
  // put your setup code here, to run once:
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  Wire.begin();
  Serial.begin(9600);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // use the code below to change accel/gyro offset values
  /*
  Serial.println("Updating internal sensor offsets...");
  // -76	-2359	1688	0	0	0
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");
  accelgyro.setXGyroOffset(220);
  accelgyro.setYGyroOffset(76);
  accelgyro.setZGyroOffset(-85);
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");
  */
   
  IMUData rawData;
  mpu.getMotion6(&rawData.accelX, &rawData.accelY, &rawData.accelZ, &rawData.gyroX, &rawData.gyroY, &rawData.gyroZ);

  float startAngle = atan2(-rawData.accelX, sqrt((long)rawData.accelY*rawData.accelY + (long)rawData.accelZ*rawData.accelZ)) * 57.296;
  
  compFilter.initialize(tau, dt, startAngle);
  Serial.print("Start Angle: "); Serial.println(startAngle);//initialize complementary filter and initial angle from accelerometer
  Serial.println("Setup complete.");
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu.getMotion6(&imuData.accelX, &imuData.accelY, &imuData.accelZ, &imuData.gyroX, &imuData.gyroY, &imuData.gyroZ);
  //convert gyroscope values to deg/s
  float gyroY = imuData.gyroY / 131.0; 

  float acc_angle = atan2(-imuData.accelX, sqrt((long)imuData.accelY*imuData.accelY + (long)imuData.accelZ*imuData.accelZ)) * 57.296;

  compFilter.update(gyroY, acc_angle); //update complementary filter with gyroscope Y axis and accelerometer angle
  Serial.print("Filtered Angle: ");
  Serial.println(compFilter.filteredAngle);

  //teleplot
  Serial.print("> Filtered Angle:"); Serial.print(compFilter.filteredAngle);
  Serial.print("> Gyro angle:"); Serial.print(gyroY);
  Serial.print("> Acc angle:"); Serial.println(acc_angle);
  

  //PID implementation
  delay(10); //loop delay
}



void setMotorSpeed(Motor motor, Direction direction, uint8_t speed){
  if(speed > 255) speed = 255;
  if(motor == MOTOR_A){ //
    if(direction==FORWARD){
      analogWrite(IN1,LOW);
      analogWrite(IN2, HIGH);
    }else if(direction==BACKWARD){
      analogWrite(IN1,HIGH);
      analogWrite(IN2, LOW);
    }
    analogWrite(ENA,speed);
    return;
  } else if(motor == MOTOR_B){
    if(direction==FORWARD){
      analogWrite(IN3,LOW);
      analogWrite(IN4, HIGH);
    }else if(direction==BACKWARD){
      analogWrite(IN3,HIGH);
      analogWrite(IN4, LOW);
    }
    analogWrite(ENB,speed);
    return;
  }
}

