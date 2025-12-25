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

 
void setMotorSpeed(Motor motor, Direction direction, uint8_t speed); //setter function for motor speed and direction

void setup() {
  // put your setup code here, to run once:
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  // pinMode(ENA, OUTPUT);
  // pinMode(ENB, OUTPUT);

  
}

void loop() {
  // put your main code here, to run repeatedly:
}

void setMotorSpeed(Motor motor, Direction direction, uint8_t speed){
  if(speed > 255) speed = 255;
  if(motor == MOTOR_A){
    if(direction==FORWARD){
      analogWrite(IN1,LOW);
      analogWrite(IN2, HIGH);
    }else if(direction==BACKWARD){
      analogWrite(IN1,HIGH);
      analogWrite(IN2, LOW);
    }
    analogWrite(ENA,speed);
    return;
  } else if(motro == MOTOR_B){
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

