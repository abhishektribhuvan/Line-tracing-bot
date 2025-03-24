# Line-tracing-bot
#include <QTRSensors.h>

// Motor Driver Pins
#define MOTOR_A1 25
#define MOTOR_A2 26
#define MOTOR_B1 27
#define MOTOR_B2 14

// QTR-8RC Sensor Pins
const uint8_t sensorPins[] = {32, 33, 34, 35, 36, 39, 4, 5};  // Adjust as per wiring
#define NUM_SENSORS 8

QTRSensorsRC qtr;
uint16_t sensorValues[NUM_SENSORS];

void setup() {
    Serial.begin(115200);

    
  pinMode(MOTOR_A1, OUTPUT);
    pinMode(MOTOR_A2, OUTPUT);
    pinMode(MOTOR_B1, OUTPUT);
    pinMode(MOTOR_B2, OUTPUT);

// Initialize QTR sensor
    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, NUM_SENSORS);
    qtr.setEmitterPin(2);  

  // Calibrate sensors
    for (int i = 0; i < 200; i++) {  
        qtr.calibrate();
        delay(10);
    }
}

void loop() {
    int position = qtr.readLine(sensorValues);
    int error = position - 3500;  

    
}

void moveForward() {
    analogWrite(MOTOR_A1, 100);
    analogWrite(MOTOR_A2, 0);
    analogWrite(MOTOR_B1, 100);
    analogWrite(MOTOR_B2, 0);
}

void turnLeft() {
    analogWrite(MOTOR_A1, 0);
    analogWrite(MOTOR_A2, 100);
    analogWrite(MOTOR_B1, 100);
    analogWrite(MOTOR_B2, 0);
}

void turnRight() {
    analogWrite(MOTOR_A1, 100);
    analogWrite(MOTOR_A2, 0);
    analogWrite(MOTOR_B1, 0);
    analogWrite(MOTOR_B2, 100);
}
