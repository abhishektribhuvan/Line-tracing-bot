# Line-tracing-bot
include features like auto calibration,
sharp turn, and dash line 
#include <QTRSensors.h>

// Define Motor Driver Pins
#define MOTOR_A1 16  // Motor A (Forward)
#define MOTOR_A2 17  // Motor A (Backward)
#define MOTOR_B1 18  // Motor B (Forward)
#define MOTOR_B2 19  // Motor B (Backward)

// Define PWM Channels
#define PWM_CHANNEL_A  0
#define PWM_CHANNEL_B  1
#define PWM_FREQ       5000  // Frequency for PWM
#define PWM_RESOLUTION 8      // 8-bit resolution (0-255)

// QTR Sensor Setup
QTRSensorsRC qtr;
const uint8_t sensorCount = 8;
uint16_t sensorValues[sensorCount];

// PID Constants
float Kp = 0.2, Ki = 0.0005, Kd = 1.5;  
int lastError = 0;
int baseSpeed = 150;  // Base speed (adjustable)

// Function to control motor speed using ledcWrite
void setMotorSpeed(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    if (leftSpeed >= 0) {
        digitalWrite(MOTOR_A1, HIGH);
        digitalWrite(MOTOR_A2, LOW);
        ledcWrite(PWM_CHANNEL_A, leftSpeed);
    } else {
        digitalWrite(MOTOR_A1, LOW);
        digitalWrite(MOTOR_A2, HIGH);
        ledcWrite(PWM_CHANNEL_A, -leftSpeed);
    }

    if (rightSpeed >= 0) {
        digitalWrite(MOTOR_B1, HIGH);
        digitalWrite(MOTOR_B2, LOW);
        ledcWrite(PWM_CHANNEL_B, rightSpeed);
    } else {
        digitalWrite(MOTOR_B1, LOW);
        digitalWrite(MOTOR_B2, HIGH);
        ledcWrite(PWM_CHANNEL_B, -rightSpeed);
    }
}

// Sensor Calibration
void calibrateSensors() {
    Serial.println("Calibrating Sensors...");
    for (int i = 0; i < 100; i++) {
        qtr.calibrate();
        setMotorSpeed(80, -80);  // Small Rotation for Calibration
        delay(20);
    }
    setMotorSpeed(0, 0);
    Serial.println("Calibration Complete!");
}

// Handling Sharp Turns
void handleSharpTurn(int position) {
    if (position < 100) {
        setMotorSpeed(-180, 180);  // Left Sharp Turn
    } else if (position > 7000) {
        setMotorSpeed(180, -180);  // Right Sharp Turn
    }
}

// Handling Dashed Line
void handleDashLine() {
    setMotorSpeed(baseSpeed, baseSpeed);
    delay(300);
}

// PID Line Following
void lineFollow() {
    int position = qtr.readLine(sensorValues);
    int error = position - 3500;
    int speedAdjustment = Kp * error + Kd * (error - lastError);

    int leftSpeed = baseSpeed - speedAdjustment;
    int rightSpeed = baseSpeed + speedAdjustment;
    setMotorSpeed(leftSpeed, rightSpeed);

    lastError = error;
}

// Setup Function
void setup() {
    Serial.begin(115200);
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){34, 35, 32, 33, 25, 26, 27, 14}, sensorCount);

    pinMode(MOTOR_A1, OUTPUT);
    pinMode(MOTOR_A2, OUTPUT);
    pinMode(MOTOR_B1, OUTPUT);
    pinMode(MOTOR_B2, OUTPUT);

    // Setup PWM Channels
    ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
    
    ledcAttachPin(MOTOR_A1, PWM_CHANNEL_A);
    ledcAttachPin(MOTOR_B1, PWM_CHANNEL_B);

    calibrateSensors();
}

// Main Loop
void loop() {
    int position = qtr.readLine(sensorValues);
    if (position < 100 || position > 7000) {
        handleSharpTurn(position);
    } else if (position == 3500) {
        handleDashLine();
    } else {
        lineFollow();
    }
}