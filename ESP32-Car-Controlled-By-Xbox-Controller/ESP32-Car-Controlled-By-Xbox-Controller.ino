#include <Arduino.h>
#include <BLEGamepadClient.h>
#include <ESP32Servo.h>

//Controller
XboxController controller;
XboxControlsState currentData;
bool isConnected = false;
SemaphoreHandle_t dataMutex;

//Servo
const int SERVO_PIN = 13;
Servo myServo;

const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;
const int SERVO_CENTER = 90;
const int MIN_ANGLE = -30;
const int MAX_ANGLE = 38;

//H-Bridge
const int ENA_PIN = 19;
const int IN1 = 18;
const int IN2 = 5;
const int IN3 = 17;
const int IN4 = 16;
const int ENB_PIN = 4;

const int FREQ = 5000;
const int PWM_CHANNEL_A = 0;
const int PWM_CHANNEL_B = 1;
const int RESOLUTION = 8;

// Core 1: Read controller data
void readControllerTask(void* parameter) {
  while (true) {
    if (controller.isConnected()) {
      XboxControlsState state;
      controller.read(&state);

      if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        currentData = state;
        isConnected = true;
        xSemaphoreGive(dataMutex);
      }
    } else {
      if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        isConnected = false;
        xSemaphoreGive(dataMutex);
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// Core 0: Print to serial and control servo
void motorAControl(int speed, bool forward);
void motorBControl(int speed, bool forward);
void stopMotors();

void controlMotorsAndServo(void* parameter) {
  while (true) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      if (isConnected) {
        Serial.printf("lx: %.2f, ly: %.2f, rx: %.2f, ry: %.2f, lt: %.2f, rt: %.2f\n",
                      currentData.leftStickX, currentData.leftStickY,
                      currentData.rightStickX, currentData.rightStickY,
                      currentData.leftTrigger, currentData.rightTrigger);

        int angle = map(currentData.leftStickX * 100, -100, 100, MIN_ANGLE, MAX_ANGLE) + SERVO_CENTER;
        myServo.write(angle);

        int speedForward = map(currentData.rightTrigger * 100, 0, 100, 0, 255);
        int speedBackward = map(currentData.leftTrigger * 100, 0, 100, 0, 255);
        int speed = speedForward - speedBackward;
        int absSpeed = abs(speed);
        if (absSpeed > 10) {
          if(speed > 0) {
            motorAControl(absSpeed, true);
            motorBControl(absSpeed, true);
          }
          else {
            motorAControl(absSpeed, false);
            motorBControl(absSpeed, false);
          } 
        } else {
          // Stop motors when trigger is released
          stopMotors();
        }

      } else {
        Serial.println("Controller not connected");
        myServo.write(SERVO_CENTER);
        // Stop motors when disconnected
        motorAControl(0, true);
        motorBControl(0, true);
      }
      xSemaphoreGive(dataMutex);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Control Motor A
// speed: 0-255, forward: true for forward, false for backward
void motorAControl(int speed, bool forward) {
  ledcWrite(ENA_PIN, speed);

  if (forward) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
}

// Control Motor B
// speed: 0-255, forward: true for forward, false for backward
void motorBControl(int speed, bool forward) {
  ledcWrite(ENB_PIN, speed);

  if (forward) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

// Stop both motors
void stopMotors() {
  motorAControl(0, true);
  motorBControl(0, true);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize servo first
  myServo.attach(SERVO_PIN);
  myServo.write(SERVO_CENTER);  // Center position (90°)

  Serial.println("Servo initialized at center position (90°)");
  delay(500);  // Give servo time to reach center

  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Setup PWM for ENA and ENB pins
  ledcAttach(ENA_PIN, FREQ, RESOLUTION);
  ledcAttach(ENB_PIN, FREQ, RESOLUTION);

  Serial.println("L298N Motor Driver Ready");
  delay(500);

  controller.begin();
  dataMutex = xSemaphoreCreateMutex();

  // Core 1: Read controller
  xTaskCreatePinnedToCore(readControllerTask, "ReadController", 8192, NULL, 2, NULL, 1);

  // Core 0: Print data and control servo
  xTaskCreatePinnedToCore(controlMotorsAndServo, "PrintData", 4096, NULL, 1, NULL, 0);

  Serial.println("Started!");
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}