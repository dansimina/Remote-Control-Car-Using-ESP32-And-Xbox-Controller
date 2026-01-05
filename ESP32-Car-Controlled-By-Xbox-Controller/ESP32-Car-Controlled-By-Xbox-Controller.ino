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
const int SERVO_CENTER = 92;
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
const int RESOLUTION = 8;

// HC-SR04 Ultrasonic Sensor
const int TRIG_PIN = 12;
const int ECHO_PIN = 14;
const double SOUND_SPEED = 0.034;

enum STATE_MACHINE {START, TRIGGER, MEASUREMENT, WAITING};
enum STATE_MACHINE state;

volatile unsigned long echoStartTime = 0;
volatile unsigned long echoEndTime = 0;
volatile bool echoComplete = false;
unsigned long lastMeasurementTime = 0;
float distanceCm = 0.0;
float distance = 0.0; // Mapped value 0-100 for distances 3-100cm
const unsigned long MEASUREMENT_INTERVAL = 100; // milliseconds between measurements
const float MIN_DISTANCE = 3.0;  // Minimum valid distance in cm
const float MAX_DISTANCE = 100.0; // Maximum valid distance in cm

const int MAX_SPEED_WHEN_OBSTACLE = 50;
const int WARNING_DISTANCE = 30;
const int MAX_SPEED = 230;

// Interrupt handler for echo pin
void IRAM_ATTR echoISR() {
  if (digitalRead(ECHO_PIN) == HIGH) {
    // Echo pulse started
    echoStartTime = micros();
  } else {
    // Echo pulse ended
    echoEndTime = micros();
    echoComplete = true;
  }
}

// Forward declarations
void motorAControl(int speed, bool forward);
void motorBControl(int speed, bool forward);
void stopMotors();

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

// Core 0: Ultrasonic sensor task (runs independently)
void ultrasonicTask(void* parameter) {
  while (true) {
    unsigned long currentTime = millis();
    
    switch(state) {
      case START:
        // Initialize measurement
        digitalWrite(TRIG_PIN, LOW);
        echoComplete = false;
        lastMeasurementTime = currentTime;
        state = TRIGGER;
        break;
        
      case TRIGGER:
        // Wait 2ms for sensor to settle, then send trigger pulse
        if (currentTime - lastMeasurementTime >= 2) {
          digitalWrite(TRIG_PIN, HIGH);
          delayMicroseconds(10);
          digitalWrite(TRIG_PIN, LOW);
          lastMeasurementTime = currentTime;
          state = MEASUREMENT;
        }
        break;
        
      case MEASUREMENT:
        // Check if echo measurement is complete
        if (echoComplete) {
          unsigned long duration = echoEndTime - echoStartTime;
          distanceCm = duration * SOUND_SPEED / 2;
          
          if (distanceCm < MIN_DISTANCE) {
            distance = MIN_DISTANCE;
          } else if (distanceCm > MAX_DISTANCE) {
            distance = MAX_DISTANCE;
          } else {
            distance = distanceCm;
          }
          
          state = WAITING;
          lastMeasurementTime = currentTime;
        } else if (currentTime - lastMeasurementTime > 40) {
          // Timeout after 40ms
          distance = 100.0;
          state = WAITING;
          lastMeasurementTime = currentTime;
        }
        break;
        
      case WAITING:
        // Wait before next measurement
        if (currentTime - lastMeasurementTime >= MEASUREMENT_INTERVAL) {
          state = START;
        }
        break;
    }
    
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// Core 0: Control motors and servo (fast response)
void controlMotorsAndServo(void* parameter) {
  static int currentSpeedLeft = 0;  // Track left motor speed
  static int currentSpeedRight = 0; // Track right motor speed
  static bool wasMoving = false;
  
  while (true) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      if (isConnected) {
        Serial.printf("lx: %.2f, ly: %.2f, rx: %.2f, ry: %.2f, lt: %.2f, rt: %.2f | Dist: %.1f cm\n",
                      currentData.leftStickX, currentData.leftStickY,
                      currentData.rightStickX, currentData.rightStickY,
                      currentData.leftTrigger, currentData.rightTrigger,
                      distance);

        // Servo control
        int angle = map(currentData.leftStickX * 100, -100, 100, MIN_ANGLE, MAX_ANGLE) + SERVO_CENTER;
        myServo.write(angle);

        // Calculate base speed
        int speedForward = map(currentData.rightTrigger * 100, 0, 100, 0, MAX_SPEED);
        int speedBackward = map(currentData.leftTrigger * 100, 0, 100, 0, MAX_SPEED);
        int speed = speedForward - speedBackward;
        int absSpeed = abs(speed);
        bool isForward = (speed > 0);
        
        // Calculate steering factor (-1.0 to 1.0)
        // Negative = left, Positive = right
        float steerFactor = currentData.leftStickX;
        
        // Determine target speeds with obstacle detection
        int targetSpeedBase = absSpeed;
        
        if (absSpeed > 10) {
          // Check for obstacles when moving forward - STOP COMPLETELY
          if(isForward && distance < WARNING_DISTANCE) {
            targetSpeedBase = 0;
          }
          
          // Calculate differential speeds based on steering
          // IMPORTANT: Do this AFTER obstacle check
          int targetSpeedLeft, targetSpeedRight;
          
          if (targetSpeedBase == 0) {
            // Obstacle detected - STOP both motors completely
            targetSpeedLeft = 0;
            targetSpeedRight = 0;
          }
          else if (steerFactor < -0.1) {
            // Turn LEFT - reduce LEFT motor speed (Motor A)
            float reduction = abs(steerFactor); // 0.1 to 1.0
            targetSpeedLeft = targetSpeedBase * (1.0 - reduction * 0.3);
            targetSpeedRight = targetSpeedBase;
          } 
          else if (steerFactor > 0.1) {
            // Turn RIGHT - reduce RIGHT motor speed (Motor B)
            float reduction = abs(steerFactor); // 0.1 to 1.0
            targetSpeedLeft = targetSpeedBase;
            targetSpeedRight = targetSpeedBase * (1.0 - reduction * 0.3);
          }
          else {
            // Go straight - both motors at same speed
            targetSpeedLeft = targetSpeedBase;
            targetSpeedRight = targetSpeedBase;
          }
          
          // Soft start: gradually ramp up speed
          const int RAMP_STEP = 10;
          
          // Left motor ramping
          if (currentSpeedLeft < targetSpeedLeft) {
            currentSpeedLeft = min(currentSpeedLeft + RAMP_STEP, targetSpeedLeft);
          } else if (currentSpeedLeft > targetSpeedLeft) {
            currentSpeedLeft = max(currentSpeedLeft - RAMP_STEP, targetSpeedLeft);
          }
          
          // Right motor ramping
          if (currentSpeedRight < targetSpeedRight) {
            currentSpeedRight = min(currentSpeedRight + RAMP_STEP, targetSpeedRight);
          } else if (currentSpeedRight > targetSpeedRight) {
            currentSpeedRight = max(currentSpeedRight - RAMP_STEP, targetSpeedRight);
          }
          
          // Apply speed to motors
          motorAControl(currentSpeedLeft, isForward);  // LEFT motor
          motorBControl(currentSpeedRight, isForward); // RIGHT motor
          
          wasMoving = true;
        } else {
          // Stop motors when trigger is released
          if (wasMoving) {
            // Gradual deceleration
            if (currentSpeedLeft > 0 || currentSpeedRight > 0) {
              currentSpeedLeft = max(0, currentSpeedLeft - 30);
              currentSpeedRight = max(0, currentSpeedRight - 30);
              motorAControl(currentSpeedLeft, isForward);
              motorBControl(currentSpeedRight, isForward);
            } else {
              stopMotors();
              wasMoving = false;
            }
          } else {
            stopMotors();
            currentSpeedLeft = 0;
            currentSpeedRight = 0;
          }
        }

      } else {
        Serial.println("Controller not connected");
        myServo.write(SERVO_CENTER);
        stopMotors();
        currentSpeedLeft = 0;
        currentSpeedRight = 0;
        wasMoving = false;
      }
      xSemaphoreGive(dataMutex);
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
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
  ledcWrite(ENA_PIN, 0);
  ledcWrite(ENB_PIN, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Attach interrupt to echo pin (triggers on both RISING and FALLING edges)
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE);

  state = START;
  
  Serial.println("HC-SR04 Ultrasonic Sensor Ready (Interrupt-based)");

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
  BaseType_t task1 = xTaskCreatePinnedToCore(readControllerTask, "ReadController", 8192, NULL, 2, NULL, 1);

  // Core 0: Control motors and servo (highest priority for responsiveness)
  BaseType_t task2 = xTaskCreatePinnedToCore(controlMotorsAndServo, "MotorControl", 4096, NULL, 2, NULL, 0);
  
  // Core 0: Ultrasonic sensor (lower priority)
  BaseType_t task3 = xTaskCreatePinnedToCore(ultrasonicTask, "Ultrasonic", 2048, NULL, 1, NULL, 0);

  if (task1 != pdPASS || task2 != pdPASS || task3 != pdPASS) {
    Serial.println("ERROR: Failed to create tasks!");
    while(1); // Halt
  }

  Serial.println("Started!");
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}