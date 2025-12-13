#include <Arduino.h>
#include <BLEGamepadClient.h>

XboxController controller;
XboxControlsEvent currentData;
bool isConnected = false;
SemaphoreHandle_t dataMutex;

// Core 1: Read controller data
void readControllerTask(void* parameter) {
  while (true) {
    if (controller.isConnected()) {
      XboxControlsEvent e;
      controller.read(&e);

      if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        currentData = e;
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

// Core 0: Print to serial
void printDataTask(void* parameter) {
  while (true) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      if (isConnected) {
        Serial.printf("lx: %.2f, ly: %.2f, rx: %.2f, ry: %.2f, lt: %.2f, rt: %.2f\n",
                      currentData.leftStickX, currentData.leftStickY,
                      currentData.rightStickX, currentData.rightStickY,
                      currentData.leftTrigger, currentData.rightTrigger);
      } else {
        Serial.println("Controller not connected");
      }
      xSemaphoreGive(dataMutex);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  controller.begin();
  dataMutex = xSemaphoreCreateMutex();

  // Core 1: Read controller
  xTaskCreatePinnedToCore(readControllerTask, "ReadController", 8192, NULL, 2, NULL, 1);

  // Core 0: Print data
  xTaskCreatePinnedToCore(printDataTask, "PrintData", 4096, NULL, 1, NULL, 0);

  Serial.println("Started!");
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}