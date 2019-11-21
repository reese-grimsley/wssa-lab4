#include <SPI.h>
#include <FreeRTOS_ARM.h>

#include "FXOS8700CQ.h"

#define INT_PIN 51

FXOS8700CQ sensor;

QueueHandle_t inputQueue;
SemaphoreHandle_t sem;  // Declare a semaphore handle.
TaskHandle_t handleCollectData, handleProcessData;

uint32_t dataCount, countX, countY, countZ;

static void ThreadCollectData(void* arg) {
  SerialUSB.println("start collect");
  while (1) {
    xSemaphoreTake(sem, portMAX_DELAY);
    SerialUSB.println("collectData()");
    sensor.resetInterrupt();
    sensor.readMagData();
    sensor.printMagData();

    vTaskResume(handleProcessData);
  }
}

static void ThreadProcessData(void* arg) {
  while (1) {
    SerialUSB.print("processData() = ");
    SerialUSB.println(++dataCount);
    vTaskSuspend(handleProcessData);

  }


}

//interrupt service routine
void Isr() {
  //do something with the queue (using ISR version)
  SerialUSB.println("Start ISR");

  //necessary to have this
  static BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(sem, &xHigherPriorityTaskWoken);
}

void blink(int ms) {
  digitalWrite(6, HIGH);
  delay(ms >> 1);
  digitalWrite(6, LOW);
  delay(ms >> 1);
}

void setup() {
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);

  // Initialize SerialUSB
  SerialUSB.begin(9600);
  while (!SerialUSB) blink(100);
  SerialUSB.println("Serial configured; setup SPI");

  // Initialize SPI
  SPI.begin();

  // Initialize sensor
  sensor = FXOS8700CQ();
  sensor.init();
  delay(10); //delay to initialize fully

  // check WhoAmI
  sensor.checkWhoAmI();
  SerialUSB.println("Calibrate Magnetometer");
  sensor.calibrateMag();
  SerialUSB.println("Setup interrupt for magnitude of magnetometer readings.");
  sensor.enableMagInterrupt();


  SerialUSB.println("Setup RTOS tasks");
  portBASE_TYPE taskCollectData, taskDataProcess;

  sem = xSemaphoreCreateCounting(1, 1);
//  xSemaphoreGive(sem);
  if (sem == NULL) {
    SerialUSB.println("semaphore failed creation");
    while (1);
  }

  SerialUSB.println("Semaphore setup, create tasks..");
  taskCollectData = xTaskCreate( ThreadCollectData, NULL, configMINIMAL_STACK_SIZE, NULL, 1, &handleCollectData);
  if (taskCollectData != pdPASS) {
    SerialUSB.println("Creation task failed!");
    while (1);
  }


  taskDataProcess = xTaskCreate( ThreadProcessData, NULL, configMINIMAL_STACK_SIZE, NULL, 2, &handleProcessData);
  if (taskDataProcess != pdPASS) {
    SerialUSB.println("Creation task failed!");
    while (1);
  }


//  inputQueue = xQueueCreate((UBaseType_t) 128, (UBaseType_t) sizeof(char));
//  if (inputQueue == NULL) {
//    SerialUSB.println("Queue not created...");
//    while (1);
//  }


  //Initialize Interrupt pin and mode
  SerialUSB.println("Setup Interrupt pin, mode, ISR");
  pinMode(INT_PIN, INPUT);
  attachInterrupt(INT_PIN, Isr, FALLING);
  xSemaphoreTake(sem, portMAX_DELAY);

  SerialUSB.println("Start FreeRTOS scheduler");
//  vTaskSuspend(handleCollectData);
  vTaskSuspend(handleProcessData);
  vTaskStartScheduler();



}

void loop() {

}
