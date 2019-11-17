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
  while (1) {
    sensor.readMagData();

    vTaskResume(handleProcessData);
    vTaskSuspend(handleCollectData);
  }
}

static void ThreadProcessData(void* arg) {
  while (1) {
    dataCount++;
    vTaskSuspend(handleProcessData);

  }


}

//interrupt service routine
void Isr() {
  //do something with the queue (using ISR version)
  SerialUSB.println("Start ISR");
  
  //  vTaskResume(handleCollectData);
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

  //Initialize Interrupt pin and mode
  SerialUSB.println("Setup Interrupt pin, mode, ISR");
  pinMode(INT_PIN, INPUT);
  attachInterrupt(INT_PIN, Isr, FALLING);

  // check WhoAmI
  sensor.checkWhoAmI();
  SerialUSB.println("Calibrate Magnetometer");
  sensor.calibrateMag();
  SerialUSB.println("Setup interrupt for magnitude of magnetometer readings.");
  sensor.enableMagInterrupt();


  SerialUSB.println("Setup RTOS tasks");
  portBASE_TYPE taskCollectData, taskDataProcess;

//  sem = xSemaphoreCreateCounting(1, 1);
//  if (sem == NULL) {
//    SerialUSB.println("semaphore failed creation");
//    while (1);
//  }

//  taskCollectData = xTaskCreate( ThreadCollectData, NULL, configMINIMAL_STACK_SIZE, NULL, 1, &handleCollectData);
//  if (taskCollectData != pdPASS) {
//    SerialUSB.println("Creation task failed!");
//    while (1);
//  }
//
//
//  taskDataProcess = xTaskCreate( ThreadProcessData, NULL, configMINIMAL_STACK_SIZE, NULL, 1, &handleProcessData);
//  if (taskDataProcess != pdPASS) {
//    SerialUSB.println("Creation task failed!");
//    while (1);
//  }


//  inputQueue = xQueueCreate((UBaseType_t) 128, (UBaseType_t) sizeof(char));
//  if (inputQueue == NULL) {
//    SerialUSB.println("Queue not created...");
//    while (1);
//  }

//  vTaskStartScheduler();


}

void loop() {
//  Fill code here to read from Magnetometer
    sensor.readMagData();
  
    SerialUSB.print("X value = ");
    SerialUSB.println(sensor.magData.x);
  
    SerialUSB.print("Y value = ");
    SerialUSB.println(sensor.magData.y);
  
    SerialUSB.print("Z value = ");
    SerialUSB.println(sensor.magData.z);
  
    SerialUSB.println("");
  
    delay(200);

}
