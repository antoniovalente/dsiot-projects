/**
 * @file main.h
 * @author original Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @author adaptated Alexandre Breda Matos (Esc. Secundária S. Pedro - Vila Real, Portugal)
 * @author adaptated Miguel Azevedo (Esc. Secundária S. Pedro - Vila Real, Portugal)
 * @brief Includes, definitions and global declarations for RAK4631_DeepSleep_BME680_LoRaWan.ino
 * @version 0.1
 * @date 2020-08-01
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <Arduino.h>
#include <SPI.h>

#include <LoRaWan-RAK4630.h>

// Comment the next line if you want DEBUG output. But the power savings are not as good then!!!!!!!
#define MAX_SAVE

/* Time the device is sleeping in milliseconds = 15 minutes * 60 seconds * 1000 milliseconds */
#define SLEEP_TIME 15 * 60 * 1000

// LoRaWan stuff
int8_t initLoRaWan(void);
bool sendLoRaFrame(float Temp_bme680, float Hum_bme680, float Press_bme680, float Gas_bme680);
extern SemaphoreHandle_t loraEvent;

// Main loop stuff
void periodicWakeup(TimerHandle_t unused);
extern SemaphoreHandle_t taskEvent;
extern uint8_t rcvdLoRaData[];
extern uint8_t rcvdDataLen;
extern uint8_t eventType;
extern SoftwareTimer taskWakeupTimer;
