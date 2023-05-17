#ifndef CONSTANTES_H
#define CONSTANTES_H

#include "mbed.h"

#define TX_LIDAR            PA_9
#define RX_LIDAR            PA_10
#define LIDAR_BAUD_RATE     115200

#define PWM                 PA_8
#define GPIO_Avertissement  PB_4
#define GPIO_Danger         PB_5

//extern  BufferedSerial pc;
void Error_Handler();

#endif // CONSTANTES_H