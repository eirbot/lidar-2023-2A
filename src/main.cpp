#include "mbed.h"
#include "tools.h"
#include "LidarSerial.h"
#include "usart.h"
#include "dma.h"

DigitalOut led(LED1);
BufferedSerial lidarUART(TX_LIDAR, RX_LIDAR, LIDAR_BAUD_RATE);
BufferedSerial pc(USBTX, USBRX);
PwmOut pwm(PWM);
DigitalOut avertissement(GPIO_Avertissement);
DigitalOut danger(GPIO_Danger);

//Lidar lidar(&lidarUART, &avertissement, &danger, &led, 300);

/*
 * Utiliser printf pour envoyer à l'ordinateur des données à 9600 bauds/s
 * Pour avoir un moniteur série facilement :
 * > mbed sterm
 */

Thread lidarThread(osPriorityAboveNormal, OS_STACK_SIZE);



int main() {
    pwm.period_us(25); // 40kHz
    pwm.write(1);
    danger = 0;
    led = 1;

    lidarThread.start(lidarMain);

    while (1) {

    }


}