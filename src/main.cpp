#include "mbed.h"
#include "constantes.h"
#include "Lidar.h"

DigitalOut led(LED1);
BufferedSerial lidarUART(TX_Lidar, RX_Lidar, LIDAR_BAUD_RATE);
PwmOut pwm(PWM);
DigitalOut avertissement(GPIO_Avertissement);
DigitalOut danger(GPIO_Danger);

Lidar lidar(&lidarUART, 200);

/*
 * Utiliser printf pour envoyer à l'ordinateur des données à 115200 bauds/s
 * Pour avoir un moniteur série facilement :
 * > mbed sterm -b 115200
 */

int main(){
    // Setup
    pwm.period_us(25); // 40kHz
    pwm.write(0.66);
    danger = 0;
    led = 0;

    lidar.init();

    // Loop
    while(1){
        led = 1;
        ThisThread::sleep_for(1s);
        led = 0;
        ThisThread::sleep_for(1s);
    }
}