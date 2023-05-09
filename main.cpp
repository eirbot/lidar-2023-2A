#include "mbed.h"
#include "constantes.h"

DigitalOut led(LED1);
BufferedSerial lidarUART(TX_Lidar, RX_Lidar);
PwmOut pwm(PWM);
DigitalOut avertissement(GPIO_Avertissement);
DigitalOut danger(GPIO_Danger);

/*
 * Utiliser printf pour envoyer à l'ordinateur des données à 115200 bauds/s
 * Pour avoir un moniteur série facilement :
 * > mbed sterm -b 115200
 */

int main(){
    // Setup
    // Loop
    while(1){
        led = 1;
        ThisThread::sleep_for(1s);
        led = 0;
        ThisThread::sleep_for(1s);
    }
}