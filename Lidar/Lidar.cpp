#include "Lidar.h"

Lidar::Lidar(BufferedSerial *ptrSerial, DigitalOut *d, DigitalOut *a, DigitalOut *l, int n) {
    serial = ptrSerial;
    nb_point = n;
    avertissement = a;
    danger = d;
    led = l;
}

void Lidar::write_int(const uint8_t value) {
    //while (!serial->writable());
    serial->write(&value, 1);
}

uint8_t Lidar::read_uint8() {
    // long unsigned int value;
    uint8_t value;
    //while (!serial->readable());
    serial->read(&value, 1);
    return value;
}

void Lidar::read_uint8_n(uint8_t *value, int n) {
    //while (!serial->readable());
    int rBytes = serial->read(value, n);
    //return rBytes;
}


int Lidar::init() {
    //printf("Begin of Lidar initialisation\n");

    const int n = 10;
    int fail = 0;
    uint8_t health[n];

    do {
        printf("O\n");

        write_int(0xA5);
        write_int(0x52);

        for (uint8_t i = 0; i < n; i++) {
            health[i] = read_uint8();
            //printf("%X \n", health[i]);
        }

        if (health[7] == 2) {
            fail++;
            if (fail == 10) {
                printf("Time out\n");
                return -1;
            }
            printf("Reset\n");
        }

    } while (health[7] == 2);

    printf("ok\n");

    return 0;
}

void Lidar::reset() {
    printf("Begin of Lidar RESET\n");

    write_int(0xA5);
    write_int(0x40);

    printf("End of Lidar RESET\n");
}


void Lidar::adverse() {
    uint8_t *scan = (uint8_t *) malloc(5 * sizeof(uint8_t));
    uint8_t descriptor[7];

    float data, angle;


    write_int(0xA5);
    write_int(0x20);

    do {
        descriptor[0] = read_uint8();
    } while (descriptor[0] != 0xA5);

    for (int i = 1; i < 7; i++) {
        descriptor[i] = read_uint8();
    }

    int nbr_danger = 0;
    int nbr_arret = 0;
    int nbr_point = 0;


    while (1) {
        data = ((scan[4] << 8) | scan[3]);
        angle = ((scan[2] << 7) | (scan[1] >> 1));
        read_uint8_n(scan, 5);

        if (angle > 16640 || angle < 1280 || (angle < 13440 && angle > 4480)) {
            if (data > 240 && data < 2000) {
                nbr_arret++;
            }
        } else {
            nbr_danger = 0;
            nbr_arret = 0;
            led->write(0);
            danger->write(0);
            avertissement->write(0);
        }

        // if(nbr_point >= 400)
        // {
        //     led->write(0);
        //     nbr_point = 0;
        // }
        // if((angle >= 1280 && angle <= 4480) || (angle >= 13440 && angle <= 16640))
        // {
        //     nbr_danger = 0;
        //     nbr_arret = 0;
        //     led->write(0);
        //     danger->write(0);

        //     //nbr_point++;
        // }
        if (nbr_arret >= nbr_seuil_alerte) {
            led->write(1);
            danger->write(1);
            avertissement->write(1);

        }
    }

    free(scan);

}
