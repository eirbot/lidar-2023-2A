#ifndef LIDAR_H
#define LIDAR_H

#ifndef LIDAR_H_
#define LIDAR_H_

#include "mbed.h"
#include <string>
#include <cmath>

using namespace std;

class Lidar {
private:
    int fileID = -1;
    BufferedSerial *serial;

    int nb_point = 200;
    int nb_calibration = 3;

    int dist_min = 50; //150;
    int dist_max = 3800;

    float dist_seuil = 100.;
    float taille_rep_min = 60.;
    float taille_rep_max = 180.;
    int nbr_seuil = 4;
    int nbr_seuil_max = 200;
    float seuil_saut = 100.;
    float seuil_balise = 100.;

    float d_min_1 = 2300;
    float d_max_1 = 2700;
    float d_min_2 = 2600;
    float d_max_2 = 3200;

    float x1 = 0;
    float y1 = 0;
    float x2 = 0;
    float y2 = 3000;

    int nb_part = 0;

    float balises_X[3];
    float balises_Y[3];
    float angle_balises[3];

    float x;
    float y;

    bool affichage = false;

    int nbr_seuil_alerte = 7;
    float seuil_danger = 500;
    float seuil_arret = 350;

    DigitalOut *danger;
    DigitalOut *avertissement;
    DigitalOut *led;

public:
    Lidar(BufferedSerial *ptrSerial, DigitalOut *d, DigitalOut *a, DigitalOut *l, int n);

    void write_int(const uint8_t value);

    uint8_t read_uint8();

    void read_uint8_n(uint8_t *value, int n);

    void reset();

    int init();

    void adverse();

};

#endif


#endif //LIDAR_H