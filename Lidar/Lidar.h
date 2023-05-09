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
    BufferedSerial* serial;

    int nb_point = 200;
    int nb_calibration = 3;

    int dist_min = 150;
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

    float seuil_danger = 700;
    float seuil_arret = 350;

public:
    Lidar(BufferedSerial *ptrSerial, int n);

    void write_int(const uint8_t value);
    uint8_t read_uint8();
    int read_uint8_n(uint8_t *value, int n);

    void set_threshold(int min, int max);
    void set_nb_point(int n);

    void end();
    int init();
    void reset();
    int info();
    void calibration();
    int scan_pol(float *data, float *angle);
    int scan_cart(float *X, float *Y);

    void tri(float *data, float *angle, float *data_tri, float *angle_tri);
    int point_part(float *data, float *angle, float *X_part, float *Y_part, float *angle_part);
    int balises(float *X_part, float *Y_part, float *angle_part);
    int position();

    void get_position(int *pos_x, int *pos_y);
};

#endif


#endif //LIDAR_H