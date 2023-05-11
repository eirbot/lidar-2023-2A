 #include "Lidar.h"

 Lidar::Lidar(BufferedSerial *ptrSerial, int n) {
     serial = ptrSerial;
     nb_point = n;
 }

 void Lidar::write_int(const uint8_t value) {
      while (!serial->writable());
     serial->write(&value, 1);
 }

 uint8_t Lidar::read_uint8() {
     // long unsigned int value;
     uint8_t value;
      while (!serial->readable());
     serial->read(&value, 1);
     return value;
 }

 int Lidar::read_uint8_n(uint8_t *value, int n) {
      while (!serial->readable());
     int rBytes = serial->read(value, n);
     return rBytes;
 }


 void Lidar::set_threshold(int min, int max) {
     dist_min = min;
     dist_max = max;
 }


 void Lidar::end() {
     write_int(0xA5);
     write_int(0x40);
 }

 int Lidar::init() {
     printf("Begin of Lidar initialisation\n");

     const int n = 10;
     int fail = 0;
     uint8_t health[n];

     do {
         printf("GET HEALTH\n");

         write_int(0xA5);
         write_int(0x52);

         for (uint8_t i = 0; i < n; i++) {
             health[i] = read_uint8();
             printf("%X \n", health[i]);
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

     printf("End of Lidar initialisation\n");

     return 0;
 }

 void Lidar::reset() {
     printf("Begin of Lidar RESET\n");

     write_int(0xA5);
     write_int(0x40);

     printf("End of Lidar RESET\n");
 }

 void Lidar::calibration() {
     printf("Begin of calibration\n");
     affichage = false;

     for (int i = 0; i < nb_calibration; i++) {
         position();
         printf("%f %% \n", (i * 1. + 1.) / (nb_calibration * 1.) * 100);
     }

     affichage = true;
     printf("End of calibration\n");
 }

 int Lidar::scan_pol(float *data, float *angle) {
     uint8_t *scan = (uint8_t *) malloc(5 * sizeof(uint8_t));
     uint8_t descriptor[7];

     if (affichage) {
         printf("Begin of Lidar SCAN\n");
     }

     write_int(0xA5);
     write_int(0x20);

     if (affichage) {
         printf("Wait for a descriptor\n");
     }

     do {
         descriptor[0] = read_uint8();
     } while (descriptor[0] != 0xA5);

     for (int i = 1; i < 7; i++) {
         descriptor[i] = read_uint8();
     }

     if (affichage) {
         printf("New scan\n");
     }

     int l = 0;
     int nbr_point = 0;

     do {
         data[nbr_point] = ((scan[4] << 8) | scan[3]) / 4.0;
         angle[nbr_point] = ((scan[2] << 7) | (scan[1] >> 1)) / 64.0;

         if (dist_min < data[nbr_point] && data[nbr_point] < dist_max) {
             nbr_point++;
             if (affichage) {
                 printf("%2.2X | %2.2X | %2.2X | %2.2X | %2.2X | %9.3f mm | %9.3f °| %d\n", scan[0], scan[1], scan[2],
                        scan[3], scan[4], data[nbr_point], angle[nbr_point], nbr_point);
             }
         }

         l = read_uint8_n(scan, 5);

         if (l != 5 * sizeof(uint8_t)) {
             if (affichage) {
                 printf("Problème de communication\n");
             }
             return -1;
         }

     } while (nbr_point < nb_point);

     free(scan);

     if (affichage) {
         printf("End of scan\n");
     }

     return 0;
 }

 int Lidar::scan_cart(float *X, float *Y) {
     uint8_t *scan = (uint8_t *) malloc(5 * sizeof(uint8_t));
     uint8_t descriptor[7];

     float data;
     float angle;

     if (affichage) {
         printf("Begin of Lidar SCAN\n");
     }

     write_int(0xA5);
     write_int(0x20);

     if (affichage) {
         printf("Wait for a descriptor\n");
     }

     do {
         descriptor[0] = read_uint8();
     } while (descriptor[0] != 0xA5);

     for (int i = 1; i < 7; i++) {
         descriptor[i] = read_uint8();
     }

     if (affichage) {
         printf("New scan\n");
     }

     int l = 0;
     int nbr_point = 0;

     do {
         data = ((scan[4] << 8) | scan[3]) / 4.0;
         angle = ((scan[2] << 7) | (scan[1] >> 1)) / 64.0;

         if (dist_min < data && data < dist_max) {
             nbr_point++;

             X[nbr_point] = -data * cos(3.14 * angle / 180);
             Y[nbr_point] = data * sin(3.14 * angle / 180);

             if (affichage) {
                 printf("%2.2X | %2.2X | %2.2X | %2.2X | %2.2X | %9.3f mm | %9.3f °| %d\n", scan[0], scan[1], scan[2],
                        scan[3], scan[4], X[nbr_point], Y[nbr_point], nbr_point);
             }
         }

         l = read_uint8_n(scan, 5);

         if (l != 5 * sizeof(uint8_t)) {
             if (affichage) {
                 printf("Problème de communication\n");
             }
             return -1;
         }

     } while (nbr_point < nb_point);

     free(scan);

     if (affichage) {
         printf("End of scan\n");
     }

     return 0;
 }

 void Lidar::tri(float *data, float *angle, float *data_tri, float *angle_tri) {
     float min = angle[0];
     int index = 0;

     for (int i = 0; i < nb_point; i++) {
         for (int j = 0; j < nb_point; j++) {
             if (angle[j] < min) {
                 min = angle[j];
                 index = j;
             }
         }
         data_tri[i] = data[index];
         angle_tri[i] = angle[index];
         angle[index] = 500;
         min = 400;
     }
 }

 int Lidar::point_part(float *data, float *angle, float *X_part, float *Y_part, float *angle_part) {
     nb_part = 0;
//     printf("Point part\n");
     float dist_moy = data[0];
     float angle_moy = angle[0];
     float nbr_moy = 1;
     float d, d1;

     float X[nb_point];
     float Y[nb_point];

     for (int i = 1; i < nb_point; i++) {
         X[i] = -data[i] * cos(3.14 * angle[i] / 180);
         Y[i] = data[i] * sin(3.14 * angle[i] / 180);
     }

     int index = 0;

     float x, y, x1, y1;

     for (int i = 1; i < nb_point; i++) {

         x = -(dist_moy / nbr_moy) * cos(3.14 * angle_moy / nbr_moy / 180);
         y = (dist_moy / nbr_moy) * sin(3.14 * angle_moy / nbr_moy / 180);
         x1 = -(data[i]) * cos(3.14 * angle[i] / 180);
         y1 = (data[i]) * sin(3.14 * angle[i] / 180);
         d1 = sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));

         d = sqrt((X[i] - X[i - 1]) * (X[i] - X[i - 1]) + (Y[i] - Y[i - 1]) * (Y[i] - Y[i - 1]));
         if (d > dist_seuil || d1 > seuil_saut) {
             if (nbr_moy >= nbr_seuil && nbr_moy <= nbr_seuil_max) {
                 dist_moy /= nbr_moy;
                 angle_moy /= nbr_moy;

                 if (dist_moy < seuil_arret) {
                     printf("ALERTE: Objet trop proche\n");
                     return -1;
                 } else if (dist_moy < seuil_danger) {
                     printf("ATTENTION: Objet proche\n");
                 }

                 d = sqrt((X[i - 1] - X[index]) * (X[i - 1] - X[index]) + (Y[i - 1] - Y[index]) * (Y[i - 1] - Y[index]));
                 if (d > taille_rep_min && d < taille_rep_max) {
                     X_part[nb_part] = -(dist_moy) * cos(3.14 * angle_moy / 180);
                     Y_part[nb_part] = (dist_moy) * sin(3.14 * angle_moy / 180);
                     angle_part[nb_part] = angle_moy;
                     if (affichage) {
                         printf("| %9.3f | %9.3f |\n", X_part[nb_part], Y_part[nb_part]);
                     }
                     nb_part++;
                 }
             }

             index = i;
             dist_moy = 0;
             angle_moy = 0;
             nbr_moy = 0;
         }

         dist_moy += data[i];
         angle_moy += angle[i];
         nbr_moy++;
     }
     return 0;
 }

 int Lidar::balises(float *X_part, float *Y_part, float *angle_part) {
     string name;
     string filename = "test";
     int result;

     name = "Donnees/balise_X_" + filename + ".txt";
     FILE *balise_X = fopen(name.c_str(), "w+");
     name = "Donnees/balise_Y_" + filename + ".txt";
     FILE *balise_Y = fopen(name.c_str(), "w+");

     float X_prov[3] = {0};
     float Y_prov[3] = {0};
     float angle_prov[3] = {0};

     int m = 0;
     int d;

     for (int i = 0; i < nb_part; i++) {
         X_prov[0] = X_part[i];
         Y_prov[0] = Y_part[i];
         angle_prov[0] = angle_part[i];

         for (int j = i; j < nb_part + i; j++) {
             d = sqrt((X_prov[0] - X_part[j % nb_part]) * (X_prov[0] - X_part[j % nb_part]) +
                      (Y_prov[0] - Y_part[j % nb_part]) * (Y_prov[0] - Y_part[j % nb_part]));

             if (d_min_1 < d && d < d_max_1 && X_prov[1] == 0) {
                 X_prov[1] = X_part[j % nb_part];
                 Y_prov[1] = Y_part[j % nb_part];
                 angle_prov[1] = angle_part[j % nb_part];
             } else if (d_min_1 < d && d < d_max_1) {
                 X_prov[2] = X_part[j % nb_part];
                 Y_prov[2] = Y_part[j % nb_part];
                 angle_prov[2] = angle_part[j % nb_part];
             }

             if (X_prov[1] != 0 and X_prov[2] != 0) {
                 d = sqrt((X_prov[1] - X_prov[2]) * (X_prov[1] - X_prov[2]) +
                          (Y_prov[1] - Y_prov[2]) * (Y_prov[1] - Y_prov[2]));
                 if (d_min_2 < d && d < d_max_2) {
                     for (int k = 0; k < 3; k++) {
                         balises_X[k] = X_prov[k];
                         balises_Y[k] = Y_prov[k];
                         angle_balises[k] = angle_prov[k];

                         fprintf(balise_X, "%9.3f ", balises_X[k]);
                         fprintf(balise_Y, "%9.3f ", balises_Y[k]);
                         if (affichage) {
                             printf("| %9.3f | %9.3f | %d |\n", balises_X[k], balises_Y[k], m);
                         }
                     }
                     m++;
                 }
             }
         }

         for (int k = 0; k < 3; k++) {
             X_prov[k] = 0;
             Y_prov[k] = 0;
             angle_prov[k] = 0;
         }
     }

     if (m == 0) {
         for (int i = 0; i < nb_part; i++) {
             X_prov[1] = X_part[i];
             Y_prov[1] = Y_part[i];
             angle_prov[1] = angle_part[i];

             for (int j = i; j < nb_part + i; j++) {
                 d = sqrt((X_prov[1] - X_part[j % nb_part]) * (X_prov[1] - X_part[j % nb_part]) +
                          (Y_prov[1] - Y_part[j % nb_part]) * (Y_prov[1] - Y_part[j % nb_part]));

                 if (d_min_1 < d && d < d_max_1) {
                     X_prov[2] = X_part[j % nb_part];
                     Y_prov[2] = Y_part[j % nb_part];
                     angle_prov[2] = angle_part[j % nb_part];
                 }

                 if (X_prov[2] != 0) {
                     for (int k = 1; k < 3; k++) {
                         balises_X[k] = X_prov[k];
                         balises_Y[k] = Y_prov[k];
                         angle_balises[k] = angle_prov[k];

                         fprintf(balise_X, "%9.3f ", balises_X[k]);
                         fprintf(balise_Y, "%9.3f ", balises_Y[k]);
                         if (affichage) {
                             printf("| %9.3f | %9.3f | %d |\n", balises_X[k], balises_Y[k], m);
                         }
                     }
                     m++;
                 }

                 for (int k = 0; k < 3; k++) {
                     X_prov[k] = 0;
                     Y_prov[k] = 0;
                     angle_prov[k] = 0;
                 }
             }
         }

         float dx, dy, dth;

         if (m == 0) {
             for (int i = 0; i < nb_part; i++) {
                 dx = balises_X[1] - X_part[i];
                 dy = balises_Y[1] - Y_part[i];
                 dth = angle_balises[1] - angle_part[i];

                 d = sqrt(dx * dx + dy * dy);
                 if (d <= seuil_balise) {
                     for (int k = 0; k < 3; k++) {
                         X_prov[k] = balises_X[k] - dx;
                         Y_prov[k] = balises_Y[k] - dy;
                         angle_prov[k] = angle_balises[k] - dth;
                     }
                     m++;
                 }
             }
             if (m == 1) {
                 for (int k = 1; k < 3; k++) {
                     balises_X[k] = X_prov[k];
                     balises_Y[k] = Y_prov[k];
                     angle_balises[k] = angle_prov[k];

                     fprintf(balise_X, "%9.3f ", balises_X[k]);
                     fprintf(balise_Y, "%9.3f ", balises_Y[k]);
                     if (affichage) {
                         printf("| %9.3f | %9.3f | %d |\n", balises_X[k], balises_Y[k], m);
                     }
                 }

             }
         }

         if (m == 0) {
             result = -1;
         } else {
             result = 1;
         }
     } else {
         result = 0;
     }

     fclose(balise_X);
     fclose(balise_Y);

     return result;
 }

 int Lidar::position() {

     float *data = (float *) malloc(sizeof(float) * nb_point);
     float *angle = (float *) malloc(sizeof(float) * nb_point);
     float *data_tri = (float *) malloc(sizeof(float) * nb_point);
     float *angle_tri = (float *) malloc(sizeof(float) * nb_point);
     float *X_part = (float *) malloc(sizeof(float) * nb_point);
     float *Y_part = (float *) malloc(sizeof(float) * nb_point);
     float *angle_part = (float *) malloc(sizeof(float) * nb_point);

     for (int i = 0; i < nb_point; i++) {
         data[i] = 0;
         angle[i] = 0;
         data_tri[i] = 0;
         angle_tri[i] = 0;
         X_part[i] = 0;
         Y_part[i] = 0;
         angle_part[i] = 0;
     }
     scan_pol(data, angle);

     tri(data, angle, data_tri, angle_tri);
     point_part(data_tri, angle_tri, X_part, Y_part, angle_part);
     balises(X_part, Y_part, angle_part);

     float r1 = sqrt((balises_X[2]) * (balises_X[2]) + (balises_Y[2]) * (balises_Y[2]));
     float r2 = sqrt((balises_X[1]) * (balises_X[1]) + (balises_Y[1]) * (balises_Y[1]));
     int d = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
     float a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
     float h = sqrt(abs(r1 * r1 - a * a));
     float x5 = x1 + a / d * (x2 - x1);
     float y5 = y1 + a / d * (y2 - y1);

     x = x5 + (h / d * (y2 - y1));
     y = y5 + (h / d * (x2 - x1));

     float theta = (90 + angle_balises[1] - 180 / 3.14 * atan((x - x2) / (y2 - y)) - 45);

     if (affichage) {
         printf("%f %f %f\n", x, y, theta);
     }

     free(data);
     free(angle);
     free(data_tri);
     free(angle_tri);
     free(X_part);
     free(Y_part);
     free(angle_part);

     return 0;
 }

 void Lidar::get_position(int *pos_x, int *pos_y) {
     *pos_x = x;
     *pos_y = y;
 }