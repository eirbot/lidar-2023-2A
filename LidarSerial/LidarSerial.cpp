/*
 * POKIBOT 2023
 * Mbed Software for Pokirobot V1
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#include "LidarSerial.h"
#include "tools.h"
#include "usart.h"
#include "dma.h"



EventFlags lidarThreadFlag;
uint8_t lidar_new_value, lidar_processing;
uint16_t lidar_overflow = 0, start_sequence_incr = 0;


uint8_t lidar_frame[LDS_01_TRAM_LENGTH];
uint16_t lidar_distances_mean[LDS_01_MEDIAN_RES];
uint8_t dma_mode = 0;
uint8_t lidar_back_trig = 0, lidar_front_trig = 0;

// General handler for UART Interrupt, need to be linked to NVIC using vector
// See https://os.mbed.com/forum/mbed/topic/33580/
void custom_usart1_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart1);
}

// General handler for DMA Interrupt, need to be linked to NVIC using vector
void custom_DMA2_Stream2_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

// Called by HAL_UART_IRQHandler and HAL_DMA_IRQHandler
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    if (dma_mode) {
        dma_mode = 0;
        lidarThreadFlag.set(LIDAR_THREAD_FLAG);
        HAL_UART_Receive_IT(&huart1, &lidar_new_value, 1);
    } else {

        if ((start_sequence_incr == 0) && (lidar_new_value == LDS_01_START_FIRST)) {
            start_sequence_incr = 1;
            lidar_frame[0] = lidar_new_value;
            HAL_UART_Receive_IT(&huart1, &lidar_new_value, 1);
        } else if ((start_sequence_incr == 1) && (lidar_new_value == LDS_01_START_SECOND)) {
            start_sequence_incr = 0;
            lidar_frame[1] = lidar_new_value;
            //            lidarThreadFlag.set(LIDAR_THREAD_FLAG);
            // setup DMA to get the rest of the message
            HAL_UART_Receive_DMA(&huart1, lidar_frame + 2, LDS_01_TRAM_LENGTH - 2);
            dma_mode = 1;

            if (lidar_processing) {
                lidar_overflow++;
            }

        } else {
            start_sequence_incr = 0;
            HAL_UART_Receive_IT(&huart1, &lidar_new_value, 1);
        }
    }
}

// Lidar serial on PA9 / PA10 at 230400bps (Robotis LDS-01)
void init_lidar_serial() {
    MX_DMA_Init();
    MX_USART1_UART_Init();
}

void lidarMain() {

    init_lidar_serial();

    uint32_t motor_speed = 0;
    uint16_t rpms;
    int lidar_index = 0;
    //    int print_wait = 0;
    while (true) {
        // Wait for asserv tick
        lidarThreadFlag.wait_any(LIDAR_THREAD_FLAG);
        lidar_processing = 1;

        // part of source code from Robotis:
        // https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver/blob/master/applications/lds_driver/lds_driver.cpp
        for (uint16_t i = 0; i < LDS_01_TRAM_LENGTH; i = i + 42) {
            if (lidar_frame[i] == 0xFA && lidar_frame[i + 1] == (0xA0 + i / 42)) {
                motor_speed += (lidar_frame[i + 3] << 8) + lidar_frame[i + 2];
                rpms = (lidar_frame[i + 3] << 8 | lidar_frame[i + 2]) / 10;

                uint32_t sum = 0;
                uint8_t sum_num = 0;
                for (uint16_t j = i + 4; j < i + 40; j = j + 6) { // 6 paquets
                    lidar_index = 6 * (i / 42) + (j - 4 - i) / 6;

                    uint8_t byte0 = lidar_frame[j];
                    uint8_t byte1 = lidar_frame[j + 1];
                    uint8_t byte2 = lidar_frame[j + 2];
                    uint8_t byte3 = lidar_frame[j + 3];

                    uint16_t intensity = (byte1 << 8) + byte0;
                    uint16_t range = (byte3 << 8) + byte2;

                    // add to median values
                    if (range != 0) { // if 0, then probably invalid data, or too far
                        sum += range;
                        sum_num++;
                    }
                }

                // update median
                lidar_distances_mean[(lidar_index / 6)]
                        = sum_num == 0 ? 0xFFFF : uint16_t(sum / sum_num);
            }
        }

        updateLidarDetect();
        lidar_processing = 0;

        //        print_wait++;
        //        if (print_wait > 5) {
        //            print_wait = 0;
        //            for (int i = 0; i < LDS_01_MEDIAN_RES; i++) {
        //                terminal_printf("r[%3d]=%3d\n", i, lidar_distances_mean[i]);
        //            }
        //        }
    }
}

void updateLidarDetect() {

    // back
    for (int index = LIDAR_BACK_MIN; index < LIDAR_BACK_MAX; index++) {
        lidar_back_trig = 0;
        if (lidar_distances_mean[index] < LIDAR_TRIG_DETECT) {
            lidar_back_trig = 1;
            break;
        }
    }

    // front
    for (int index = LIDAR_FRONT_MIN; index < LIDAR_FRONT_MAX; index++) {
        lidar_front_trig = 0;
        if (lidar_distances_mean[index] < LIDAR_TRIG_DETECT) {
            lidar_front_trig = 1;
            break;
        }
    }
}