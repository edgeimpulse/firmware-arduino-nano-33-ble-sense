/*
 * Copyright (c) 2023 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/* Include ----------------------------------------------------------------- */
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <stdint.h>
#include <stdlib.h>
#include "ei_inertialsensor_rev2.h"
#include "ei_bm270_bmm150.h"

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f

#define CONVERT_ADC_GYR             (float)(250.0f/32768.0f)
#define CONVERT_ADC_MAG_XY          (float)(1300.0f/4096.0f)
#define CONVERT_ADC_MAG_Z           (float)(2500.0f/16384.0f)

static float imu_data[INERTIAL_REV_2_AXIS_SAMPLED];
static float mag_data[MAG_REV_2_AXIS_SAMPLED];

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_inertial_rev2_init(void)
{
    bool retval = true;

    if (!ei_IMU_BMI270_BMM150.begin()) {    
        ei_printf("Failed to initialize IMU for rev2!");
        retval = false;
    }
    else {
        ei_IMU_BMI270_BMM150.setContinuousMode();
        ei_printf("IMU for rev2 initialized\r\n");        
        ei_add_sensor_to_fusion_list(inertial_rev2_sensor);
        ei_add_sensor_to_fusion_list(mag_rev2_sensor);
    }
    
    return retval;
}

/**
 * @brief 
 * 
 * @param n_samples 
 * @return float* 
 */
float *ei_fusion_inertial_rev2_read_data(int n_samples)
{       
    memset(imu_data, 0, sizeof(imu_data));

    if (ei_IMU_BMI270_BMM150.accelerationAvailable()) {
        ei_IMU_BMI270_BMM150.readAcceleration(imu_data[0], imu_data[1], imu_data[2]);

        imu_data[0] *= CONVERT_G_TO_MS2;
        imu_data[1] *= CONVERT_G_TO_MS2;
        imu_data[2] *= CONVERT_G_TO_MS2;
    }

    if ((n_samples > 3) && (ei_IMU_BMI270_BMM150.gyroscopeAvailable())) {
        ei_IMU_BMI270_BMM150.readGyroscope(imu_data[3], imu_data[4], imu_data[5]);
        
        imu_data[3] *= CONVERT_ADC_GYR;
        imu_data[4] *= CONVERT_ADC_GYR;
        imu_data[5] *= CONVERT_ADC_GYR;
    }

    return imu_data;
}

/**
 * @brief 
 * 
 * @param n_samples 
 * @return float* 
 */
float *ei_fusion_mag_rev2_read_data(int n_samples)
{    
    memset(mag_data, 0, sizeof(mag_data));

    if (ei_IMU_BMI270_BMM150.magneticFieldAvailable()) {
        ei_IMU_BMI270_BMM150.readMagneticField(mag_data[0], mag_data[1], mag_data[2]);

        mag_data[0] *= CONVERT_ADC_MAG_XY;
        mag_data[1] *= CONVERT_ADC_MAG_XY;
        mag_data[2] *= CONVERT_ADC_MAG_Z;
    }

    return mag_data;
}
