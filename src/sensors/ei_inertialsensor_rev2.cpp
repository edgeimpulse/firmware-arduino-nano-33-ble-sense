/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
