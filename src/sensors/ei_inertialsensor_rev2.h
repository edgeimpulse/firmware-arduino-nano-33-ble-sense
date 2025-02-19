
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

#ifndef _EI_INERTIALSENSOR_REV2_H
#define _EI_INERTIALSENSOR_REV2_H

/* Include ----------------------------------------------------------------- */
#include "ei_config_types.h"
#include "ei_fusion.h"

#define INERTIAL_REV_2_AXIS_SAMPLED			    6
#define MAG_REV_2_AXIS_SAMPLED			        3

/* Function prototypes ----------------------------------------------------- */
bool ei_inertial_rev2_init(void);
float *ei_fusion_inertial_rev2_read_data(int n_samples);
float *ei_fusion_mag_rev2_read_data(int n_samples);


static const ei_device_fusion_sensor_t inertial_rev2_sensor = {
    // name of sensor module to be displayed in fusion list
    "Inertial (Accelerometer / Gyroscope)",
    // number of sensor module axis
    INERTIAL_REV_2_AXIS_SAMPLED,
    // sampling frequencies
    { 20.0f, 50.0f, 62.5f, 100.0f, 200.0f },
    // axis name and units payload (must be same order as read in)
    { {"accX", "m/s2"}, {"accY", "m/s2"}, {"accZ", "m/s2"}, {"gyrX", "deg/s"}, {"gyrY", "deg/s"}, {"gyrZ", "deg/s"} },
    // reference to read data function
    &ei_fusion_inertial_rev2_read_data
};

static const ei_device_fusion_sensor_t mag_rev2_sensor = {
    // name of sensor module to be displayed in fusion list
    "Magnetometer",
    // number of sensor module axis
    MAG_REV_2_AXIS_SAMPLED,
    // sampling frequencies
    { 10.0f },
    // axis name and units payload (must be same order as read in)
    { {"magX", "uT"}, {"magY", "uT"}, {"magZ", "uT"} },
    // reference to read data function
    &ei_fusion_mag_rev2_read_data
};

#endif
