
/*
 * Copyright (c) 2022 EdgeImpulse Inc.
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
