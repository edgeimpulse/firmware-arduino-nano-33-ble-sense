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

#ifndef _EI_ENVIRONMENTSENSOR_REV2_H
#define _EI_ENVIRONMENTSENSOR_REV2_H

/* Include ----------------------------------------------------------------- */
#include "ei_config_types.h"
#include "ei_fusion.h"

/** Number of axis used and sample data format */
#define ENVIRONMENT_AXIS_SAMPLED			3

/* Function prototypes ----------------------------------------------------- */
bool ei_environment_rev2_init(void);
float *ei_fusion_environment_rev2_read_data(int n_samples);

static const ei_device_fusion_sensor_t environment_sensor_rev2 = {
    // name of sensor module to be displayed in fusion list
    "Environmental (Temperature / Humidity / Pressure)",
    // number of sensor module axis
    ENVIRONMENT_AXIS_SAMPLED,
    // sampling frequencies
    { 1.0f, 12.5f },
    // axis name and units payload (must be same order as read in)
    { {"temperature", "degC"}, {"humidity", "%"}, {"pressure", "kPa"} },
    // reference to read data function
    &ei_fusion_environment_rev2_read_data
};

#endif