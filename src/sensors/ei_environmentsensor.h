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

#ifndef _EI_ENVIRONMENTSENSOR_H
#define _EI_ENVIRONMENTSENSOR_H

/* Include ----------------------------------------------------------------- */
#include "ei_config_types.h"
#include "ei_fusion.h"

/** Number of axis used and sample data format */
#define ENVIRONMENT_AXIS_SAMPLED			3

/* Function prototypes ----------------------------------------------------- */
bool ei_environment_init(void);
float *ei_fusion_environment_read_data(int n_samples);

static const ei_device_fusion_sensor_t environment_sensor = {
    // name of sensor module to be displayed in fusion list
    "Environmental (Temperature / Humidity / Pressure)",
    // number of sensor module axis
    ENVIRONMENT_AXIS_SAMPLED,
    // sampling frequencies
    { 1.0f, 12.5f },
    // axis name and units payload (must be same order as read in)
    { {"temperature", "degC"}, {"humidity", "%"}, {"pressure", "kPa"} },
    // reference to read data function
    &ei_fusion_environment_read_data
};

#endif