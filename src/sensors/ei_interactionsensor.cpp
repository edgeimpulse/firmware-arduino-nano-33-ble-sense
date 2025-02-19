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
#include <stdint.h>
#include <stdlib.h>

#include "ei_interactionsensor.h"
#include "ei_device_nano_ble33.h"
#include "firmware-sdk/sensor-aq/sensor_aq.h"

#include <Arduino_APDS9960.h>
#include "mbed.h"

/* Constant defines -------------------------------------------------------- */
extern void ei_printf(const char *format, ...);

static float apds_data[INTERACTION_AXIS_SAMPLED];
static int temp_data[4];

bool ei_interaction_init(void)
{
	if (!APDS.begin()) {
		ei_printf("Failed to initialize APDS!\r\n");
	}
	else {
		ei_printf("APDS initialized\r\n");
    }
    ei_add_sensor_to_fusion_list(interaction_sensor);
}

float *ei_fusion_interaction_read_data(int n_samples)
{
    if (APDS.colorAvailable()) {
        APDS.readColor(temp_data[0], temp_data[1], temp_data[2], temp_data[3]);

        apds_data[0] = temp_data[0];
        apds_data[1] = temp_data[1];
        apds_data[2] = temp_data[2];
        apds_data[3] = temp_data[3];
    }

    if (APDS.proximityAvailable()) {
        apds_data[4] = (float)APDS.readProximity();
    }

    if (APDS.gestureAvailable()) {
        apds_data[5] = (float)APDS.readGesture();
    }

    return apds_data;
}