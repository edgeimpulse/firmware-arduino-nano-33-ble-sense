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

#include "ei_environmentsensor.h"
#include "ei_device_nano_ble33.h"
#include "firmware-sdk/sensor-aq/sensor_aq.h"

#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>
#include "mbed.h"

/* Constant defines -------------------------------------------------------- */
extern void ei_printf(const char *format, ...);

static bool rev2 = false;
static float htps_data[ENVIRONMENT_AXIS_SAMPLED];

bool ei_environment_init(void)
{
	if (!HTS.begin()) {
		ei_printf("Failed to initialize HTS!\r\n");
		return false;
	}
	else {
		ei_printf("HTS initialized\r\n");
	}

	if (!BARO.begin()) {
		ei_printf("Failed to initialize BARO!\r\n");
	}
	else {
		ei_printf("BARO initialized\r\n");
	}

    ei_add_sensor_to_fusion_list(environment_sensor);

	return true;
}


float *ei_fusion_environment_read_data(int n_samples)
{
	htps_data[0] = HTS.readTemperature();
	htps_data[1] = HTS.readHumidity();

	htps_data[2] = BARO.readPressure(); // (PSI/MILLIBAR/KILOPASCAL) default kPa

	return htps_data;
}
