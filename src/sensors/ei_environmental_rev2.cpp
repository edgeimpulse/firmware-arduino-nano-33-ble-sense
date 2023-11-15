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
#include <stdint.h>
#include <stdlib.h>
#include "ei_environmental_rev2.h"

#include <Arduino_HS300x.h>
#include <Arduino_LPS22HB.h>

/* Constant defines -------------------------------------------------------- */
extern void ei_printf(const char *format, ...);

static float htps_data[ENVIRONMENT_AXIS_SAMPLED];

bool ei_environment_rev2_init(void)
{
	if (!HS300x.begin()) {
        ei_printf("Failed to initialize HS300x!\r\n");
		return false;
	}
	else {
		ei_printf("HS300x initialized\r\n");
	}

	if (!BARO.begin()) {
		ei_printf("Failed to initialize BARO!\r\n");
	}
	else {
		ei_printf("BARO initialized\r\n");
	}

    ei_add_sensor_to_fusion_list(environment_sensor_rev2);

	return true;
}


float *ei_fusion_environment_rev2_read_data(int n_samples)
{
    htps_data[0] = HS300x.readTemperature();
    htps_data[1] = HS300x.readHumidity();

	htps_data[2] = BARO.readPressure(); // (PSI/MILLIBAR/KILOPASCAL) default kPa

	return htps_data;
}
