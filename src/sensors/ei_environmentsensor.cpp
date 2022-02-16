/* Edge Impulse ingestion SDK
 * Copyright (c) 2020 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Include ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdlib.h>

#include "ei_environmentsensor.h"
#include "ei_device_nano_ble33.h"
#include "sensor_aq.h"

#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>
#include "mbed.h"

/* Constant defines -------------------------------------------------------- */
extern void ei_printf(const char *format, ...);

static float htps_data[ENVIRONMENT_AXIS_SAMPLED];

bool ei_environment_init(void)
{
	if (!HTS.begin()) {
		ei_printf("Failed to initialize HTS!\r\n");
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
}


float *ei_fusion_environment_read_data(int n_samples)
{
	htps_data[0] = HTS.readTemperature();
	htps_data[1] = HTS.readHumidity();
	htps_data[2] = BARO.readPressure(); // (PSI/MILLIBAR/KILOPASCAL) default kPa

	return htps_data;
}
