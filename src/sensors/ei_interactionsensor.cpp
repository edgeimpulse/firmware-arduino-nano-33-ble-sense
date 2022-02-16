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

#include "ei_interactionsensor.h"
#include "ei_device_nano_ble33.h"
#include "sensor_aq.h"

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