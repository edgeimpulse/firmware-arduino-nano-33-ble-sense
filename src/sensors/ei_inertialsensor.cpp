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
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <stdint.h>
#include <stdlib.h>
#include "ei_inertialsensor.h"

#include "ei_sampler.h"
#include "ei_flash_nano_ble33.h"
#include "sensor_aq.h"

#include "Arduino_BMI270_BMM150.h"
#include "mbed.h"

#include "ei_device_nano_ble33.h"

using namespace rtos;
using namespace events;


/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f

Thread inertial_thread;
EventQueue inertial_queue;
mbed::Ticker inertial_sample_rate;

sampler_callback  inertial_cb_sampler;

static float acc_data[N_AXIS_SAMPLED];
static float imu_data[INERTIAL_AXIS_SAMPLED];

bool ei_inertial_init(void)
{
	if (!IMU.begin()) {
        ei_printf("Failed to initialize IMU!\r\n");
	}
	else {
        ei_printf("IMU initialized\r\n");
    }

    ei_add_sensor_to_fusion_list(inertial_sensor);
}

void ei_inertial_read_data(void)
{
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(acc_data[0], acc_data[1], acc_data[2]);

        acc_data[0] *= CONVERT_G_TO_MS2;
        acc_data[1] *= CONVERT_G_TO_MS2;
        acc_data[2] *= CONVERT_G_TO_MS2;
    }

    if(inertial_cb_sampler((const void *)&acc_data[0], SIZEOF_N_AXIS_SAMPLED))
        inertial_sample_rate.detach();
}

bool ei_inertial_sample_start(sampler_callback callsampler, float sample_interval_ms)
{
    inertial_cb_sampler = callsampler;

    inertial_thread.start(callback(&inertial_queue, &EventQueue::dispatch_forever));
    inertial_sample_rate.attach(inertial_queue.event(&ei_inertial_read_data), (sample_interval_ms / 1000.f));

    return true;
}

bool ei_inertial_setup_data_sampling(void)
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();
    EiDeviceMemory *mem = dev->get_memory();

    if (dev->get_sample_interval_ms() < 10.0f ) {
        dev->set_sample_interval_ms(10.0f);
    }

    // Calculate number of bytes available on flash for sampling, reserve 1 block for header + overhead
    uint32_t available_bytes = (mem->get_available_sample_blocks()-1) * mem->block_size;
    // Check available sample size before sampling for the selected frequency
    uint32_t requested_bytes = ceil((dev->get_sample_length_ms() / dev->get_sample_interval_ms()) * SIZEOF_N_AXIS_SAMPLED * 2);
    if(requested_bytes > available_bytes) {
        ei_printf("ERR: Sample length is too long. Maximum allowed is %ims at %.1fHz.\r\n", 
            (int)floor(available_bytes / ((SIZEOF_N_AXIS_SAMPLED * 2) / dev->get_sample_interval_ms())),
            (1000 / dev->get_sample_interval_ms()));
        return false;
    }

    sensor_aq_payload_info payload = {
        // Unique device ID (optional), set this to e.g. MAC address or device EUI **if** your device has one
        dev->get_device_id().c_str(),
        // Device type (required), use the same device type for similar devices
        dev->get_device_type().c_str(),
        // How often new data is sampled in ms. (100Hz = every 10 ms.)
        dev->get_sample_interval_ms(),
        // The axes which you'll use. The units field needs to comply to SenML units (see https://www.iana.org/assignments/senml/senml.xhtml)
        { { "accX", "m/s2" }, { "accY", "m/s2" }, { "accZ", "m/s2" }, },
    };

    ei_sampler_start_sampling(&payload, &ei_inertial_sample_start, SIZEOF_N_AXIS_SAMPLED);
}

float *ei_fusion_inertial_read_data(int n_samples)
{
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(imu_data[0], imu_data[1], imu_data[2]);

        for(int i = 0; i < 3; i++) {
            imu_data[i] = imu_data[i] > 2.f ? 2.f : imu_data[i] < -2.f ? -2.f : imu_data[i];
        }

        imu_data[0] *= CONVERT_G_TO_MS2;
        imu_data[1] *= CONVERT_G_TO_MS2;
        imu_data[2] *= CONVERT_G_TO_MS2;
    }

    if (n_samples > 3 && IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(imu_data[3], imu_data[4], imu_data[5]);
    }

    if (n_samples > 6 && IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(imu_data[6], imu_data[7], imu_data[8]);
    }

    return imu_data;
}