
/* Include ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdlib.h>

#include "ei_inertialsensor.h"

#include "ei_config_types.h"
#include "nano_fs_commands.h"

#include <Arduino_LSM9DS1.h>
#include "mbed.h"

#include "ei_device_nano_ble33.h"
#include "sensor_aq.h"

using namespace mbed;
using namespace rtos;
using namespace events;


/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f


extern void ei_printf(const char *format, ...);
extern ei_config_t *ei_config_get_config();
extern EI_CONFIG_ERROR ei_config_set_sample_interval(float interval);

Thread inertion_thread;
EventQueue intertion_queue;
Ticker sample_rate;

sampler_callback  cb_sampler;

static float imu_data[N_AXIS_SAMPLED];

bool ei_inertial_init(void)
{
	if (!IMU.begin()) {
		ei_printf("Failed to initialize IMU!\r\n");    
	}	
	else
		ei_printf("IMU initialized\r\n");
}

void ei_inertial_read_data(void)
{		
	if (IMU.accelerationAvailable()) {
	    IMU.readAcceleration(imu_data[0], imu_data[1], imu_data[2]);

        imu_data[0] *= CONVERT_G_TO_MS2;
        imu_data[1] *= CONVERT_G_TO_MS2;
        imu_data[2] *= CONVERT_G_TO_MS2;
	}

    #if(N_AXIS_SAMPLED == 6)
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(imu_data[3], imu_data[4], imu_data[5]);
    }
    #endif	

	if(cb_sampler((const void *)&imu_data[0], SIZEOF_N_AXIS_SAMPLED))
		sample_rate.detach();

}

bool ei_inertial_sample_start(sampler_callback callsampler, float sample_interval_ms)
{
	cb_sampler = callsampler;

    inertion_thread.start(callback(&intertion_queue, &EventQueue::dispatch_forever));
    sample_rate.attach(intertion_queue.event(&ei_inertial_read_data), (sample_interval_ms / 1000.f));	

    return true;
}

bool ei_inertial_setup_data_sampling(void)
{

    if (ei_config_get_config()->sample_interval_ms < 10.0f) {
        ei_config_set_sample_interval(10.0f);
    }

    sensor_aq_payload_info payload = {
        // Unique device ID (optional), set this to e.g. MAC address or device EUI **if** your device has one
        EiDevice.get_id_pointer(),
        // Device type (required), use the same device type for similar devices
        EiDevice.get_type_pointer(),
        // How often new data is sampled in ms. (100Hz = every 10 ms.)
        ei_config_get_config()->sample_interval_ms,
        // The axes which you'll use. The units field needs to comply to SenML units (see https://www.iana.org/assignments/senml/senml.xhtml)
        { { "accX", "m/s2" }, { "accY", "m/s2" }, { "accZ", "m/s2" }, 
        /*{ "gyrX", "dps" }, { "gyrY", "dps" }, { "gyrZ", "dps" } */},        
    };	
    
    ei_sampler_start_sampling(&payload, SIZEOF_N_AXIS_SAMPLED);
}