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

#ifndef EI_DEVICE_NANO_BLE33
#define EI_DEVICE_NANO_BLE33

/* Include ----------------------------------------------------------------- */
#include "ei_device_info_lib.h"
#include "ei_fusion_sensors_config.h"

/** Number of sensors used */
#define EI_DEVICE_N_SENSORS					1
#define EI_MAX_FREQUENCIES                  5
#define EI_DEVICE_N_RESOLUTIONS				2
#define EI_DEVICE_N_RESIZE_RESOLUTIONS		2

/**
 * @brief      Class description and implementation of device specific 
 * 			   characteristics
 */	
class EiDeviceNanoBle33 : public EiDeviceInfo
{
private:
	ei_device_sensor_t sensors[EI_DEVICE_N_SENSORS];
	ei_device_snapshot_resolutions_t snapshot_resolutions[EI_DEVICE_N_RESOLUTIONS];
	ei_device_snapshot_resolutions_t resize_resolutions[EI_DEVICE_N_RESIZE_RESOLUTIONS];
	bool camera_present;
public:	
	EiDeviceNanoBle33(EiDeviceMemory* mem);
	void init_device_id(void);
	bool get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size);
	bool get_resize_list(const ei_device_snapshot_resolutions_t **resize_list, size_t *resize_list_size);
	EiSnapshotProperties get_snapshot_list(void);
    bool start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms) override;
    bool stop_sample_thread(void) override;

#if MULTI_FREQ_ENABLED == 1
	bool start_multi_sample_thread(void (*sample_multi_read_cb)(uint8_t), float* multi_sample_interval_ms, uint8_t num_fusioned) override;		
#endif
};

#endif
