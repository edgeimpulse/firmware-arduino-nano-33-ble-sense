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
#include "ei_device_nano_ble33.h"
#include "ei_microphone.h"
#include "ei_camera.h"
#include <stdio.h>
#include <stdarg.h>
#include "Arduino.h"
#include "mbed.h"
#include "ei_flash_nano_ble33.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/ei_fusion.h"

using namespace rtos;
using namespace events;

/* Constants --------------------------------------------------------------- */

/** Memory location for the Nordic nRF52 device address */
#define DEVICE_ID_LSB_ADDR  ((uint32_t)0x100000A4)
#define DEVICE_ID_MSB_ADDR  ((uint32_t)0x100000A8)

/** Max size for device id array */
#define DEVICE_ID_MAX_SIZE  32

/** Sensors */
typedef enum
{
    MICROPHONE = 0,

}used_sensors_t;

#define EDGE_STRINGIZE_(x) #x
#define EDGE_STRINGIZE(x) EDGE_STRINGIZE_(x)

/** MBED thread */
Thread* fusion_thread;
EventQueue* fusion_queue;
int queue_id;

#if MULTI_FREQ_ENABLED == 1
void multi_sample_thread(void);
#endif

/* Public functions -------------------------------------------------------- */
EiDeviceNanoBle33::EiDeviceNanoBle33(EiDeviceMemory* mem)
{
    EiDeviceInfo::memory = mem;

    load_config();

    init_device_id();

    camera_present = ei_camera_init();

    device_type = std::string(EDGE_STRINGIZE(TARGET_NAME));

    /* Clear frequency arrays */
    for(int i = 0; i < EI_DEVICE_N_SENSORS; i++) {
        for(int y = 0; y < EI_MAX_FREQUENCIES; y++) {
            sensors[i].frequencies[y] = 0.f;
        }
    }
}

void EiDeviceNanoBle33::init_device_id(void)
{
    uint32_t *id_msb = (uint32_t *)DEVICE_ID_MSB_ADDR;
    uint32_t *id_lsb = (uint32_t *)DEVICE_ID_LSB_ADDR;
    char buf[DEVICE_ID_MAX_SIZE];

    /* Setup device ID */
    snprintf(&buf[0], DEVICE_ID_MAX_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X"
        ,(*id_msb >> 8) & 0xFF
        ,(*id_msb >> 0) & 0xFF
        ,(*id_lsb >> 24)& 0xFF
        ,(*id_lsb >> 16)& 0xFF
        ,(*id_lsb >> 8) & 0xFF
        ,(*id_lsb >> 0) & 0xFF
        );

    device_id = std::string(buf);
    device_type = std::string(EDGE_STRINGIZE(TARGET_NAME));

}

/**
 * @brief get_device is a static method of EiDeviceInfo class
 * It is used to implement singleton paradigm, so we are returning
 * here pointer always to the same object (dev)
 * 
 * @return EiDeviceInfo* 
 */
EiDeviceInfo* EiDeviceInfo::get_device(void)
{
    static EiFlashMemory memory(sizeof(EiConfig));
    static EiDeviceNanoBle33 dev(&memory);

    return &dev;
}

/**
 * @brief      Create sensor list with sensor specs
 *             The studio and daemon require this list
 * @param      sensor_list       Place pointer to sensor list
 * @param      sensor_list_size  Write number of sensors here
 *
 * @return     False if all went ok
 */
bool EiDeviceNanoBle33::get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size)
{
    /* Calculate number of bytes available on flash for sampling, reserve 1 block for header + overhead */
    uint32_t available_bytes = (memory->get_available_sample_blocks()-1) * memory->block_size;

    sensors[MICROPHONE].name = "Built-in microphone";
    sensors[MICROPHONE].start_sampling_cb = &ei_microphone_sample_start;
    sensors[MICROPHONE].max_sample_length_s = available_bytes / (16000 * 2);
    sensors[MICROPHONE].frequencies[0] = 16000.0f;
    sensors[MICROPHONE].frequencies[1] = 8000.0f;
    sensors[MICROPHONE].frequencies[2] = 4000.0f;
    *sensor_list      = sensors;
    *sensor_list_size = EI_DEVICE_N_SENSORS;

    return false;
}

/**
 * @brief      Create resolution list for snapshot setting
 *             The studio and daemon require this list
 * @param      snapshot_list       Place pointer to resolution list
 * @param      snapshot_list_size  Write number of resolutions here
 *
 * @return     False if all went ok
 */
EiSnapshotProperties EiDeviceNanoBle33::get_snapshot_list(void)
{
    ei_device_snapshot_resolutions_t *res = NULL;
    uint8_t res_num = 0;

    EiSnapshotProperties props = {
        .has_snapshot = false,
        .support_stream = false,
        .color_depth = "RGB",
        .resolutions_num = res_num,
        .resolutions = res
    };

    if(this->camera_present == true) {
        get_resolutions(&res, &res_num);
        props.has_snapshot = true;
        props.support_stream = true;
        props.color_depth = "RGB";
        props.resolutions_num = res_num;
        props.resolutions = res;
    }

    return props;
}

/**
 * @brief Setup timer or thread with given interval and call cb function each period
 * @param sample_read_cb
 * @param sample_interval_ms
 * @return true
 */
bool EiDeviceNanoBle33::start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms)
{
    osStatus retstatus;
    fusion_thread = new Thread;
    fusion_queue = new EventQueue;
    retstatus = fusion_thread->start(callback(fusion_queue, &EventQueue::dispatch_forever));    
    queue_id = fusion_queue->call_every(sample_interval_ms, sample_read_cb);

    if ((queue_id == 0) || (retstatus != 0)) {
        ei_printf("Can't allocate new thread\r\n");
        return false;
    }

    return true;
}

#if MULTI_FREQ_ENABLED == 1
/**
 * 
 */
bool EiDeviceNanoBle33::start_multi_sample_thread(void (*sample_multi_read_cb)(uint8_t), float* multi_sample_interval_ms, uint8_t num_fusioned)
{
    uint8_t i;
    uint8_t flag = 0;

    this->sample_multi_read_callback = sample_multi_read_cb;
    this->fusioning = num_fusioned;
    this->multi_sample_interval.clear();

    for (i = 0; i < num_fusioned; i++){
        this->multi_sample_interval.push_back(1.f/multi_sample_interval_ms[i]*1000.f);
    }

    /* to improve, we consider just a 2 sensors case for now */
    this->sample_interval = ei_fusion_calc_multi_gcd(this->multi_sample_interval.data(), this->fusioning);

    /* force first reading */
    for (i = 0; i < this->fusioning; i++){
            flag |= (1<<i);
    }
    this->sample_multi_read_callback(flag);

    this->actual_timer = 0;

    osStatus retstatus;
    fusion_thread = new Thread;
    fusion_queue = new EventQueue;
    retstatus = fusion_thread->start(callback(fusion_queue, &EventQueue::dispatch_forever));    
    queue_id = fusion_queue->call_every(sample_interval_ms, multi_sample_thread);

    if ((queue_id == 0) || (retstatus != 0)) {
        ei_printf("Can't allocate new thread\r\n");
        return false;
    }

    return true;
}
#endif

/**
 * @brief Stop timer of thread
 * @return true
 */
bool EiDeviceNanoBle33::stop_sample_thread(void)
{    
    fusion_queue->cancel(queue_id);
    fusion_queue->break_dispatch();
    delete fusion_queue;
    fusion_thread->join();
    delete fusion_thread;

    return true;
}

#if MULTI_FREQ_ENABLED == 1
/**
 * @brief Thread that handles the multi fusion sampling
 * 
 */
void multi_sample_thread(void)
{
    EiDeviceNanoBle33 *dev = static_cast<EiDeviceNanoBle33*>(EiDeviceInfo::get_device());

    uint8_t flag = 0;
    uint8_t i = 0;
    
    dev->actual_timer += dev->get_sample_interval();  /* update actual time */

    for (i = 0; i < dev->get_fusioning(); i++){
        if (((uint32_t)(dev->actual_timer % (uint32_t)dev->multi_sample_interval.at(i))) == 0) {   /* check if period of sensor is a multiple of actual time*/
            flag |= (1<<i);                                                                     /* if so, time to sample it! */
        }
    }

    if (dev->sample_multi_read_callback != nullptr){
        dev->sample_multi_read_callback(flag);        
    }    
}
#endif

/**
 * @brief      Create resolution list for resizing
 * @param      resize_list       Place pointer to resolution list
 * @param      resize_list_size  Write number of resolutions here
 *
 * @return     False if all went ok
 */
bool EiDeviceNanoBle33::get_resize_list(
    const ei_device_snapshot_resolutions_t **resize_list,
    size_t *resize_list_size)
{
    resize_resolutions[0].width = 42;
    resize_resolutions[0].height = 32;
    resize_resolutions[1].width = 128;
    resize_resolutions[1].height = 96;

    *resize_list = resize_resolutions;
    *resize_list_size = EI_DEVICE_N_RESIZE_RESOLUTIONS;

    return false;
}

/**
 * @brief      Write serial data with length to Serial output
 *
 * @param      data    The data
 * @param[in]  length  The length
 */
void ei_write_string(char *data, int length) {
    Serial.write(data, length);
}

/**
 * @brief      Check if new serial data is available
 *
 * @return     Returns number of available bytes
 */
int ei_get_serial_available(void) {
    return Serial.available();
}

/**
 * @brief      Get next available byte
 *
 * @return     byte
 */
char ei_get_serial_byte(void) {
    return Serial.read();
}
