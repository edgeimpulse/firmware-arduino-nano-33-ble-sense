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
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <stdint.h>
#include <stdlib.h>

#include "ei_sampler.h"
#include "ei_config_types.h"
#include "ei_device_nano_ble33.h"

#include "mbed.h"
#include "ei_flash_nano_ble33.h"
#include "sensor_aq_mbedtls_hs256.h"


using namespace rtos;
using namespace events;

/* Private variables ------------------------------------------------------- */
static uint32_t samples_required;
static uint32_t samples_required_increase;
static volatile uint32_t current_sample;
static uint32_t sample_buffer_size;
static uint32_t headerOffset = 0;


static char write_word_buf[4];
static int write_addr = 0;

static size_t ei_write(const void *buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM*)
{
    EiDeviceMemory *mem = EiDeviceInfo::get_device()->get_memory();

    for(int i=0; i<count; i++) {

        write_word_buf[write_addr&0x3] = *((char *)buffer++);

        if((++write_addr & 0x03) == 0x00) {
            mem->write_sample_data((const uint8_t*)write_word_buf, (write_addr - 4) + headerOffset, 4);
        }

    }

    return count;
}

static int ei_seek(EI_SENSOR_AQ_STREAM*, long int offset, int origin)
{
    return 0;
}

static unsigned char ei_mic_ctx_buffer[1024];
static sensor_aq_signing_ctx_t ei_mic_signing_ctx;
static sensor_aq_mbedtls_hs256_ctx_t ei_mic_hs_ctx;
static sensor_aq_ctx ei_mic_ctx = {
    { ei_mic_ctx_buffer, 1024 },
    &ei_mic_signing_ctx,
    &ei_write,
    &ei_seek,
    NULL,
};

static void ei_write_last_data(void)
{
    EiDeviceMemory *mem = EiDeviceInfo::get_device()->get_memory();
    char fill = (write_addr & 0x03);

    if(fill != 0x00) {
        for(int i=fill; i<4; i++) {
            write_word_buf[i] = 0xFF;
        }

        mem->write_sample_data((const uint8_t*)write_word_buf, (write_addr & ~0x03) + headerOffset, 4);
    }
}

EI_SENSOR_AQ_STREAM stream;


/* Private function prototypes --------------------------------------------- */
static void finish_and_upload(void);
static bool sample_data_callback(const void *sample_buf, uint32_t byteLenght);

static bool create_header(sensor_aq_payload_info *payload);


/**
 * @brief      Setup and start sampling, write CBOR header to flash
 *
 * @param      v_ptr_payload  sensor_aq_payload_info pointer hidden as void
 * @param[in]  sample_size    Number of bytes for 1 sample (include all axis)
 *
 * @return     true if successful
 */
bool ei_sampler_start_sampling(void *v_ptr_payload, starter_callback ei_sample_start, uint32_t sample_size)
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();
    EiDeviceMemory *mem = dev->get_memory();
    sensor_aq_payload_info *payload = (sensor_aq_payload_info *)v_ptr_payload;

    ei_printf("Sampling settings:\n");
    ei_printf("\tInterval: %.5f ms.\n", dev->get_sample_interval_ms());
    ei_printf("\tLength: %lu ms.\n", dev->get_sample_length_ms());
    ei_printf("\tName: %s\n", dev->get_sample_label().c_str());
    ei_printf("\tHMAC Key: %s\n", dev->get_sample_hmac_key().c_str());
    ei_printf("\tFile name: /fs/%s\n", dev->get_sample_label().c_str());


    samples_required = (uint32_t)((float)dev->get_sample_length_ms());
    samples_required_increase = (uint32_t)dev->get_sample_interval_ms();

    sample_buffer_size = (samples_required/samples_required_increase) * sample_size * 2;
    current_sample = 0;

    // Minimum delay of 2000 ms for daemon
    if(((sample_buffer_size / mem->block_size)+1) * mem->block_erase_time < 2000) {
        ThisThread::sleep_for(2000 - ((sample_buffer_size / mem->block_size)+1) * mem->block_erase_time);
        ei_printf("Starting in %lu ms... (or until all flash was erased)\n", 2000);
    }
    else {
        ei_printf("Starting in %lu ms... (or until all flash was erased)\n",
        ((sample_buffer_size / mem->block_size)+1) * mem->block_erase_time);
    }

	if(mem->erase_sample_data(0, sample_buffer_size) != sample_buffer_size) {
        ei_printf("ERR: Failed to erase sample memory\r\n");
		return false;
    }

    if(create_header(payload) == false)
        return false;

    ei_printf("Sampling...\n");
    if(ei_sample_start(&sample_data_callback, dev->get_sample_interval_ms()) == false)
        return false;
	
    while(current_sample <= samples_required) {
        ThisThread::sleep_for(50);
    };

    ei_write_last_data();
    write_addr++;

    uint8_t final_byte[] = { 0xff };
    int ctx_err = ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, final_byte, 1);
    if (ctx_err != 0) {
        return ctx_err;
    }

    // finish the signing
    ctx_err = ei_mic_ctx.signature_ctx->finish(ei_mic_ctx.signature_ctx, ei_mic_ctx.hash_buffer.buffer);

    finish_and_upload();

    return true;
}


static bool create_header(sensor_aq_payload_info *payload)
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();
    EiDeviceMemory *mem = dev->get_memory();
    sensor_aq_init_mbedtls_hs256_context(&ei_mic_signing_ctx, &ei_mic_hs_ctx, dev->get_sample_hmac_key().c_str());


    int tr = sensor_aq_init(&ei_mic_ctx, payload, NULL, true);

    if (tr != AQ_OK) {
        ei_printf("sensor_aq_init failed (%d)\n", tr);
        return false;
    }
    // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
    // That should give us the whole header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_mic_ctx.cbor_buffer.len - 1; ix >= 0; ix--) {
        if (((uint8_t*)ei_mic_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        ei_printf("Failed to find end of header\n");
        return false;
    }

    // Write to blockdevice
    tr = mem->write_sample_data((const uint8_t*)ei_mic_ctx.cbor_buffer.ptr, 0, end_of_header_ix);
    ei_printf("Try to write %d bytes\r\n", end_of_header_ix);
    if (tr != end_of_header_ix) {
        ei_printf("Failed to write to header blockdevice\n");
        return false;
    }

    ei_mic_ctx.stream = &stream;

    headerOffset = end_of_header_ix;
    write_addr = 0;

    return true;
}

/**
 * @brief      Sampling is finished, signal no uploading file
 *
 */
static void finish_and_upload(void)
{
    ei_printf("Done sampling, total bytes collected: %u\n", write_addr);
    ei_printf("[1/1] Uploading file to Edge Impulse...\n");
    ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=0, to=%lu.\n", write_addr + headerOffset);//sample_buffer_size + headerOffset);
    ei_printf("[1/1] Uploading file to Edge Impulse OK (took 0 ms.)\n");
    ei_printf("OK\n");
}

/**
 * @brief      Write samples to FLASH in CBOR format
 *
 * @param[in]  sample_buf  The sample buffer
 * @param[in]  byteLenght  The byte lenght
 *
 * @return     true if all required samples are received. Caller should stop sampling,
 */
static bool sample_data_callback(const void *sample_buf, uint32_t byteLenght)
{
    sensor_aq_add_data(&ei_mic_ctx, (float *)sample_buf, byteLenght / sizeof(float));    
    current_sample += samples_required_increase;

    if(current_sample > samples_required) {
        return true;
    }
    else {
        return false;
    }
}
