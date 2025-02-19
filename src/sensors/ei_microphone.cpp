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
#include "ei_microphone.h"
#include "ei_device_nano_ble33.h"
#include <PDM.h>
#include "setup.h"
#include "ei_config_types.h"
#include "sensor_aq_mbedtls_hs256.h"
#include "firmware-sdk/sensor-aq/sensor_aq_none.h"
#include "edge-impulse-sdk/CMSIS/DSP/Include/arm_math.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "ei_flash_nano_ble33.h"


typedef struct {
    int16_t *buffers[2];
    uint8_t buf_select;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

/* Extern declared --------------------------------------------------------- */
extern mbed::DigitalOut led;

using namespace ei;

static size_t ei_write(const void*, size_t size, size_t count, EI_SENSOR_AQ_STREAM*)
{
    ei_printf("Writing: %d\r\n", count);
    return count;
}

static int ei_seek(EI_SENSOR_AQ_STREAM*, long int offset, int origin)
{
    ei_printf("Seeking: %d\r\n", offset);
    return 0;
}

/* Private variables ------------------------------------------------------- */
static signed short *sampleBuffer;
static bool is_uploaded = false;
static bool record_ready = false;
static uint32_t headerOffset;
static uint32_t samples_required;
static uint32_t current_sample;
static uint32_t audio_sampling_frequency = 16000;

//static signed short *inference_buffers[2];
static inference_t inference;

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

/* Audio thread setup */
#define AUDIO_THREAD_STACK_SIZE             4096
static uint8_t AUDIO_THREAD_STACK[AUDIO_THREAD_STACK_SIZE];
Thread queue_thread(osPriorityHigh, AUDIO_THREAD_STACK_SIZE, AUDIO_THREAD_STACK, "audio-thread-stack");
static EventQueue mic_queue;


extern "C" {
    #include <hal/nrf_pdm.h>
}

/* Private functions ------------------------------------------------------- */

/**
 * @brief PDM clock frequency calculation based on 32MHz clock and
 * decimation filter ratio 80
 * @details For more info on clock generation:
 * https://infocenter.nordicsemi.com/index.jsp?topic=%2Fps_nrf5340%2Fpdm.html
 * @param sampleRate in Hz
 * @return uint32_t clk value
 */
static uint32_t pdm_clock_calculate(uint64_t sampleRate)
{
    const uint64_t PDM_RATIO = 80ULL;
    const uint64_t CLK_32MHZ = 32000000ULL;
    uint64_t clk_control = 4096ULL * (((sampleRate * PDM_RATIO) * 1048576ULL) / (CLK_32MHZ + ((sampleRate * PDM_RATIO) / 2ULL)));

    return (uint32_t)clk_control;
}

static void ei_microphone_blink() {
    led = !led;
}

static void audio_buffer_callback(uint32_t n_bytes)
{
    EiDeviceMemory *mem = EiDeviceInfo::get_device()->get_memory();
    mem->write_sample_data((const uint8_t*)sampleBuffer, headerOffset + current_sample, n_bytes);


    ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, (uint8_t*)sampleBuffer, n_bytes);

    current_sample += n_bytes;
    if(current_sample >= (samples_required << 1)) {
        record_ready = false;
        PDM.end();
        ei_free(sampleBuffer);
    }
}

/**
 * @brief 
 * 
 */
static void pdm_data_ready_callback(void)
{
    int bytesAvailable = PDM.available();


    // read into the sample buffer
    int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);

    if(record_ready == true) {
        mic_queue.call(&audio_buffer_callback, bytesRead);
    }
}

/**
 * @brief 
 * 
 * @param n_bytes 
 */
static void audio_buffer_inference_callback(uint32_t n_bytes)
{
    for(int i = 0; i < n_bytes>>1; i++) {
        inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];

        if(inference.buf_count >= inference.n_samples) {
            inference.buf_select ^= 1;
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

/**
 * @brief 
 * 
 */
static void pdm_data_ready_inference_callback(void)
{
    int bytesAvailable = PDM.available();

    // read into the sample buffer
    int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);

    if(record_ready == true) {
        mic_queue.call(&audio_buffer_inference_callback, bytesRead);
    }
}

static void finish_and_upload(void) {

    ei_printf("Done sampling, total bytes collected: %u\n", current_sample);

    mbed::Ticker t;
    t.attach(&ei_microphone_blink, 1.0f);
    ei_printf("[1/1] Uploading file to Edge Impulse...\n");
    ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=%lu, to=%lu.\n", 0, current_sample + headerOffset);
    ei_printf("[1/1] Uploading file to Edge Impulse OK (took 0 ms.)\n");

    is_uploaded = true;

    led = 0;

    ei_printf("OK\n");
}

static int insert_ref(char *buffer, int hdrLength)
{
    #define EXTRA_BYTES(a)  ((a & 0x3) ? 4 - (a & 0x3) : (a & 0x03))
    const char *ref = "Ref-BINARY-i16";
    int addLength = 0;
    int padding = EXTRA_BYTES(hdrLength);

    buffer[addLength++] = 0x60 + 14 + padding;
    for(int i = 0; i < strlen(ref); i++) {
        buffer[addLength++] = *(ref + i);
    }
    for(int i = 0; i < padding; i++) {
        buffer[addLength++] = ' ';
    }

    buffer[addLength++] = 0xFF;

    return addLength;
}

static bool create_header(void)
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();
    EiDeviceMemory *mem = dev->get_memory();
    sensor_aq_init_mbedtls_hs256_context(&ei_mic_signing_ctx, &ei_mic_hs_ctx, dev->get_sample_hmac_key().c_str());

    sensor_aq_payload_info payload = {
        dev->get_id_pointer(),
        dev->get_type_pointer(),
        1000.0f / static_cast<float>(audio_sampling_frequency),
        { { "audio", "wav" } }
    };

    int tr = sensor_aq_init(&ei_mic_ctx, &payload, NULL, true);

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


    int ref_size = insert_ref(((char*)ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix), end_of_header_ix);

    // and update the signature
    tr = ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, (uint8_t*)(ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix), ref_size);
    if (tr != 0) {
        ei_printf("Failed to update signature from header (%d)\n", tr);
        return false;
    }

    end_of_header_ix += ref_size;

    // Write to blockdevice
    tr = mem->write_sample_data((const uint8_t*)ei_mic_ctx.cbor_buffer.ptr, 0, end_of_header_ix);

    if (tr != end_of_header_ix) {
        ei_printf("Failed to write to header blockdevice\n");
        return false;
    }

    headerOffset = end_of_header_ix;

    return true;
}


/* Public functions -------------------------------------------------------- */

bool ei_microphone_record(uint32_t sample_length_ms, uint32_t start_delay_ms, bool print_start_messages)
{
    EiDeviceMemory *mem = EiDeviceInfo::get_device()->get_memory();

    if (print_start_messages) {
        ei_printf("Starting in %lu ms... (or until all flash was erased)\n", start_delay_ms < 2000 ? 2000 : start_delay_ms);
    }

    if(start_delay_ms < 2000) {
        ThisThread::sleep_for(2000 - start_delay_ms);
    }

    if(mem->erase_sample_data(0, (samples_required << 1) + 4096) != (samples_required << 1) + 4096) {
        return false;
    }

    create_header();

    queue_thread.start(callback(&mic_queue, &EventQueue::dispatch_forever));

    if (print_start_messages) {
        ei_printf("Sampling...\n");
    }

    return true;
}

bool ei_microphone_inference_start(uint32_t n_samples, float interval_ms)
{
    record_ready = false;

    inference.buffers[0] = (int16_t *)ei_malloc(n_samples * sizeof(int16_t));

    if(inference.buffers[0] == NULL) {
        return false;
    }

    inference.buffers[1] = (int16_t *)ei_malloc(n_samples * sizeof(int16_t));

    if(inference.buffers[1] == NULL) {
        ei_free(inference.buffers[0]);
        return false;
    }

    sampleBuffer = (int16_t *)ei_malloc((n_samples / 100) * sizeof(int16_t));

    if(sampleBuffer == NULL) {
        ei_free(inference.buffers[0]);
        ei_free(inference.buffers[1]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;

    // configure the data receive callback
    PDM.onReceive(&pdm_data_ready_inference_callback);

    PDM.setBufferSize((n_samples / 100) * sizeof(int16_t));

    // initialize PDM with:
    // - one channel (mono mode)
    // - a 16 kHz sample rate
    if (!PDM.begin(1, 16000)) {
        ei_printf("Failed to start PDM!");
    }

    /* Calclate sample rate from sample interval */
    audio_sampling_frequency = (uint32_t)(1000.f / interval_ms);

    if(audio_sampling_frequency != 16000) {
        nrf_pdm_clock_set((nrf_pdm_freq_t)pdm_clock_calculate(audio_sampling_frequency));
    }

    // set the gain, defaults to 20
    PDM.setGain(127);

    queue_thread.start(callback(&mic_queue, &EventQueue::dispatch_forever));

    record_ready = true;

    return true;

}

/**
 * @brief      Wait for a full buffer
 *
 * @return     In case of an buffer overrun return false
 */
bool ei_microphone_inference_record(void)
{
    bool ret = true;

    if (inference.buf_ready == 1) {
        ei_printf(
            "Error sample buffer overrun. Decrease the number of slices per model window "
            "(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)\n");
        ret = false;
    }

    while (inference.buf_ready == 0) {
        ThisThread::sleep_for(1);
    }

    inference.buf_ready = 0;

    return ret;
}

/**
 * @brief      Reset buffer counters for non-continuous inferecing
 */
void ei_microphone_inference_reset_buffers(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;
}

/**
 * Get raw audio signal data
 */
int ei_microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    return numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_microphone_init(void)
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();
    EiDeviceMemory *mem = dev->get_memory();
    
    PDM.setBufferSize(mem->block_size);

    return true;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_microphone_inference_end(void)
{
    record_ready = false;
    PDM.end();

    ei_free(inference.buffers[0]);
    ei_free(inference.buffers[1]);
    ei_free(sampleBuffer);
}

/**
 * Sample raw data
 */
bool ei_microphone_sample_start(void)
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();
    EiDeviceMemory *mem = dev->get_memory();
    // this sensor does not have settable interval...
    // ei_config_set_sample_interval(static_cast<float>(1000) / static_cast<float>(audio_sampling_frequency));
    int sample_length_blocks;

    record_ready = false;

    ei_printf("Sampling settings:\n");
    ei_printf("\tInterval: %.5f ms.\n", dev->get_sample_interval_ms());
    ei_printf("\tLength: %lu ms.\n", dev->get_sample_length_ms());
    ei_printf("\tName: %s\n", dev->get_sample_label().c_str());
    ei_printf("\tHMAC Key: %s\n", dev->get_sample_hmac_key().c_str());
    ei_printf("\tFile name: /fs/%s\n", dev->get_sample_label().c_str());

    samples_required = (uint32_t)((dev->get_sample_length_ms()) / dev->get_sample_interval_ms());

    /* Round to even number of samples for word align flash write */
    if(samples_required & 1) {
        samples_required++;
    }

    current_sample = 0;

    is_uploaded = false;

    sampleBuffer = (int16_t *)ei_malloc(mem->block_size);

    if (sampleBuffer == NULL) {
        return false;
    }

    // configure the data receive callback
    PDM.onReceive(&pdm_data_ready_callback);
    
    // initialize PDM with:
    // - one channel (mono mode)
    // - a 16 kHz sample rate
    if (!PDM.begin(1, 16000)) {
        ei_printf("Failed to start PDM!");
    }

    /* Calclate sample rate from sample interval */
    audio_sampling_frequency = (uint32_t)(1000.f / dev->get_sample_interval_ms());

    if(audio_sampling_frequency != 16000) {
        nrf_pdm_clock_set((nrf_pdm_freq_t)pdm_clock_calculate(audio_sampling_frequency));
    }

    // set the gain, defaults to 20
    PDM.setGain(127);

    bool r = ei_microphone_record(dev->get_sample_length_ms(), (((samples_required <<1)/ mem->block_size) * mem->block_erase_time), true);
    if (!r) {
        return r;
    }
    record_ready = true;


    while(record_ready == true) {
        ThisThread::sleep_for(100);
    };

    int ctx_err = ei_mic_ctx.signature_ctx->finish(ei_mic_ctx.signature_ctx, ei_mic_ctx.hash_buffer.buffer);
    if (ctx_err != 0) {
        ei_printf("Failed to finish signature (%d)\n", ctx_err);
        return false;
    }

    finish_and_upload();

    return true;
}
