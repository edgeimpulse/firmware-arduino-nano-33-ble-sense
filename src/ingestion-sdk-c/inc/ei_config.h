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

#ifndef _EDGE_IMPULSE_CONFIG_H_
#define _EDGE_IMPULSE_CONFIG_H_

#include <stdint.h>
#include "ei_config_types.h"

#define EDGE_IMPULSE_MAX_FREQUENCIES        5



// Available sensors to sample from on this board
typedef struct {
    // Name (e.g. 'Built-in accelerometer')
    const char *name;
    // Frequency list
    float frequencies[EDGE_IMPULSE_MAX_FREQUENCIES];
    // Max. sample length in seconds (could be depending on the size of the flash chip)
    uint16_t max_sample_length_s;
    // Start sampling, this function should be blocking and is called when sampling commences
// #ifdef __MBED__
//     Callback<bool()> start_sampling_cb;
// #else
    bool (*start_sampling_cb)();
//#endif
} ei_sensor_t;

// Configuration should be writable and readable, so let's declare some pointers for that
typedef struct {
    // Load the configuration from non-volatile storage
    // Store the data into the ei_config_t structure (already allocated)
    // return 0 if everything is OK
///    int (*load_config)(ei_config_t*);
    int (*load_config)(uint32_t *config, uint32_t config_size);

    // Save configuration to non-volatile storage
    // return 0 if everything is OK
//    int (*save_config)(const ei_config_t*);
    int (*save_config)(const uint32_t *config, uint32_t config_size);

    // Get unique device identifier
    // copy the ID into the provided buffer, and set the out_size
    // return 0 if OK, or other value if device does not have an ID
    int (*get_device_id)(uint8_t out_buffer[32], size_t *out_size);

    // Get device type
    // copy the type into the provided buffer, and set the out_size
    // return 0 if OK, or other value if device does not have an ID
    int (*get_device_type)(uint8_t out_buffer[32], size_t *out_size);

    // Return whether the network is connected
    bool (*wifi_connection_status)(void);

    // Return whether the board has wifi capabilities
    bool (*wifi_present)(void);

    // Function that will be called when WiFi settings are changed
    // use this to re-connect to the network
    // return true if network successfully connected
    bool (*wifi_settings_changed)();

    // List files function, invoke the function pointer for every file
    // you have on the file system
#ifdef __MBED__
    void (*list_files)(Callback<void(char*)> data_fn);
#else
    void (*list_files)(void(*data_fn)(char*));
#endif

    // Read a single file, invoke the function pointer with a block
    // of memory for each section of the file you're reading
    // Return false if the file does not exist
#ifdef __MBED__
    bool (*read_file)(const char *path, Callback<void(uint8_t*, size_t)> data_fn);
#else
    bool (*read_file)(const char *path, void(*data_fn)(uint8_t*, size_t));
#endif

    // Read from buffer, invoke the function pointer with a block
    // of memory for each section of the buffer you're reading
    // Return false if the file does not exist
#ifdef __MBED__
    bool (*read_buffer)(size_t start, size_t length, Callback<void(uint8_t*, size_t)> data_fn);
#else
    bool (*read_buffer)(size_t start, size_t length, void(*data_fn)(uint8_t*, size_t));
#endif    

    /**
     * Unlink a single file from the file system
     */
    bool (*unlink_file)(const char *path);

    // Scan for WiFi networks
    // Invoke the function pointer with each network you see
    // Return false if an error occured during scanning
#ifdef __MBED__
    bool (*scan_wifi)(Callback<void(const char*, ei_config_security_t, int8_t)> data_fn);
#else
    bool (*scan_wifi)(void(*data_fn)(const char *ssid, ei_config_security_t security, int8_t rssi));
#endif

    // Upload a file, return false if an error occured
//    bool (*upload_file)(EI_SENSOR_AQ_STREAM *file, const char *filename);

    // Whether the device is connected to the remote management interface
    bool (*mgmt_is_connected)();

    // Get the last error from the remote management interface (if applicable)
    bool (*mgmt_get_last_error)(char *error, size_t error_size);

} ei_config_ctx_t;

// Only single context has to be active, so store this here
static ei_config_ctx_t *ei_config_ctx = NULL;
static ei_config_t ei_config = { 0 };

/**
 * Initialize the configuration store
 * @param ctx A structure with load / save functions for the configuration
 */
EI_CONFIG_ERROR ei_config_init(ei_config_ctx_t *ctx) {
    ei_config_ctx = ctx;

    // directly after we'll load the configuration
    int r = ei_config_ctx->load_config != NULL ? ei_config_ctx->load_config((uint32_t *)&ei_config, sizeof(ei_config)) : -1;
    if (r != 0) {
        // @todo: how to get error data from the context back to user without violating the return type?
        return EI_CONFIG_CONTEXT_ERROR;
    }

    // check if magic marker is set (valid file)
    if (ei_config.magic != 0xDEADBEEF) {
        // invalid, reset content...
        memset(&ei_config, 0, sizeof(ei_config_t));

        ei_config.sample_interval_ms = 10.0f;
        ei_config.sample_length_ms = 10000;
        // HMAC likes having a key set, API key doesn't have to be set at this point yet
        const char *setme = "please-set-me";
        memcpy(ei_config.sample_hmac_key, setme, strlen(setme) + 1);
        const char *test = "test";
        memcpy(ei_config.sample_label, test, strlen(test) + 1);
    }

    ei_config.magic = 0xDEADBEEF;

    return EI_CONFIG_OK;
}

/**
 * Save the configuration (not to be called externally)
 */
static EI_CONFIG_ERROR ei_config_save() {
    if (ei_config_ctx == NULL) return EI_CONFIG_NO_CONTEXT;

    int r = ei_config_ctx->save_config((const uint32_t*)&ei_config, sizeof(ei_config));

    if (r != 0) {
        // @todo: how to get error data from the context back to user without violating the return type?
        return EI_CONFIG_CONTEXT_ERROR;
    }

    return EI_CONFIG_OK;
}

/**
 * Set WiFi credentials
 * @param ssid SSID
 * @param password Password
 * @param security Security level
 */
EI_CONFIG_ERROR ei_config_set_wifi(const char *ssid, const char *password, ei_config_security_t security) {
    if (strlen(ssid) > 127) return EI_CONFIG_BOUNDS_ERROR;
    if (strlen(password) > 127) return EI_CONFIG_BOUNDS_ERROR;

    memcpy(ei_config.wifi_ssid, ssid, strlen(ssid) + 1);
    memcpy(ei_config.wifi_password, password, strlen(password) + 1);
    ei_config.wifi_security = security;

    EI_CONFIG_ERROR r = ei_config_save();

    if (ei_config_ctx->wifi_settings_changed) {
        if (ei_config_ctx->wifi_settings_changed()) {
            return r;
        }
        else {
            return EI_CONFIG_WIFI_CONN_FAILED;
        }
    }

    return r;
}

/**
 * Get WiFi credentials
 * @param ssid Out parameter
 * @param password Out parameter
 * @param security Out parameter
 */
EI_CONFIG_ERROR ei_config_get_wifi(char ** ssid, char ** password, ei_config_security_t *security, bool *connected, bool *present) {
    if (ei_config_ctx == NULL) return EI_CONFIG_NO_CONTEXT;

    *ssid = ei_config.wifi_ssid;
    *password = ei_config.wifi_password;
    *security = ei_config.wifi_security;
    if (ei_config_ctx->wifi_connection_status) {
        *connected = ei_config_ctx->wifi_connection_status();
    }
    else {
        *connected = false;
    }
    
    if(ei_config_ctx->wifi_present) {
        *present = ei_config_ctx->wifi_present();
    }
    else {
        *present = false;
    }

    return EI_CONFIG_OK;
}

/**
 * Whether WiFi credentials are set
 */
bool ei_config_has_wifi() {
    // check whether we have an SSID
    return strcmp(ei_config.wifi_ssid, "") != 0;
}

/**
 * Get unique device identifier
 */
EI_CONFIG_ERROR ei_config_get_device_id(uint8_t out_buffer[32], size_t *out_size) {
    if (ei_config_ctx == NULL) return EI_CONFIG_NO_CONTEXT;

    int v = ei_config_ctx->get_device_id(out_buffer, out_size);
    if (v != 0) {
        return EI_CONFIG_CONTEXT_ERROR;
    }
    return EI_CONFIG_OK;
}

/**
 * Set sampling settings
 * @param label The label to attach to the settings
 * @param interval How often sampling needs to be done (in ms.)
 * @param length Total length of the sample (in ms.)
 */
EI_CONFIG_ERROR ei_config_set_sample_settings(const char *label, float interval, uint32_t length) {
    if (strlen(label) > 127) return EI_CONFIG_BOUNDS_ERROR;

    memcpy(ei_config.sample_label, label, strlen(label) + 1);
    ei_config.sample_interval_ms = interval;
    ei_config.sample_length_ms = length;

    return ei_config_save();
}

/**
 * Set sampling interval
 * @param interval How often sampling needs to be done (in ms.)
 */
EI_CONFIG_ERROR ei_config_set_sample_interval(float interval) {
    ei_config.sample_interval_ms = interval;

    return ei_config_save();
}

/**
 * Set sampling settings
 * @param label The label to attach to the settings
 * @param interval How often sampling needs to be done (in ms.)
 * @param length Total length of the sample (in ms.)
 * @param hmac_key Signature
 */
EI_CONFIG_ERROR ei_config_set_sample_settings(const char *label, float interval, uint32_t length, char *hmac_key) {
    if (strlen(label) > 127) return EI_CONFIG_BOUNDS_ERROR;
    if (strlen(hmac_key) > 32) return EI_CONFIG_BOUNDS_ERROR;

    memcpy(ei_config.sample_label, label, strlen(label) + 1);
    ei_config.sample_interval_ms = interval;
    ei_config.sample_length_ms = length;
    memcpy(ei_config.sample_hmac_key, hmac_key, strlen(hmac_key) + 1);

    return ei_config_save();
}

/**
 * Get sample settings
 * @param label Out parameter
 * @param password Out parameter
 * @param security Out parameter
 */
EI_CONFIG_ERROR ei_config_get_sample_settings(char ** label, float *interval, uint32_t *length, char ** hmac_key) {
    if (ei_config_ctx == NULL) return EI_CONFIG_NO_CONTEXT;

    *label = ei_config.sample_label;
    *interval = ei_config.sample_interval_ms;
    *length = ei_config.sample_length_ms;
    *hmac_key = ei_config.sample_hmac_key;

    return EI_CONFIG_OK;
}

/**
 * Set upload settings
 * @param upload_path Parameter
 */
EI_CONFIG_ERROR ei_config_set_upload_path_settings(const char *upload_path) {
    if (strlen(upload_path) > 127) return EI_CONFIG_BOUNDS_ERROR;

    memcpy(ei_config.upload_path, upload_path, strlen(upload_path) + 1);

    return ei_config_save();
}

/**
 * Get upload settings
 * @param api_key Out parameter
 * @param upload_host Out parameter
 */
EI_CONFIG_ERROR ei_config_set_upload_host_settings(const char *api_key, const char *upload_host) {
    if (strlen(api_key) > 127) return EI_CONFIG_BOUNDS_ERROR;
    if (strlen(upload_host) > 127) return EI_CONFIG_BOUNDS_ERROR;

    memcpy(ei_config.upload_api_key, api_key, strlen(api_key) + 1);
    memcpy(ei_config.upload_host, upload_host, strlen(upload_host) + 1);

    return ei_config_save();
}

/**
 * Set upload settings
 * @param upload_host Parameter
 */
EI_CONFIG_ERROR ei_config_set_upload_host_settings(const char *upload_host) {
    if (strlen(upload_host) > 127) return EI_CONFIG_BOUNDS_ERROR;

    memcpy(ei_config.upload_host, upload_host, strlen(upload_host) + 1);

    return ei_config_save();
}

/**
 * Get upload settings
 * @param api_key Out parameter
 * @param upload_path Out parameter
 */
EI_CONFIG_ERROR ei_config_set_upload_path_settings(const char *api_key, const char *upload_path) {
    if (strlen(api_key) > 127) return EI_CONFIG_BOUNDS_ERROR;
    if (strlen(upload_path) > 127) return EI_CONFIG_BOUNDS_ERROR;

    memcpy(ei_config.upload_api_key, api_key, strlen(api_key) + 1);
    memcpy(ei_config.upload_path, upload_path, strlen(upload_path) + 1);

    return ei_config_save();
}

/**
 * Get upload settings
 * @param api_key Out parameter
 * @param upload_host Out parameter
 * @param upload_path Out parameter
 */
EI_CONFIG_ERROR ei_config_get_upload_settings(char ** api_key, char ** upload_host, char ** upload_path) {
    if (ei_config_ctx == NULL) return EI_CONFIG_NO_CONTEXT;

    *api_key = ei_config.upload_api_key;
    *upload_host = ei_config.upload_host;
    *upload_path = ei_config.upload_path;

    return EI_CONFIG_OK;
}

/**
 * Set management settings
 * @param mgmt_url Management interface URL
 */
EI_CONFIG_ERROR ei_config_set_mgmt_settings(const char *mgmt_url) {
    if (strlen(mgmt_url) > 127) return EI_CONFIG_BOUNDS_ERROR;

    memcpy(ei_config.mgmt_url, mgmt_url, strlen(mgmt_url) + 1);

    return ei_config_save();
}

/**
 * Get management settings
 * @param mgmt_url Out param for Management interface URL
 */
EI_CONFIG_ERROR ei_config_get_mgmt_settings(char ** mgmt_url, bool *is_connected, char *last_error, size_t last_error_size) {
    if (ei_config_ctx == NULL) return EI_CONFIG_NO_CONTEXT;

    *mgmt_url = ei_config.mgmt_url;
    if (ei_config_ctx->mgmt_is_connected) {
        *is_connected = ei_config_ctx->mgmt_is_connected();
    }
    else {
        *is_connected = false;
    }
    if (ei_config_ctx->mgmt_get_last_error) {
        // no last error set to 0
        if (!ei_config_ctx->mgmt_get_last_error(last_error, last_error_size)) {
            memset(last_error, 0, last_error_size);
        }
    }

    return EI_CONFIG_OK;
}

EI_CONFIG_ERROR ei_config_clear() {
    memset(&ei_config, 0, sizeof(ei_config_t));
    return ei_config_save();
}

ei_config_t *ei_config_get_config() {
    return &ei_config;
}

ei_config_ctx_t *ei_config_get_context() {
    return ei_config_ctx;
}

#endif // _EDGE_IMPULSE_CONFIG_H_
