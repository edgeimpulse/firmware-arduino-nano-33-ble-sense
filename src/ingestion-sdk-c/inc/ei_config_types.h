#ifndef _EI_CONFIG_TYPES_H
#define _EI_CONFIG_TYPES_H

// return types
typedef enum {
    EI_CONFIG_OK = 0,
    EI_CONFIG_BOUNDS_ERROR = -1,
    EI_CONFIG_CONTEXT_ERROR = -2,
    EI_CONFIG_NO_CONTEXT = -8,
    EI_CONFIG_WIFI_CONN_FAILED = -9,
} EI_CONFIG_ERROR;

// same as https://github.com/ARMmbed/mbed-os/blob/d4122b0b3a091d3f1130f3bb0d91c96f9183e2de/features/netsocket/nsapi_types.h#L115
typedef enum {
    EI_SECURITY_NONE         = 0x0,      /*!< open access point */
    EI_SECURITY_WEP          = 0x1,      /*!< phrase conforms to WEP */
    EI_SECURITY_WPA          = 0x2,      /*!< phrase conforms to WPA */
    EI_SECURITY_WPA2         = 0x3,      /*!< phrase conforms to WPA2 */
    EI_SECURITY_WPA_WPA2     = 0x4,      /*!< phrase conforms to WPA/WPA2 */
    EI_SECURITY_PAP          = 0x5,      /*!< phrase conforms to PPP authentication context */
    EI_SECURITY_CHAP         = 0x6,      /*!< phrase conforms to PPP authentication context */
    EI_SECURITY_EAP_TLS      = 0x7,      /*!< phrase conforms to EAP-TLS */
    EI_SECURITY_PEAP         = 0x8,      /*!< phrase conforms to PEAP */
    EI_SECURITY_UNKNOWN      = 0xFF,     /*!< unknown/unsupported security in scan results */
} ei_config_security_t;


// All the possible configuration options we can set
typedef struct {
    char wifi_ssid[128];
    char wifi_password[128];
    ei_config_security_t wifi_security;
    float sample_interval_ms;
    uint32_t sample_length_ms;
    char sample_label[128];
    char sample_hmac_key[33];
    char upload_host[128];
    char upload_path[128];
    char upload_api_key[128];
    char mgmt_url[128];
    uint32_t magic;
} ei_config_t;

#endif