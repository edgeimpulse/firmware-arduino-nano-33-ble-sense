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

#ifndef EI_CAMERA
#define EI_CAMERA

/* Include ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "ei_device_nano_ble33.h"
#include "at_base64_lib.h"
#include "../edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "../edge-impulse-sdk/dsp/numpy_types.h"

/* Constants --------------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           160
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           120

/* Public function prototypes ---------------------------------------------- */
extern bool ei_camera_init(void);
extern bool ei_has_camera(void);
extern void ei_camera_deinit(void);
extern bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *buf);
extern bool ei_camera_take_snapshot_encode_and_output(size_t width, size_t height, bool use_max_baudrate);
extern bool ei_camera_start_snapshot_stream_encode_and_output(size_t width, size_t height, bool use_max_baudrate);
extern void ei_printf(const char *format, ...);

/**
 * @brief      Determine whether to resize and to which dimension
 *
 * @param[in]  out_width     width of output image
 * @param[in]  out_height    height of output image
 * @param[out] resize_col_sz       pointer to frame buffer's column/width value
 * @param[out] resize_row_sz       pointer to frame buffer's rows/height value
 * @param[out] do_resize     returns whether to resize (or not)
 *
 */
int calculate_resize_dimensions(uint32_t out_width, uint32_t out_height, uint32_t *resize_col_sz, uint32_t *resize_row_sz, bool *do_resize);

/**
 * @brief      Retrieves (cut-out) float RGB image data from the frame buffer
 *
 * @param[in]  offset        offset within cut-out image
 * @param[in]  length        number of bytes to read
 * @param[int] out_ptr       pointer to output buffre
 *
 * @retval     0 if successful
 *
 * @note       This function is called by the classifier to get float RGB image data
 */
int ei_camera_cutout_get_data(size_t offset, size_t length, float *out_ptr);

/* Reference to object for external usage ---------------------------------- */
extern EiDeviceNanoBle33 EiDevice;

#include <Arduino_OV767X.h>
class OV7675 : public OV767X {
    public:
        int begin(int resolution, int format, int fps);
        void readFrame(void* buffer);

    private:
        int vsyncPin;
        int hrefPin;
        int pclkPin;
        int xclkPin;

        volatile uint32_t* vsyncPort;
        uint32_t vsyncMask;
        volatile uint32_t* hrefPort;
        uint32_t hrefMask;
        volatile uint32_t* pclkPort;
        uint32_t pclkMask;

        uint16_t width;
        uint16_t height;
        uint8_t bytes_per_pixel;
        uint16_t bytes_per_row;
        uint8_t buf_rows;
        uint16_t buf_size;
        uint8_t resize_height;
        uint8_t *raw_buf;
        void *buf_mem;
        uint8_t *intrp_buf;
        uint8_t *buf_limit;

        void readBuf();
        int allocate_scratch_buffs();
        int deallocate_scratch_buffs();
};

#endif // EI_CAMERA
