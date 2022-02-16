#ifndef ENCODE_AS_JPG_H_
#define ENCODE_AS_JPG_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "JPEGENC.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "edge-impulse-sdk/dsp/numpy_types.h"

int encode_as_jpg(uint8_t *framebuffer, size_t framebuffer_size, int width, int height, uint8_t *out_buffer, size_t out_buffer_size, size_t *out_size) {
    static JPEGClass jpg;
    JPEGENCODE jpe;

    int rc = jpg.open(out_buffer, out_buffer_size);
    if (rc != JPEG_SUCCESS) {
        return rc;
    }

    rc = jpg.encodeBegin(&jpe, width, height, JPEG_PIXEL_GRAYSCALE, JPEG_SUBSAMPLE_444, JPEG_Q_BEST);
    if (rc != JPEG_SUCCESS) {
        return rc;
    }

    int imcuCount = ((width + jpe.cx-1)/ jpe.cx) * ((height + jpe.cy-1) / jpe.cy);

    int bytePp = 1;
    int pitch = bytePp * width;

    for (int i=0; i < imcuCount && rc == JPEG_SUCCESS; i++) {
        // pass a pointer to the upper left corner of each MCU
        // the JPEGENCODE structure is updated by addMCU() after
        // each call
        rc = jpg.addMCU(&jpe, &framebuffer[(jpe.x * bytePp) + (jpe.y * pitch)], pitch);
        if (rc != JPEG_SUCCESS) {
            return rc;
        }
    }

    *out_size = jpg.close();

    return 0;
}

int encode_bw_signal_as_jpg(signal_t *signal, int width, int height, uint8_t *out_buffer, size_t out_buffer_size, size_t *out_size) {
    static JPEGClass jpg;
    JPEGENCODE jpe;
    float *encode_buffer = NULL;
    uint8_t *encode_buffer_u8 = NULL;

    int rc = jpg.open(out_buffer, out_buffer_size);
    if (rc != JPEG_SUCCESS) {
        return rc;
    }

    rc = jpg.encodeBegin(&jpe, width, height, JPEG_PIXEL_GRAYSCALE, JPEG_SUBSAMPLE_444, JPEG_Q_BEST);
    if (rc != JPEG_SUCCESS) {
        return rc;
    }

    int imcu_count = ((width + jpe.cx-1)/ jpe.cx) * ((height + jpe.cy-1) / jpe.cy);

    int bytePp = 1;
    int pitch = bytePp * width;

    // We read through the signal paged...
    int buf_len = width * 8;

    int last_offset = 0;
    int max_offset_diff = 0;

    encode_buffer = (float*)malloc(buf_len * 4);
    if (!encode_buffer) {
        rc = JPEG_MEM_ERROR;
        goto cleanup;
    }
    encode_buffer_u8 = (uint8_t*)malloc(buf_len);
    if (!encode_buffer_u8) {
        rc = JPEG_MEM_ERROR;
        goto cleanup;
    }

    for (int i = 0; i < imcu_count; i++) {
        // pass a pointer to the upper left corner of each MCU
        // the JPEGENCODE structure is updated by addMCU() after
        // each call

        int offset = (jpe.x * bytePp) + (jpe.y * pitch);
        rc = signal->get_data(offset, buf_len, encode_buffer);
        if (rc != 0) {
            goto cleanup;
        }

        for (size_t ix = 0; ix < buf_len; ix++) {
            encode_buffer_u8[ix] = static_cast<uint32_t>(encode_buffer[ix]) & 0xff;
        }

        rc = jpg.addMCU(&jpe, encode_buffer_u8, pitch);
        if (rc != JPEG_SUCCESS) {
            goto cleanup;
        }

        if (offset - last_offset > max_offset_diff) {
            max_offset_diff = offset - last_offset;
        }

        last_offset = offset;
    }

    printf("Max_offset_diff %d\n", max_offset_diff);

    rc = JPEG_SUCCESS;

cleanup:
    *out_size = jpg.close();

    free(encode_buffer);
    free(encode_buffer_u8);

    return rc;
}

#endif // ENCODE_AS_JPG_H
