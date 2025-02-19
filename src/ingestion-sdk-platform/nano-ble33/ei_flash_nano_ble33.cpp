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

#include "ei_flash_nano_ble33.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <cmath>

/** 32-bit align write buffer size */
#define WORD_ALIGN(a)	((a & 0x3) ? (a & ~0x3) + 0x4 : a)
/** Align addres to given sector size */
#define SECTOR_ALIGN(a, sec_size)	((a & (sec_size-1)) ? (a & ~(sec_size-1)) + sec_size : a)

uint32_t EiFlashMemory::read_data(uint8_t *data, uint32_t address, uint32_t num_bytes)
{
    int ret;

    ret = iap.read((void *)data, base_address + address, num_bytes);
    if (ret != 0) {
        ei_printf("ERR: Failed to read data! (%d)\n", ret);
        return 0;
    }

    return num_bytes;
}

uint32_t EiFlashMemory::write_data(const uint8_t *data, uint32_t address, uint32_t num_bytes)
{
    int ret;

    ret = iap.program(data, WORD_ALIGN(base_address + address), WORD_ALIGN(num_bytes));
    if (ret != 0) {
        ei_printf("ERR: Failed to write data! (%d)\n", ret);
        return 0;
    }

    return num_bytes;
}

uint32_t EiFlashMemory::erase_data(uint32_t address, uint32_t num_bytes)
{
    int ret;
    
    ret = iap.erase(base_address + address, SECTOR_ALIGN(num_bytes, block_size));

    if(ret != 0) {
        ei_printf("ERR: Failed to erase flash (%d)\n", ret);
        return 0;
    }

    return num_bytes;
}

EiFlashMemory::EiFlashMemory(uint32_t config_size):
    EiDeviceMemory(config_size, 90, 0, 0)
{
    iap.init();

	block_size = iap.get_sector_size(iap.get_flash_start() + iap.get_flash_size() - 1UL);
    base_address = SECTOR_ALIGN(FLASHIAP_APP_ROM_END_ADDR, block_size);
    memory_size = iap.get_flash_size() - base_address;
    used_blocks = (config_size < block_size) ? 1 : ceil(float(config_size) / block_size);
    memory_blocks = memory_size / block_size;
}
