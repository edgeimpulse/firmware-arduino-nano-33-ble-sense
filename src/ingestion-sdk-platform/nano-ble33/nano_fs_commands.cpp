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
#include "nano_fs_commands.h"

#include "mbed.h"
#include "FlashIAP.h"

/* Private types & constants ---------------------------------------------- */

/**
 * File system config struct.
 * @details Holds all the info needed for config file and sample data.<br>
 * - The config file is stored in the last available sector<br>
 * - The sample data starts at the end of the program data and ends before the
 * config file
 */
typedef struct
{
	uint32_t sector_size;					/*!< Erase sector size 			 */
	uint32_t page_size;						/*!< Minimum page write size 	 */
	uint32_t config_file_address;			/*!< Start address of config file*/
	uint32_t sample_start_address;			/*!< Start of sample storage mem */
	bool     fs_init;						/*!< FS is successfully init  	 */

}ei_nano_fs_t;

/** 32-bit align write buffer size */
#define WORD_ALIGN(a)	((a & 0x3) ? (a & ~0x3) + 0x4 : a)
/** Align addres to given sector size */
#define SECTOR_ALIGN(a, sec_size)	((a & (sec_size-1)) ? (a & ~(sec_size-1)) + sec_size : a)

/* Private variables ------------------------------------------------------- */
static mbed::FlashIAP iap;
static ei_nano_fs_t nano_fs = {0};

/* Public functions -------------------------------------------------------- */

/**
 * @brief      Init Flash pheripheral for reading & writing and set all
 * 			   parameters for the file system.
 * @return     true if succesful else false
 */
bool ei_nano_fs_init(void)
{
	iap.init();

	/* Setup addresses for fs */
	nano_fs.sector_size = iap.get_sector_size(iap.get_flash_start() + iap.get_flash_size() - 1UL);
    nano_fs.page_size = iap.get_page_size();
    nano_fs.config_file_address = (iap.get_flash_start() + iap.get_flash_size()) - (nano_fs.sector_size);
    nano_fs.sample_start_address = SECTOR_ALIGN(FLASHIAP_APP_ROM_END_ADDR, nano_fs.sector_size);

    // ei_printf("Start config: %d start sample: %d size sample: %d\r\n", nano_fs.config_file_address,
    // 	nano_fs.sample_start_address, nano_fs.config_file_address - nano_fs.sample_start_address);

	/* Check correct init of all parameters */
	if((nano_fs.sector_size == 0) || (nano_fs.page_size == 0)
		|| (nano_fs.config_file_address == 0) || (nano_fs.sample_start_address == 0)) {
		nano_fs.fs_init = false;
	}
	else {
		nano_fs.fs_init = true;
	}

	return nano_fs.fs_init;
}

/**
 * @brief      Copy configuration data to config pointer
 *
 * @param      config       Destination pointer for config
 * @param[in]  config_size  Size of configuration in bytes
 *
 * @return     ei_nano_ret_t enum
 */
int ei_nano_fs_load_config(uint32_t *config, uint32_t config_size)
{
	ei_nano_ret_t ret;

	if(config == NULL) {
		ret = NANO_FS_CMD_NULL_POINTER;
	}

	else if(nano_fs.fs_init == true) {

		ret = (iap.read((void *)config, nano_fs.config_file_address, config_size) == 0)
			? NANO_FS_CMD_OK
			: NANO_FS_CMD_READ_ERROR;
	}
	else {
		ret = NANO_FS_CMD_NOT_INIT;
	}

	return (int)ret;
}

/**
 * @brief      Write config to Flash
 *
 * @param[in]  config       Pointer to configuration data
 * @param[in]  config_size  Size of configuration in bytes
 *
 * @return     ei_nano_ret_t enum
 */
int ei_nano_fs_save_config(const uint32_t *config, uint32_t config_size)
{
	ei_nano_ret_t ret;

	if(config == NULL) {
		ret = NANO_FS_CMD_NULL_POINTER;
	}

	else if(nano_fs.fs_init == true) {

		ret = (iap.erase(nano_fs.config_file_address, nano_fs.sector_size) == 0)
			? NANO_FS_CMD_OK
			: NANO_FS_CMD_ERASE_ERROR;

		if(ret == NANO_FS_CMD_OK) {

			ret = (iap.program((const void *)config, nano_fs.config_file_address, WORD_ALIGN(config_size)) == 0)
				? NANO_FS_CMD_OK
				: NANO_FS_CMD_WRITE_ERROR;
		}
		else {
			ret = NANO_FS_CMD_ERASE_ERROR;
		}
	}
	else {
		ret = NANO_FS_CMD_NOT_INIT;
	}

	return (int)ret;
}

int ei_nano_fs_prepare_sampling(void)
{
	ei_nano_ret_t ret;

	if(nano_fs.fs_init == true) {
		ret = (iap.erase(nano_fs.sample_start_address, nano_fs.config_file_address - nano_fs.sample_start_address) == 0)
			? NANO_FS_CMD_OK
			: NANO_FS_CMD_ERASE_ERROR;
	}
	else {
		ret = NANO_FS_CMD_NOT_INIT;
	}

	return ret;
}

int ei_nano_fs_erase_sampledata(uint32_t start_block, uint32_t end_address)
{
	ei_nano_ret_t ret;

	if(nano_fs.fs_init == true) {
		ret = (iap.erase(nano_fs.sample_start_address, SECTOR_ALIGN(end_address, nano_fs.sector_size)) == 0)
			? NANO_FS_CMD_OK
			: NANO_FS_CMD_ERASE_ERROR;
	}
	else {
		ret = NANO_FS_CMD_NOT_INIT;
	}

	return ret;
}

int ei_nano_fs_write_sample_block(const void *sample_buffer, uint32_t address_offset)
{
	ei_nano_ret_t ret;

	if(sample_buffer == NULL) {
		ret = NANO_FS_CMD_NULL_POINTER;
	}
	else if (nano_fs.fs_init == true) {

		ret = (iap.program(sample_buffer, nano_fs.sample_start_address + address_offset, nano_fs.sector_size) == 0)
			? NANO_FS_CMD_OK
			: NANO_FS_CMD_WRITE_ERROR;
	}
	else {
		ret = NANO_FS_CMD_NOT_INIT;
	}

	return ret;
}

int ei_nano_fs_write_samples(const void *sample_buffer, uint32_t address_offset, uint32_t n_samples)
{
	ei_nano_ret_t ret;

	if(sample_buffer == NULL) {
		ret = NANO_FS_CMD_NULL_POINTER;
	}
	else if (nano_fs.fs_init == true) {

		ret = (iap.program(sample_buffer, WORD_ALIGN(nano_fs.sample_start_address + address_offset), WORD_ALIGN(n_samples)) == 0)
			? NANO_FS_CMD_OK
			: NANO_FS_CMD_WRITE_ERROR;
	}
	else {
		ret = NANO_FS_CMD_NOT_INIT;
	}

	return ret;
}

int ei_nano_fs_read_sample_data(void *sample_buffer, uint32_t address_offset, uint32_t n_read_bytes)
{
	ei_nano_ret_t ret;

	if(sample_buffer == NULL) {
		ret = NANO_FS_CMD_NULL_POINTER;
	}

	else if(nano_fs.fs_init == true) {

		ret = (iap.read((void *)sample_buffer, nano_fs.sample_start_address + address_offset, n_read_bytes) == 0)
			? NANO_FS_CMD_OK
			: NANO_FS_CMD_READ_ERROR;
	}
	else {
		ret = NANO_FS_CMD_NOT_INIT;
	}

	return (int)ret;
}

uint32_t ei_nano_fs_get_block_size(void)
{
	uint32_t block_size = 0;

	if(nano_fs.fs_init == true) {
		block_size = nano_fs.sector_size;
	}

	return block_size;
}

uint32_t ei_nano_fs_get_n_available_sample_blocks(void)
{
	uint32_t n_sample_blocks = 0;
	if(nano_fs.fs_init == true) {
		n_sample_blocks = (nano_fs.config_file_address - nano_fs.sample_start_address) / nano_fs.sector_size;
	}
	return n_sample_blocks;
}
