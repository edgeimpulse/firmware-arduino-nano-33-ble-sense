/*
 * Copyright (c) 2023 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/* Include ----------------------------------------------------------------- */
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "ei_bm270_bmm150.h"

#include "mbed_events.h"
#include "mbed_shared_queues.h"
#include "drivers/InterruptIn.h"

ei_BoschSensorClass::ei_BoschSensorClass(TwoWire& wire)
{
  _wire = &wire;
  BMI270_INT1 = p11;
}

int ei_BoschSensorClass::begin() {

  _wire->begin();

  bmi2.chip_id = BMI2_I2C_PRIM_ADDR;
  bmi2.read = bmi2_i2c_read;
  bmi2.write = bmi2_i2c_write;
  bmi2.delay_us = bmi2_delay_us;
  bmi2.intf = BMI2_I2C_INTF;
  bmi2.intf_ptr = &accel_gyro_dev_info;
  bmi2.read_write_len = 30; // Limitation of the Wire library
  bmi2.config_file_ptr = NULL; // Use the default BMI270 config file

  accel_gyro_dev_info._wire = _wire;
  accel_gyro_dev_info.dev_addr = bmi2.chip_id;

  bmm1.chip_id = BMM150_DEFAULT_I2C_ADDRESS;
  bmm1.read = bmi2_i2c_read;
  bmm1.write = bmi2_i2c_write;
  bmm1.delay_us = bmi2_delay_us;
  bmm1.intf = BMM150_I2C_INTF;
  bmm1.intf_ptr = &mag_dev_info;

  mag_dev_info._wire = _wire;
  mag_dev_info.dev_addr = bmm1.chip_id;

  int8_t rslt = bmi270_init(&bmi2);
  print_rslt(rslt);

  rslt = configure_sensor(&bmi2);
  print_rslt(rslt);

  rslt = bmm150_init(&bmm1);
  print_rslt(rslt);

  rslt = configure_sensor(&bmm1);
  print_rslt(rslt);

  _initialized = true;

  return 1;
}

int8_t ei_BoschSensorClass::configure_sensor(struct bmi2_dev *dev)
{
  int8_t rslt;
  uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
  
  struct bmi2_int_pin_config int_pin_cfg;
  int_pin_cfg.pin_type = BMI2_INT1;
  int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
  int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
  int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
  int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
  int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

  struct bmi2_sens_config sens_cfg[2];
  sens_cfg[0].type = BMI2_ACCEL;
  sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
  sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_400HZ;
  sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
  sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_2G;

  sens_cfg[1].type = BMI2_GYRO;
  sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
  sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
  sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_400HZ;
  sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_250;
  sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_250;

  rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_sensor_enable(sens_list, 2, dev);
  if (rslt != BMI2_OK)
    return rslt;

  return rslt;
}

int8_t ei_BoschSensorClass::configure_sensor(struct bmm150_dev *dev)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;
    struct bmm150_settings settings;
    
    /* Set powermode as normal mode */
    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = bmm150_set_op_mode(&settings, dev);

    if (rslt == BMM150_OK)
    {
        /* Setting the preset mode as Low power mode
         * i.e. data rate = 10Hz, XY-rep = 1, Z-rep = 2
         */
        settings.preset_mode = BMM150_PRESETMODE_REGULAR;
        //rslt = bmm150_set_presetmode(&settings, dev);

        if (rslt == BMM150_OK)
        {
            /* Map the data interrupt pin */
            settings.int_settings.drdy_pin_en = 0x01;
            //rslt = bmm150_set_sensor_settings(BMM150_SEL_DRDY_PIN_EN, &settings, dev);
        }
    }
    return rslt;
}

int8_t ei_BoschSensorClass::bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  if ((reg_data == NULL) || (len == 0) || (len > 32)) {
    return -1;
  }
  uint8_t bytes_received;

  struct dev_info* dev_info = (struct dev_info*)intf_ptr;
  uint8_t dev_id = dev_info->dev_addr;

  dev_info->_wire->beginTransmission(dev_id);
  dev_info->_wire->write(reg_addr);
  if (dev_info->_wire->endTransmission() == 0) {
    bytes_received = dev_info->_wire->requestFrom(dev_id, len);
    // Optionally, throw an error if bytes_received != len
    for (uint16_t i = 0; i < bytes_received; i++)
    {
      reg_data[i] = dev_info->_wire->read();
    }
  } else {
    return -1;
  }

  return 0;
}

int8_t ei_BoschSensorClass::bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  if ((reg_data == NULL) || (len == 0) || (len > 32)) {
    return -1;
  }

  struct dev_info* dev_info = (struct dev_info*)intf_ptr;
  uint8_t dev_id = dev_info->dev_addr;
  dev_info->_wire->beginTransmission(dev_id);
  dev_info->_wire->write(reg_addr);
  for (uint16_t i = 0; i < len; i++)
  {
    dev_info->_wire->write(reg_data[i]);
  }
  if (dev_info->_wire->endTransmission() != 0) {
    return -1;
  }

  return 0;
}

void ei_BoschSensorClass::bmi2_delay_us(uint32_t period, void *intf_ptr)
{
  delayMicroseconds(period);
}

// range is +-2G, so conversion factor is (((1 << 15)/2.0f))
#define INT16_to_2G   (16384.0f)

// Accelerometer
int ei_BoschSensorClass::readAcceleration(float& x, float& y, float& z) {  
  struct bmi2_sens_data sensor_data;
  auto ret = bmi2_get_sensor_data(&sensor_data, &bmi2);
  x = -sensor_data.acc.y / INT16_to_2G;
  y = -sensor_data.acc.x / INT16_to_2G;
  z = sensor_data.acc.z / INT16_to_2G;
  return (ret == 0);
}

int ei_BoschSensorClass::accelerationAvailable() {
  uint16_t status;  
  bmi2_get_int_status(&status, &bmi2);
  int ret = ((status | _int_status) & BMI2_ACC_DRDY_INT_MASK);
  _int_status = status;
  _int_status &= ~BMI2_ACC_DRDY_INT_MASK;
  return ret;
}

// Gyroscope
int ei_BoschSensorClass::readGyroscope(float& x, float& y, float& z) {
  struct bmi2_sens_data sensor_data;
  auto ret = bmi2_get_sensor_data(&sensor_data, &bmi2);
  x = -sensor_data.gyr.y; // ? from lib ...
  y = -sensor_data.gyr.x;
  z = sensor_data.gyr.z;
  return (ret == 0);
}

int ei_BoschSensorClass::gyroscopeAvailable() {
  uint16_t status;
  bmi2_get_int_status(&status, &bmi2);
  int ret = ((status | _int_status) & BMI2_GYR_DRDY_INT_MASK);
  _int_status = status;
  _int_status &= ~BMI2_GYR_DRDY_INT_MASK;
  return ret;
}
// Magnetometer
int ei_BoschSensorClass::readMagneticField(float& x, float& y, float& z) {
  struct bmm150_mag_data mag_data;
  int const rc = bmm150_read_mag_data(&mag_data, &bmm1);
  x = mag_data.x;
  y = mag_data.y;
  z = mag_data.z;

  if (rc == BMM150_OK)
    return 1;
  else
    return 0;
}

int ei_BoschSensorClass::magneticFieldAvailable() {
  bmm150_get_interrupt_status(&bmm1);
  return bmm1.int_status & BMM150_INT_ASSERTED_DRDY;
}

static void panic_led_trap(void)
{
#if !defined(LED_BUILTIN)
  static int const LED_BUILTIN = 2;
#endif

  pinMode(LED_BUILTIN, OUTPUT);
  while (1)
  {
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }
}

void ei_BoschSensorClass::print_rslt(int8_t rslt)
{
  switch (rslt)
  {
    case BMI2_OK: return; /* Do nothing */ break;
    case BMI2_E_NULL_PTR:
      ei_printf("Error [%d] : Null pointer\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_COM_FAIL:
      ei_printf("Error [%d] : Communication failure\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_DEV_NOT_FOUND:
      ei_printf("Error [%d] : Device not found\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_OUT_OF_RANGE:
      ei_printf("Error [%d] : Out of range\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_ACC_INVALID_CFG:
      ei_printf("Error [%d] : Invalid accel configuration\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_GYRO_INVALID_CFG:
      ei_printf("Error [%d] : Invalid gyro configuration\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_ACC_GYR_INVALID_CFG:
      ei_printf("Error [%d] : Invalid accel/gyro configuration\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_INVALID_SENSOR:
      ei_printf("Error [%d] : Invalid sensor\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_CONFIG_LOAD:
      ei_printf("Error [%d] : Configuration loading error\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_INVALID_PAGE:
      ei_printf("Error [%d] : Invalid page \r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_INVALID_FEAT_BIT:
      ei_printf("Error [%d] : Invalid feature bit\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_INVALID_INT_PIN:
      ei_printf("Error [%d] : Invalid interrupt pin\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_SET_APS_FAIL:
      ei_printf("Error [%d] : Setting advanced power mode failed\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_AUX_INVALID_CFG:
      ei_printf("Error [%d] : Invalid auxiliary configuration\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_AUX_BUSY:
      ei_printf("Error [%d] : Auxiliary busy\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_SELF_TEST_FAIL:
      ei_printf("Error [%d] : Self test failed\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_REMAP_ERROR:
      ei_printf("Error [%d] : Remapping error\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
      ei_printf("Error [%d] : Gyro user gain update failed\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_SELF_TEST_NOT_DONE:
      ei_printf("Error [%d] : Self test not done\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_INVALID_INPUT:
      ei_printf("Error [%d] : Invalid input\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_INVALID_STATUS:
      ei_printf("Error [%d] : Invalid status\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_CRT_ERROR:
      ei_printf("Error [%d] : CRT error\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_ST_ALREADY_RUNNING:
      ei_printf("Error [%d] : Self test already running\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
      ei_printf("Error [%d] : CRT ready for DL fail abort\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_DL_ERROR:
      ei_printf("Error [%d] : DL error\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_PRECON_ERROR:
      ei_printf("Error [%d] : PRECON error\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_ABORT_ERROR:
      ei_printf("Error [%d] : Abort error\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_GYRO_SELF_TEST_ERROR:
      ei_printf("Error [%d] : Gyro self test error\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_GYRO_SELF_TEST_TIMEOUT:
      ei_printf("Error [%d] : Gyro self test timeout\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_WRITE_CYCLE_ONGOING:
      ei_printf("Error [%d] : Write cycle ongoing\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_WRITE_CYCLE_TIMEOUT:
      ei_printf("Error [%d] : Write cycle timeout\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_ST_NOT_RUNING:
      ei_printf("Error [%d] : Self test not running\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_DATA_RDY_INT_FAILED:
      ei_printf("Error [%d] : Data ready interrupt failed\r\n", rslt);
      panic_led_trap();
      break;
    case BMI2_E_INVALID_FOC_POSITION:
      ei_printf("Error [%d] : Invalid FOC position\r\n", rslt);
      panic_led_trap();
      break;
    default:
      ei_printf("Error [%d] : Unknown error code\r\n", rslt);
      panic_led_trap();
      break;
  }
}
#if 0
static events::EventQueue queue(10 * EVENTS_EVENT_SIZE);

void ei_BoschSensorClass::interrupt_handler()
{  
  if (_initialized && _cb) {
    queue.call(_cb);
  }
}


// additional
void ei_BoschSensorClass::onInterrupt(mbed::Callback<void(void)> cb)
{
  if (BMI270_INT1 == NC) {
    return;
  }
  static mbed::InterruptIn irq(BMI270_INT1, PullDown);
  static rtos::Thread event_t(osPriorityHigh, 768, nullptr, "events");
  _cb = cb;
  event_t.start(callback(&queue, &events::EventQueue::dispatch_forever));
  irq.rise(mbed::callback(this, &ei_BoschSensorClass::interrupt_handler));
}
#endif

ei_BoschSensorClass ei_IMU_BMI270_BMM150(Wire1);
