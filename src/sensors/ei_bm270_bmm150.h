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
#ifndef EI_BMI270_BMM250_H
#define EI_BMI270_BMM250_H

/* Include ----------------------------------------------------------------- */
#include "BoschSensorClass.h"

#include <Arduino.h>
#include <Wire.h>
#include "utilities/BMI270-Sensor-API/bmi270.h"
#include "utilities/BMM150-Sensor-API/bmm150.h"

/* Class ----------------------------------------------------- */

class ei_BoschSensorClass : public BoschSensorClass {
  public:    
    ei_BoschSensorClass(TwoWire& wire = Wire);
    ~ei_BoschSensorClass() {}

    int begin();

    // Accelerometer
    int readAcceleration(float& x, float& y, float& z) override; // Results are in G (earth gravity).
    int accelerationAvailable() override; // Number of samples in the FIFO.    

    // Gyroscope
    int readGyroscope(float& x, float& y, float& z) override;  // Results are in degrees/second.
    int gyroscopeAvailable() override; // Number of samples in the FIFO.    

    // Magnetometer
    int readMagneticField(float& x, float& y, float& z) override; // Results are in uT (micro Tesla).
    int magneticFieldAvailable() override; // Number of samples in the FIFO.

  protected:    
    int8_t configure_sensor(struct bmm150_dev *dev) override;
    int8_t configure_sensor(struct bmi2_dev *dev) override;

  private:
    static int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static void bmi2_delay_us(uint32_t period, void *intf_ptr);
    void interrupt_handler();
    void print_rslt(int8_t rslt);

  private:
    TwoWire* _wire;
    Stream* _debug = nullptr;
    mbed::Callback<void(void)> _cb;
    bool _initialized = false;
    struct dev_info accel_gyro_dev_info;
    struct dev_info mag_dev_info;
    struct bmi2_dev bmi2;
    struct bmm150_dev bmm1;
    uint16_t _int_status;
  private:
    bool continuousMode;
};

extern ei_BoschSensorClass ei_IMU_BMI270_BMM150;
//#undef IMU
//#define IMU IMU_BMI270_BMM150
#endif