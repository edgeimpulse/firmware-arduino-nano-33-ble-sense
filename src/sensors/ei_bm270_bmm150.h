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