/* Edge Impulse 
 * Copyright (c) 2022 EdgeImpulse Inc.
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
#ifndef EI_LSM9DS1_H
#define EI_LSM9DS1_H

/* Include ----------------------------------------------------------------- */
#include <Arduino_LSM9DS1.h>

/* Class ----------------------------------------------------- */
class ei_LSM9DS1Class: public LSM9DS1Class
{
    public:
        ei_LSM9DS1Class(TwoWire& wire);
        int ei_begin(void);
        int readAcceleration(float& x, float& y, float& z); // overloading arduino basic read due to different range setting

    private:
        int writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value);
        int readRegister(uint8_t slaveAddress, uint8_t address);
        int readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t* data, size_t length);
    private:
        TwoWire* _wire;
};

extern ei_LSM9DS1Class ei_IMU;
#endif