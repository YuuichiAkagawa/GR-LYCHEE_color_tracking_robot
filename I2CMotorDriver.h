/* 
 * A library for Grove - I2C Motor Driver for mbed
 * Copyright (c) 2017 Yuuichi Akagawa
 *
 * This software is released under the MIT License.
 */
/* 
 * Grove_I2C_Motor_Driver.h
 * A library for Grove - I2C Motor Driver v1.3
 *
 * Copyright (c) 2012 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : Jerry Yip
 * Create Time: 2017-02
 * Change Log :
 *
 * The MIT License (MIT)
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
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef __I2CMOTORDRIVER_H__
#define __I2CMOTORDRIVER_H__

#include "mbed.h"

/******I2C command definitions*************/
#define MotorSpeedSet             0x82
#define PWMFrequenceSet           0x84
#define DirectionSet              0xaa
#define MotorSetA                 0xa1
#define MotorSetB                 0xa5
#define Nothing                   0x01
/**************Motor ID**********************/
#define MOTOR1 			          1
#define MOTOR2 			          2
/**************Motor Direction***************/
#define BothClockWise             0x0a
#define BothAntiClockWise         0x05
#define M1CWM2ACW                 0x06
#define M1ACWM2CW                 0x09
/**************Motor Direction***************/
#define ClockWise                 0x0a
#define AntiClockWise             0x05
/**************Prescaler Frequence***********/
#define F_31372Hz                 0x01
#define F_3921Hz                  0x02
#define F_490Hz                   0x03
#define F_122Hz                   0x04
#define F_30Hz                    0x05

/*************************Class for Grove I2C Motor Driver********************/
class I2CMotorDriver
{

private:
    // _i2c_add: 0x00~0x0f
    unsigned char _i2c_add;
    // _speed0: 0~100  _speed1: 0~100
    unsigned char _speed1;
    unsigned char _speed2;
    // the direction of M1 and M2 DC motor 1:clockwise  -1:anticlockwise
    int _M1_direction;
    int _M2_direction;
    // Set the direction of both motors
    // _direction: BothClockWise, BothAntiClockWise, M1CWM2ACW, M1ACWM2CW
    void direction(unsigned char _direction);
    inline int map(int x, int in_min, int in_max, int out_min, int out_max){  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;};
private:
    I2C     i2c;
    char    i2c_addr;

public:
	I2CMotorDriver( PinName sda, PinName scl, char i2c_address = 0x0f);
    // Set the speed of a motor, speed is equal to duty cycle here
    // motor_id: MOTOR1, MOTOR2
    // _speed: -100~100, when _speed>0, dc motor runs clockwise; 
    // when _speed<0, dc motor runs anticlockwise
    void speed(unsigned char motor_id, int _speed);
    // Set the frequence of PWM(cycle length = 510, system clock = 16MHz)
    // F_3921Hz is default
    // _frequence: F_31372Hz, F_3921Hz, F_490Hz, F_122Hz, F_30Hz
    void frequence(unsigned char _frequence);
    // Stop one motor
    // motor_id: MOTOR1, MOTOR2
    void stop(unsigned char motor_id);
};
#endif
