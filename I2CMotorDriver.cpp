/* 
 * A library for Grove - I2C Motor Driver for mbed
 * Copyright (c) 2017 Yuuichi Akagawa
 *
 * This software is released under the MIT License.
 */
/*
 * I2CMotorDriver.cpp
 * A library for Grove - I2C Motor Driver v1.3
 *
 * Copyright (c) 2012 seeed technology inc.
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

#include "mbed.h"
#include "I2CMotorDriver.h"


I2CMotorDriver::I2CMotorDriver( PinName sda, PinName scl, char i2c_address ) : i2c( sda, scl ), i2c_addr( i2c_address )
{
	i2c_addr <<= 1;
	_speed1 = 0;
	_speed2 = 0;
	_M1_direction = 1;
	_M2_direction = 1;
	wait_ms(2000);
	i2c.frequency( 400 * 1000 );
	// Set default frequence to F_3921Hz
	frequence(F_3921Hz);
}

// *****************************Private Function*******************************
// Set the direction of 2 motors
// _direction: M1CWM2ACW(M1 ClockWise M2 AntiClockWise), M1ACWM2CW, BothClockWise, BothAntiClockWise, 
void I2CMotorDriver::direction(unsigned char _direction)
{
	char cmd[3];
    cmd[0] = DirectionSet;	// Direction control header
    cmd[1] = _direction;  	// send direction control information
    cmd[2] = Nothing;     	// need to send this byte as the third byte(no meaning)
    i2c.write( i2c_addr, cmd, 3 );
  	wait_ms(4);					// wait
}

// *****************************DC Motor Function******************************
// Set the speed of a motor, speed is equal to duty cycle here
// motor_id: MOTOR1, MOTOR2
// _speed: -100~100, when _speed>0, dc motor runs clockwise; when _speed<0, 
// dc motor runs anticlockwise
void I2CMotorDriver::speed(unsigned char motor_id, int _speed)
{
	if(motor_id<MOTOR1 || motor_id>MOTOR2) {
		printf("Motor id error! Must be MOTOR1 or MOTOR2\n");
		return;
	}

	if(motor_id == MOTOR1) {
		if (_speed >= 0) {
			this->_M1_direction = 1; 
			_speed = _speed > 100 ? 100 : _speed;
			this->_speed1 = map(_speed, 0, 100, 0, 255);
		}
		else if (_speed < 0) {
			this->_M1_direction = -1;
			_speed = _speed < -100 ? 100 : -(_speed);
			this->_speed1 = map(_speed, 0, 100, 0, 255);
		}
	}
	else if(motor_id == MOTOR2) {
		if (_speed >= 0) {
			this->_M2_direction = 1;
			_speed = _speed > 100 ? 100 : _speed;
			this->_speed2 = map(_speed, 0, 100, 0, 255);
		}
		else if (_speed < 0) {
			this->_M2_direction = -1;
			_speed = _speed < -100 ? 100 : -(_speed);
			this->_speed2 = map(_speed, 0, 100, 0, 255);
		}
	}
	// Set the direction
	if (_M1_direction == 1 && _M2_direction == 1) direction(BothClockWise);
	if (_M1_direction == 1 && _M2_direction == -1) direction(M1CWM2ACW);
	if (_M1_direction == -1 && _M2_direction == 1) direction(M1ACWM2CW);
	if (_M1_direction == -1 && _M2_direction == -1) direction(BothAntiClockWise);
	// send command
	char cmd[3];
    cmd[0] = MotorSpeedSet;		// set pwm header 
    cmd[1] = this->_speed1;  	// send speed of motor1
    cmd[2] = this->_speed2;     // send speed of motor2
    i2c.write( i2c_addr, cmd, 3 );
  	wait_ms(4); 				// Wait 
}

// Set the frequence of PWM(cycle length = 510, system clock = 16MHz)
// F_3921Hz is default
// _frequence: F_31372Hz, F_3921Hz, F_490Hz, F_122Hz, F_30Hz
void I2CMotorDriver::frequence(unsigned char _frequence)
{
	if (_frequence < F_31372Hz || _frequence > F_30Hz) {
		printf("frequence error! Must be F_31372Hz, F_3921Hz, F_490Hz, F_122Hz, F_30Hz\n");
		return;
	}
	char cmd[3];
    cmd[0] = PWMFrequenceSet;	// set frequence header
    cmd[1] = _frequence;  		// send frequence 
    cmd[2] = Nothing;			// need to send this byte as the third byte(no meaning) 
    i2c.write( i2c_addr, cmd, 3 );
	wait_ms(4);					// wait
}

// Stop one motor
// motor_id: MOTOR1, MOTOR2
void I2CMotorDriver::stop(unsigned char motor_id)
{
	if (motor_id<MOTOR1 || motor_id>MOTOR2) {
		printf("Motor id error! Must be MOTOR1 or MOTOR2\n");
		return;
	}
	speed(motor_id, 0);
}
//I2CMotorDriver Motor;
