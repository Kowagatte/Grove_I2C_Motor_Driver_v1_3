/*
 * Grove_I2C_Motor_Driver.cpp
 * A library for Grove - I2C Motor Driver v1.3
 *
 * Copyright (c) 2012 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : Jerry Yip
 * Create Time: 2017-02
 * Change Log : 2018-05-31 1.support two phase stepper motor
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
 *
 * Revised by Author: Kowagatte (Nicholas Noel Ryan)
 * Changes made: Allowed for Motor class to control 2
 * seperate i2c's with 2 motors on each.
 */


#include <Grove_I2C_Motor_Driver.h>
#include <Wire.h>

/*********************************stepper motor type*******************************/
// Define stepper motor type. Support 4 phase stepper motor by default.
// If 2 phase motor is used, define TWO_PHASE_STEPPER_MOTOR in the ino file.
// #define TWO_PHASE_STEPPER_MOTOR

// *********************************Initialize*********************************
// Initialize I2C with an I2C address you set on Grove - I2C Motor Driver v1.3
// default i2c address: 0x0f
int I2CMotorDriver::begin(unsigned char i2c_id, unsigned char i2c_add){
	if(i2c_id<0 || i2c_id>1){
		Serial.println("Error! I2C id must be between 0 and 1 (Inclusive)");
		return(-1);
	}
	if (i2c_add > 0x0f) {
		Serial.println("Error! I2C address must be between 0x00 to 0x0F");
		return(-1); // I2C address error
	}
	Wire.begin();
	delayMicroseconds(10000);
	this->_i2c_add[i2c_id] = i2c_add;
	// Set default frequence to F_3921Hz
	frequence(i2c_id, F_3921Hz);
	return(0); // OK
}

// *****************************Private Function*******************************
// Set the direction of 2 motors
// _direction: M1CWM2ACW(M1 ClockWise M2 AntiClockWise), M1ACWM2CW, BothClockWise, BothAntiClockWise,
void I2CMotorDriver::direction(unsigned char i2c_id, unsigned char _direction){
		if(i2c_id<0 || i2c_id>1){
			Serial.println("Error! I2C id must be between 0 and 1 (Inclusive)");
			return;
		}
		Wire.beginTransmission(this->_i2c_add[i2c_id]); // begin transmission
		Wire.write(DirectionSet);               // Direction control header
		Wire.write(_direction);                 // send direction control information
		Wire.write(Nothing);                    // need to send this byte as the third byte(no meaning)
		Wire.endTransmission();                 // stop transmitting
		delay(4);
}

// *****************************DC Motor Function******************************
// Set the speed of a motor, speed is equal to duty cycle here
// motor_id: MOTOR1, MOTOR2
// _speed: -100~100, when _speed>0, dc motor runs clockwise; when _speed<0,
// dc motor runs anticlockwise
void I2CMotorDriver::speed(unsigned char i2c_id, unsigned char motor_id, int _speed){
	if(i2c_id<0 || i2c_id>1){
		Serial.println("Error! I2C id must be between 0 and 1 (Inclusive)");
		return;
	}
	if(motor_id<MOTOR1 || motor_id>MOTOR2) {
		Serial.println("Motor id error! Must be MOTOR1 or MOTOR2");
		return;
	}

	int index = (i2c_id == 0) ? i2c_id + motor_id : (i2c_id + motor_id) + 1;
	int motorid1 = (i2c_id == 0) ? 0 : 2;
	int motorid2 = (i2c_id == 0) ? 1 : 3;

	if (_speed >= 0) {
		this->_direction[index] = 1;
		_speed = _speed > 100 ? 100 : _speed;
		this->_speed[index] = map(_speed, 0, 100, 0, 255);
	}
	else if (_speed < 0) {
		this->_direction[index] = -1;
		_speed = _speed < -100 ? 100 : -(_speed);
		this->_speed[index] = map(_speed, 0, 100, 0, 255);
	}
	// Set the direction
	if (_direction[motorid1] == 1 && _direction[motorid2] == 1) direction(i2c_id, BothClockWise);
	if (_direction[motorid1] == 1 && _direction[motorid2] == -1) direction(i2c_id, M1CWM2ACW);
	if (_direction[motorid1] == -1 && _direction[motorid2] == 1) direction(i2c_id, M1ACWM2CW);
	if (_direction[motorid1] == -1 && _direction[motorid2] == -1) direction(i2c_id, BothAntiClockWise);
	// send command
	Wire.beginTransmission(this->_i2c_add[i2c_id]); // begin transmission
	Wire.write(MotorSpeedSet);              // set pwm header
	Wire.write(this->_speed[motorid1]);              // send speed of motor1
	Wire.write(this->_speed[motorid2]);              // send speed of motor2
	Wire.endTransmission();    		        // stop transmitting
	delay(4); 				                // Wait
}

// Set the frequence of PWM(cycle length = 510, system clock = 16MHz)
// F_3921Hz is default
// _frequence: F_31372Hz, F_3921Hz, F_490Hz, F_122Hz, F_30Hz
void I2CMotorDriver::frequence(unsigned char i2c_id, unsigned char _frequence){
	if (_frequence < F_31372Hz || _frequence > F_30Hz) {
		Serial.println("frequence error! Must be F_31372Hz, F_3921Hz, F_490Hz, F_122Hz, F_30Hz");
		return;
	}
	Wire.beginTransmission(this->_i2c_add[i2c_id]); // begin transmission
	Wire.write(PWMFrequenceSet);            // set frequence header
	Wire.write(_frequence);                 // send frequence
	Wire.write(Nothing);                    // need to send this byte as the third byte(no meaning)
	Wire.endTransmission();                 // stop transmitting
	delay(4); 				                // wait
}

// Stop one motor
// motor_id: MOTOR1, MOTOR2
void I2CMotorDriver::stop(unsigned char i2c_id, unsigned char motor_id){
	if(i2c_id<0 || i2c_id>1){
		Serial.println("Error! I2C id must be between 0 and 1 (Inclusive)");
		return;
	}
	if (motor_id<MOTOR1 || motor_id>MOTOR2) {
		Serial.println("Motor id error! Must be MOTOR1 or MOTOR2");
		return;
	}
	speed(i2c_id, motor_id, 0);
}

I2CMotorDriver Motor;

// End of File
