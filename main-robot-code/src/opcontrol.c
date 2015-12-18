/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * Copyright (c) 2011-2014, Purdue University ACM SIG BOTS.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Purdue University ACM SIG BOTS nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Purdue Robotics OS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

const int8_t JOYSTICK_SLOT = 1;

const int8_t DRIVE_AXIS = 3;
const int8_t STRAFE_AXIS = 4;
const int8_t ROTATION_AXIS = 1;

const int8_t DRIVE_BUTTON_GROUP = 7;
const int8_t SHOOTER_BUTTON_GROUP = 8;
const int8_t LIFTER_BUTTON_GROUP = 5;
const int8_t INTAKE_BUTTON_GROUP = 6;

const int8_t WALKING_SPEED = 40;
const int8_t DIAGONAL_DRIVE_DEADBAND = 30;
const int8_t MOVEMENT_DEADBAND = 30;

const int8_t INTAKE_SPEED = 127;
const int8_t LIFTER_SPEED = 60;

const int8_t DEFAULT_SHOOTER_SPEED = 127;
const int8_t SHOOTER_SPEED_INCREMENT = 10;
const int8_t MAXIMUM_SHOOTER_CAP = 127;
const int8_t MINIMUM_SHOOTER_CAP = 0;

/**
 * TODO: test field-centric drive
 * TODO: see if signs need to be changed
 *
 * Drives and/or rotates the robot at the specified speed. The direction is determined from the
 * hypotenuse of vx and vy (see "Parameters" for their definitions).
 *
 * Because the robot has a holonomic drive, it is capable of traversing at any angle regardless
 * of its orientation.
 *
 * This function only applies one pulse to the drive motors and should be called from within a
 * loop.
 *
 * Parameters:
 * vx - the horizontal speed of the robot; -127 for full left, 127 for full right
 * vy - the vertical speed of the robot; -127 for full down, 127 for full up
 * r  - the rotational speed of the robot; -127 for full counterclockwise, 127 for full clockwise
 * is_field_centric - if set to true, the direction of the robot's motion will be relative to the
 * 					  playing field rather than the robot
 */
void drive(int8_t vx, int8_t vy, int8_t r, bool isFieldCentric) {
	int16_t speed[4];	// one for each wheel
	int16_t absRawSpeed, maxRawSpeed;
	int8_t i;

	if (isFieldCentric) {
		float heading = -gyroGet(gyro) % 360 * M_PI / 180;
		float v = hypotf(vx, vy);
		vx = (int8_t) (v * sinf(heading));
		vy = (int8_t) (v * cosf(heading));
	}

	speed[0] = vy + vx + r;	// front left
	speed[1] = vy - vx + r;	// back left
	speed[2] = -vy + vx + r;	// front right
	speed[3] = -vy - vx + r;	// back right

	maxRawSpeed = 0;
	for (i = 0; i < 4; ++i) {
		absRawSpeed = abs(speed[i]);
		if (absRawSpeed > maxRawSpeed) {
			maxRawSpeed = absRawSpeed;
		}
	}

	if (maxRawSpeed > MAXIMUM_SHOOTER_CAP) {	// TODO: replace MAXIMUM_SHOOTER_CAP with macro
		float scale = (float) maxRawSpeed / MAXIMUM_SHOOTER_CAP;
		for (i = 0; i < 4; ++i) {
			speed[i] /= scale;
		}
	}

	// Linear filtering for gradual acceleration and reduced motor wear
	speed[0] = getfSpeed(FRONT_LEFT_MOTOR_CHANNEL, speed[0]);
	speed[1] = getfSpeed(BACK_LEFT_MOTOR_CHANNEL, speed[1]);
	speed[2] = getfSpeed(FRONT_RIGHT_MOTOR_CHANNEL, speed[2]);
	speed[3] = getfSpeed(BACK_RIGHT_MOTOR_CHANNEL, speed[3]);

	motorSet(FRONT_LEFT_MOTOR_CHANNEL, speed[0]);
	motorSet(BACK_LEFT_MOTOR_CHANNEL, speed[1]);
	motorSet(FRONT_RIGHT_MOTOR_CHANNEL, speed[2]);
	motorSet(BACK_RIGHT_MOTOR_CHANNEL, speed[3]);
}

void takeInInternal(int8_t ispeed) {
	// Linear filtering for gradual acceleration and reduced motor wear
	int8_t ispeed2 = getfSpeed(INTERNAL_INTAKE_MOTOR_CHANNEL, ispeed);
	motorSet(INTERNAL_INTAKE_MOTOR_CHANNEL, ispeed2);
}

void lifter(int8_t lspeed) {
	// Linear filtering for gradual acceleration and reduced motor wear
	int8_t lspeed2 = getfSpeed(LIFTER_MOTOR_CHANNEL, lspeed);
	motorSet(LIFTER_MOTOR_CHANNEL, lspeed2);
}

void shooter(int8_t sspeed){
	// Linear filtering for gradual acceleration and reduced motor wear
	int8_t sspeed2 = getfSpeed(SHOOTER_MOTOR_CHANNEL, -sspeed);
	int8_t sspeed3 = getfSpeed(SHOOTER_MOTOR_CHANNEL2, sspeed);
	motorSet (SHOOTER_MOTOR_CHANNEL , sspeed2);
	motorSet (SHOOTER_MOTOR_CHANNEL2, sspeed3);
}

void takeInFront(int8_t speed) {
	int8_t fspeed = getfSpeed(FRONT_INTAKE_MOTOR_CHANNEL, -speed);
	motorSet(FRONT_INTAKE_MOTOR_CHANNEL, fspeed);
}

/*
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
void operatorControl() {
	int8_t xSpeed, ySpeed, rotation;
	int8_t lifterSpeed/*, intakeSpeed*/;

	int16_t shooterSpeed = DEFAULT_SHOOTER_SPEED;	//shooter is on when robot starts
	bool previousIncreaseState = false; //corresponds to shooter buttons, PURPOSE: toggle
	bool previousDecreaseState = false; //corresponds to shooter buttons, PURPOSE: toggle
	bool previousToggleState = false;   //corresponds to shooter buttons, PURPOSE: toggle
	bool isShooterOn = true;

	bool isFrontIntakeOn = true;
	bool previousIntakeToggleState = false;
	int8_t frontIntakeSpeed = INTAKE_SPEED;
	//lfilterClear();

	while (true) {
		xSpeed = (int8_t) joystickGetAnalog(JOYSTICK_SLOT, STRAFE_AXIS);
		ySpeed = (int8_t) joystickGetAnalog(JOYSTICK_SLOT, DRIVE_AXIS);
		rotation = (int8_t) joystickGetAnalog(JOYSTICK_SLOT, ROTATION_AXIS) / 2;

		// Uses button-based drive controls if joysticks aren't being used
		if (abs(xSpeed) < MOVEMENT_DEADBAND && abs(ySpeed) < MOVEMENT_DEADBAND &&
				abs(rotation) < MOVEMENT_DEADBAND) {
			//TODO: adjust WALKING_SPEED signs as necessary
			//TODO: adjust WALKING_SPEED value as necessary
			if (joystickGetDigital(JOYSTICK_SLOT, DRIVE_BUTTON_GROUP, JOY_UP)) {
				ySpeed = WALKING_SPEED;
			} else if (joystickGetDigital(JOYSTICK_SLOT, DRIVE_BUTTON_GROUP, JOY_DOWN)) {
				ySpeed = -WALKING_SPEED;
			}

			if (joystickGetDigital(JOYSTICK_SLOT, DRIVE_BUTTON_GROUP, JOY_LEFT)) {
				xSpeed = -WALKING_SPEED;
			} else if (joystickGetDigital(JOYSTICK_SLOT, DRIVE_BUTTON_GROUP, JOY_RIGHT)) {
				xSpeed = WALKING_SPEED;
			}
			// To allow for straight movement, xspeed and yspeed are set to 0 if their values are
			// negligibly small.
		} else {
			if (abs(ySpeed) < DIAGONAL_DRIVE_DEADBAND) {
				ySpeed = 0;
			}

			if (abs(xSpeed) < DIAGONAL_DRIVE_DEADBAND) {
				xSpeed = 0;
			}
		}

		drive(xSpeed, ySpeed, rotation, false);

		/*if (joystickGetDigital(JOYSTICK_SLOT, INTAKE_BUTTON_GROUP, JOY_UP)) {
			intakeSpeed = INTAKE_SPEED;
		} else if (joystickGetDigital(JOYSTICK_SLOT, INTAKE_BUTTON_GROUP ,JOY_DOWN)) {
			intakeSpeed = -INTAKE_SPEED;
		} else {
			intakeSpeed = 0;
		}

		takeInInternal(intakeSpeed);*/

		if (joystickGetDigital(JOYSTICK_SLOT, LIFTER_BUTTON_GROUP, JOY_UP)) {
			lifterSpeed = LIFTER_SPEED;
		} else if (joystickGetDigital(JOYSTICK_SLOT, LIFTER_BUTTON_GROUP, JOY_DOWN)) {
			lifterSpeed = -LIFTER_SPEED;
		} else {
			lifterSpeed = 0;
		}

		lifter(lifterSpeed);
		takeInInternal(lifterSpeed);

		// shooter on button
		if (joystickGetDigital(JOYSTICK_SLOT, SHOOTER_BUTTON_GROUP, JOY_DOWN)) {
			if (!previousToggleState) {
				isShooterOn = !isShooterOn;
				shooterSpeed = isShooterOn ? DEFAULT_SHOOTER_SPEED : 0;
			}
			previousToggleState = true;
		} else {
			previousToggleState = false;
		}

		// shooter decrease speed
		if (joystickGetDigital(JOYSTICK_SLOT, SHOOTER_BUTTON_GROUP, JOY_RIGHT)) {
			if (!previousDecreaseState && isShooterOn){
				shooterSpeed += SHOOTER_SPEED_INCREMENT;
				if (shooterSpeed > MAXIMUM_SHOOTER_CAP) {
					shooterSpeed = MAXIMUM_SHOOTER_CAP;
				}
			}
			previousDecreaseState = true;
		} else {
			previousDecreaseState = false;
		}

		// shooter increase speed
		if (joystickGetDigital(JOYSTICK_SLOT, SHOOTER_BUTTON_GROUP, JOY_LEFT)) {
			if (!previousIncreaseState && isShooterOn){
				shooterSpeed -= SHOOTER_SPEED_INCREMENT;
				if (shooterSpeed < MINIMUM_SHOOTER_CAP) {
					shooterSpeed = MINIMUM_SHOOTER_CAP;
				}
			}
			previousIncreaseState = true;
		} else {
			previousIncreaseState = false;
		}

		shooter(shooterSpeed);

		// front intake on/off
		if (joystickGetDigital(JOYSTICK_SLOT, SHOOTER_BUTTON_GROUP, JOY_UP)) {
			if (!previousIntakeToggleState) {
				isFrontIntakeOn = !isFrontIntakeOn;
				frontIntakeSpeed = isFrontIntakeOn ? INTAKE_SPEED : 0;
			}
			previousIntakeToggleState = true;
		} else {
			previousIntakeToggleState = false;
		}

		takeInFront(frontIntakeSpeed);

		delay(20);
	}
}
