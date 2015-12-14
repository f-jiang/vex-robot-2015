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

const int8_t DRIVE_AXIS = 3;
const int8_t STRAFE_AXIS = 4;
const int8_t ROTATION_AXIS = 1;
const int8_t DRIVE_BUTTON_GROUP = 7;
const int8_t SHOOTER_BUTTON_GROUP = 8;
const int8_t LIFTER_BUTTON_GROUP = 5;
const int8_t BALL_INTAKE_BUTTON_GROUP = 6;

const int8_t JOYSTICK_SLOT = 1;

const int8_t WALKING_SPEED = 40;
const int8_t DIAGONAL_DRIVE_DEADBAND = 30;
const int8_t MOVEMENT_DEADBAND = 30;

const int8_t BALL_INTAKE_SPEED = 60;
const int8_t LIFTER_SPEED = 60;

const int8_t SHOOTER_SPEED = 80;

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
void drive(int8_t vx, int8_t vy, int8_t r, bool is_field_centric) {
	int8_t flspeed, blspeed, frspeed, brspeed;

	if (is_field_centric) {
		float heading = -gyroGet(gyro) % 360 * M_PI / 180;
		float v = hypotf(vx, vy);
		vx = (int8_t) (v * sinf(heading));
		vy = (int8_t) (v * cosf(heading));
	}

	// Linear filtering for gradual acceleration and reduced motor wear
	flspeed = getfSpeed(FRONT_LEFT_MOTOR_CHANNEL, vy + vx + r);
	blspeed = getfSpeed(BACK_LEFT_MOTOR_CHANNEL, vy - vx + r);
	frspeed = getfSpeed(FRONT_RIGHT_MOTOR_CHANNEL, -vy + vx + r);
	brspeed = getfSpeed(BACK_RIGHT_MOTOR_CHANNEL, -vy - vx + r);

	motorSet(FRONT_LEFT_MOTOR_CHANNEL, flspeed);
	motorSet(BACK_LEFT_MOTOR_CHANNEL, blspeed);
	motorSet(FRONT_RIGHT_MOTOR_CHANNEL, frspeed);
	motorSet(BACK_RIGHT_MOTOR_CHANNEL, brspeed);
}

void ballIntake(int8_t ispeed) {
	// Linear filtering for gradual acceleration and reduced motor wear
		int8_t ispeed2 = getfSpeed(BALL_INTAKE_MOTOR_CHANNEL, ispeed);
		motorSet(BALL_INTAKE_MOTOR_CHANNEL, ispeed2);
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
	int8_t xspeed, yspeed, rotation;
	int8_t liftSpeed, intakeSpeed;
	int8_t shooterOffset = 0;
	int8_t shooterSpeed = 0;
	bool previous_increase_state = false; //corresponds to shooter buttons, PURPOSE: toggle
	bool previous_decrease_state = false; //corresponds to shooter buttons, PURPOSE: toggle
	bool previous_toggle_state = false;   //corresponds to shooter buttons, PURPOSE: toggle
	bool is_shooter_on = true;

	//lfilterClear();

	while (true) {
		xspeed = (int8_t) joystickGetAnalog(JOYSTICK_SLOT, STRAFE_AXIS);
		yspeed = (int8_t) joystickGetAnalog(JOYSTICK_SLOT, DRIVE_AXIS);
		rotation = (int8_t) joystickGetAnalog(JOYSTICK_SLOT, ROTATION_AXIS) / 2;

		// Uses button-based drive controls if joysticks aren't being used
		if (abs(xspeed) < MOVEMENT_DEADBAND &&
			abs(yspeed) < MOVEMENT_DEADBAND &&
			abs(rotation) < MOVEMENT_DEADBAND) {
			//TODO: adjust WALKING_SPEED signs as necessary
			//TODO: adjust WALKING_SPEED value as necessary
			if (joystickGetDigital(JOYSTICK_SLOT, DRIVE_BUTTON_GROUP, JOY_UP)) {
				yspeed = WALKING_SPEED;
			} else if (joystickGetDigital(JOYSTICK_SLOT, DRIVE_BUTTON_GROUP, JOY_DOWN)) {
				yspeed = -WALKING_SPEED;
			}

			if (joystickGetDigital(JOYSTICK_SLOT, DRIVE_BUTTON_GROUP, JOY_LEFT)) {
				xspeed = -WALKING_SPEED;
			} else if (joystickGetDigital(JOYSTICK_SLOT, DRIVE_BUTTON_GROUP, JOY_RIGHT)) {
				xspeed = WALKING_SPEED;
			}
		// To allow for straight movement, xspeed and yspeed are set to 0 if their values are
		// negligibly small.
		} else {
			if (abs(yspeed) < DIAGONAL_DRIVE_DEADBAND) {
				yspeed = 0;
			}

			if (abs(xspeed) < DIAGONAL_DRIVE_DEADBAND) {
				xspeed = 0;
			}
		}

		drive(xspeed, yspeed, rotation, false);

		if (joystickGetDigital(JOYSTICK_SLOT, BALL_INTAKE_BUTTON_GROUP, JOY_UP)) {
			intakeSpeed = BALL_INTAKE_SPEED;
		} else if (joystickGetDigital(JOYSTICK_SLOT, BALL_INTAKE_BUTTON_GROUP ,JOY_DOWN)) {
			intakeSpeed = -BALL_INTAKE_SPEED;
		} else {
			intakeSpeed = 0;
		}

		if (joystickGetDigital(JOYSTICK_SLOT, LIFTER_BUTTON_GROUP, JOY_UP)) {
			liftSpeed = LIFTER_SPEED;
		} else if (joystickGetDigital(JOYSTICK_SLOT, LIFTER_BUTTON_GROUP ,JOY_DOWN)) {
			liftSpeed = -LIFTER_SPEED;
		} else {
			liftSpeed = 0;
		}


		// shooter on button
		if (joystickGetDigital(JOYSTICK_SLOT, SHOOTER_BUTTON_GROUP, JOY_DOWN)) {
			if (!previous_toggle_state) {
				is_shooter_on = !is_shooter_on;
				shooterSpeed = is_shooter_on ? SHOOTER_SPEED + shooterOffset : 0;
			}
			previous_toggle_state = true;
		} else {
			previous_toggle_state = false;
		}

		// shooter decrease speed
		if (joystickGetDigital(JOYSTICK_SLOT, SHOOTER_BUTTON_GROUP, JOY_LEFT)) {
			if (!previous_decrease_state && shooterOffset >= -70){
				shooterOffset = shooterOffset - 10;
			}
			previous_decrease_state = true;
		}	else {
			previous_increase_state = false;
		}

		// shooter increase speed
		if (joystickGetDigital(JOYSTICK_SLOT, SHOOTER_BUTTON_GROUP, JOY_RIGHT)) {
			if (!previous_increase_state && shooterOffset <= 40){
				shooterOffset += 10;
			}
			previous_increase_state = true;
		}  else {
			previous_increase_state = false;
		}

		ballIntake(intakeSpeed);
		lifter(liftSpeed);
		shooter(shooterSpeed);

		delay(20);
	}
}
