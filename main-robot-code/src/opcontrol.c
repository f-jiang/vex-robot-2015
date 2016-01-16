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

#include "actions.h"
#include "togglebtn.h"
#include <stdint.h>
#include <stdbool.h>

#define JOYSTICK_SLOT 1

#define DRIVE_AXIS 3
#define STRAFE_AXIS 4
#define ROTATION_AXIS 1

#define INTAKE_BUTTON_GROUP 7
#define CONTROL_BUTTON_GROUP 8
#define LIFTER_BUTTON_GROUP 5
#define SHOOTER_ADJUST_BUTTON_GROUP 6

#define DIAGONAL_DRIVE_DEADBAND 30

#define INTAKE_SPEED 127
#define LIFTER_SPEED 60

#define SHOOTER_MAX_SPEED MAX_SPEED
#define SHOOTER_MIN_SPEED 0

//#define AUTO
//#define TEST

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
#ifdef AUTO
	autonomous();
#elif defined(TEST)

#define DEFAULT_SHOOTER_SPEED 0
#define SHOOTER_SPEED_INCREMENT 5

	int8_t xSpeed, ySpeed, rotation;
	int8_t lifterSpeed/*, intakeSpeed*/;

	int16_t shooterSpeed = DEFAULT_SHOOTER_SPEED; //shooter is on when robot starts
	int8_t frontIntakeSpeed = INTAKE_SPEED;
	bool isShooterOn = true;
	bool isAutoShootOn = false;

	//lfilterClear();

	toggleBtnInit(JOYSTICK_SLOT, INTAKE_BUTTON_GROUP, JOY_UP);		// intake forward
	toggleBtnInit(JOYSTICK_SLOT, INTAKE_BUTTON_GROUP, JOY_LEFT);	// intake off
	toggleBtnInit(JOYSTICK_SLOT, INTAKE_BUTTON_GROUP, JOY_RIGHT);	// intake off
	toggleBtnInit(JOYSTICK_SLOT, INTAKE_BUTTON_GROUP, JOY_DOWN);	// intake backward
	toggleBtnInit(JOYSTICK_SLOT, CONTROL_BUTTON_GROUP, JOY_DOWN);   // shooter on off
	toggleBtnInit(JOYSTICK_SLOT, CONTROL_BUTTON_GROUP, JOY_RIGHT);   // auto shoot on off
	toggleBtnInit(JOYSTICK_SLOT, SHOOTER_ADJUST_BUTTON_GROUP, JOY_UP);   // shooter speed up
	toggleBtnInit(JOYSTICK_SLOT, SHOOTER_ADJUST_BUTTON_GROUP, JOY_DOWN);   // shooter speed down

	while (true) {
		printf("ultra distance (in): %f\r\n", ultrasonicGet(ultra) / 2.54);

		toggleBtnUpdateAll();

		// drive
		xSpeed = (int8_t) joystickGetAnalog(JOYSTICK_SLOT, STRAFE_AXIS);
		ySpeed = (int8_t) joystickGetAnalog(JOYSTICK_SLOT, DRIVE_AXIS);
		rotation = (int8_t) joystickGetAnalog(JOYSTICK_SLOT, ROTATION_AXIS) / 2;

		if (abs(ySpeed) < DIAGONAL_DRIVE_DEADBAND) {
			ySpeed = 0;
		}

		if (abs(xSpeed) < DIAGONAL_DRIVE_DEADBAND) {
			xSpeed = 0;
		}

		drive(xSpeed, ySpeed, rotation, false);

		// lifter up down
		if (joystickGetDigital(JOYSTICK_SLOT, LIFTER_BUTTON_GROUP, JOY_UP)) {
			lifterSpeed = LIFTER_SPEED;
		} else if (joystickGetDigital(JOYSTICK_SLOT, LIFTER_BUTTON_GROUP, JOY_DOWN)) {
			lifterSpeed = -LIFTER_SPEED;
		} else {
			lifterSpeed = 0;
		}

		lifter(lifterSpeed);
		takeInInternal(lifterSpeed);

		if (isShooterOn) {
			if (isAutoShootOn) {
				shooterSpeed = calculateShooterSpeed();
			} else {
				// shooter increase speed
				if (toggleBtnGet(JOYSTICK_SLOT, SHOOTER_ADJUST_BUTTON_GROUP, JOY_UP) == BUTTON_PRESSED) {
					shooterSpeed += SHOOTER_SPEED_INCREMENT;

					if (shooterSpeed > SHOOTER_MAX_SPEED) {
						shooterSpeed = SHOOTER_MAX_SPEED;
					}
				}

				// shooter decrease speed
				if (toggleBtnGet(JOYSTICK_SLOT, SHOOTER_ADJUST_BUTTON_GROUP, JOY_DOWN) == BUTTON_PRESSED) {
					shooterSpeed -= SHOOTER_SPEED_INCREMENT;

					if (shooterSpeed < SHOOTER_MIN_SPEED) {
						shooterSpeed = SHOOTER_MIN_SPEED;
					}
				}
			}

//			 auto shooter on off
			if (toggleBtnGet(JOYSTICK_SLOT, CONTROL_BUTTON_GROUP, JOY_RIGHT) == BUTTON_PRESSED) {
				isAutoShootOn = !isAutoShootOn;
			}
		}

		// shooter on off
		if (toggleBtnGet(JOYSTICK_SLOT, CONTROL_BUTTON_GROUP, JOY_DOWN) == BUTTON_PRESSED) {
			isShooterOn = !isShooterOn;
			shooterSpeed = isShooterOn ? DEFAULT_SHOOTER_SPEED : 0;
		}

		shooter(shooterSpeed);

		// intake mode
		if (toggleBtnGet(JOYSTICK_SLOT, INTAKE_BUTTON_GROUP, JOY_LEFT) == BUTTON_PRESSED
			|| toggleBtnGet(JOYSTICK_SLOT, INTAKE_BUTTON_GROUP, JOY_RIGHT) == BUTTON_PRESSED) {
			frontIntakeSpeed = 0;
		} else if (toggleBtnGet(JOYSTICK_SLOT, INTAKE_BUTTON_GROUP, JOY_UP) == BUTTON_PRESSED) {
			frontIntakeSpeed = -INTAKE_SPEED;
		} else if (toggleBtnGet(JOYSTICK_SLOT, INTAKE_BUTTON_GROUP, JOY_DOWN) == BUTTON_PRESSED) {
			frontIntakeSpeed = INTAKE_SPEED;
		}

		takeInFront(frontIntakeSpeed);

		delay(20);
	}
#else
	int8_t xSpeed, ySpeed, rotation;
	int8_t lifterSpeed/*, intakeSpeed*/;
	int8_t defaultPreset = 0;
	int8_t currentPreset = defaultPreset;

	int16_t shooterSpeed = shooterSpeedPresets[defaultPreset]; //shooter is on when robot starts
	int8_t frontIntakeSpeed = INTAKE_SPEED;
	bool isShooterOn = true;

	//lfilterClear();

	toggleBtnInit(JOYSTICK_SLOT, INTAKE_BUTTON_GROUP, JOY_UP);		// intake forward
	toggleBtnInit(JOYSTICK_SLOT, INTAKE_BUTTON_GROUP, JOY_LEFT);	// intake off
	toggleBtnInit(JOYSTICK_SLOT, INTAKE_BUTTON_GROUP, JOY_RIGHT);	// intake off
	toggleBtnInit(JOYSTICK_SLOT, INTAKE_BUTTON_GROUP, JOY_DOWN);	// intake backward
	toggleBtnInit(JOYSTICK_SLOT, CONTROL_BUTTON_GROUP, JOY_DOWN);   // shooter on off
	toggleBtnInit(JOYSTICK_SLOT, SHOOTER_ADJUST_BUTTON_GROUP, JOY_UP);   // shooter speed up
	toggleBtnInit(JOYSTICK_SLOT, SHOOTER_ADJUST_BUTTON_GROUP, JOY_DOWN);   // shooter speed down

	while (true) {
		printf("ultra distance (in): %f\r\n", ultrasonicGet(ultra) / 2.54);

		toggleBtnUpdateAll();

		// drive
		xSpeed = (int8_t) joystickGetAnalog(JOYSTICK_SLOT, STRAFE_AXIS);
		ySpeed = (int8_t) joystickGetAnalog(JOYSTICK_SLOT, DRIVE_AXIS);
		rotation = (int8_t) joystickGetAnalog(JOYSTICK_SLOT, ROTATION_AXIS) / 2;

		if (abs(ySpeed) < DIAGONAL_DRIVE_DEADBAND) {
			ySpeed = 0;
		}

		if (abs(xSpeed) < DIAGONAL_DRIVE_DEADBAND) {
			xSpeed = 0;
		}

		drive(xSpeed, ySpeed, rotation, false);

		// lifter up down
		if (joystickGetDigital(JOYSTICK_SLOT, LIFTER_BUTTON_GROUP, JOY_UP)) {
			lifterSpeed = LIFTER_SPEED;
		} else if (joystickGetDigital(JOYSTICK_SLOT, LIFTER_BUTTON_GROUP, JOY_DOWN)) {
			lifterSpeed = -LIFTER_SPEED;
		} else {
			lifterSpeed = 0;
		}

		lifter(lifterSpeed);
		takeInInternal(lifterSpeed);

		if (isShooterOn) {
			// shooter increase speed
			if (toggleBtnGet(JOYSTICK_SLOT, SHOOTER_ADJUST_BUTTON_GROUP, JOY_UP) == BUTTON_PRESSED) {
				++currentPreset;

				if (currentPreset >= NUM_SHOOTER_SPEED_PRESETS) {
					currentPreset = NUM_SHOOTER_SPEED_PRESETS - 1;
				}
			}

			// shooter decrease speed
			if (toggleBtnGet(JOYSTICK_SLOT, SHOOTER_ADJUST_BUTTON_GROUP, JOY_DOWN) == BUTTON_PRESSED) {
				--currentPreset;

				if (currentPreset < 0) {
					currentPreset = 0;
				}
			}

			shooterSpeed = shooterSpeedPresets[currentPreset];
		}

		// shooter on off
		if (toggleBtnGet(JOYSTICK_SLOT, CONTROL_BUTTON_GROUP, JOY_DOWN) == BUTTON_PRESSED) {
			isShooterOn = !isShooterOn;
			shooterSpeed = isShooterOn ? shooterSpeedPresets[defaultPreset] : 0;
		}

		shooter(shooterSpeed);

		// intake mode
		if (toggleBtnGet(JOYSTICK_SLOT, INTAKE_BUTTON_GROUP, JOY_LEFT) == BUTTON_PRESSED
			|| toggleBtnGet(JOYSTICK_SLOT, INTAKE_BUTTON_GROUP, JOY_RIGHT) == BUTTON_PRESSED) {
			frontIntakeSpeed = 0;
		} else if (toggleBtnGet(JOYSTICK_SLOT, INTAKE_BUTTON_GROUP, JOY_UP) == BUTTON_PRESSED) {
			frontIntakeSpeed = -INTAKE_SPEED;
		} else if (toggleBtnGet(JOYSTICK_SLOT, INTAKE_BUTTON_GROUP, JOY_DOWN) == BUTTON_PRESSED) {
			frontIntakeSpeed = INTAKE_SPEED;
		}

		takeInFront(frontIntakeSpeed);

		delay(20);
	}
#endif
}
