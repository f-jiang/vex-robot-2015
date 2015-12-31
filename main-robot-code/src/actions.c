#include "actions.h"

#include "main.h"
#include "API.h"
#include "lfilter.h"
#include <math.h>

void drive(int8_t vx, int8_t vy, int8_t r, bool isFieldCentric) {
	int8_t flspeed, blspeed, frspeed, brspeed;

	if (isFieldCentric) {
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
