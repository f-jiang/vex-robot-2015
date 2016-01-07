#include "actions.h"

#include "main.h"
#include "API.h"
#include "lfilter.h"
#include <math.h>

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

	if (maxRawSpeed > MAX_SPEED) {	// TODO: replace MAXIMUM_SHOOTER_CAP with macro
		float scale = (float) maxRawSpeed / MAX_SPEED;
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

int8_t calculateShooterSpeed() {
	float dist = ultrasonicGet(ultra) / 2.54;
	float speed = 1.11 * dist - 1.6;

	if (speed > MAX_SPEED) {
		speed = MAX_SPEED;
	} else if (speed < 0) {
		speed = 0;
	}

	return (int8_t) speed;
}
