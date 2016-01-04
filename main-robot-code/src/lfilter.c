#include "lfilter.h"

#include "main.h"

#define MOTOR_LIMIT 10
#define FILTER_CYCLE_LIMIT 12

static int8_t ch, cy;
static int8_t data[MOTOR_LIMIT][FILTER_CYCLE_LIMIT] = { { 0 } };
static int8_t chindex[MOTOR_LIMIT] = { [0 ... MOTOR_LIMIT - 1] = -1 };
static int8_t fcycles[MOTOR_LIMIT] = { 0 };
static int8_t count = 0;

void lfilterInit(const int8_t channel, int8_t numfCycles) {
	if (channel > 0 && channel <= MOTOR_LIMIT
			&& count < MOTOR_LIMIT
			&& chindex[channel - 1] == -1) {
		if (numfCycles > FILTER_CYCLE_LIMIT) {
			numfCycles = FILTER_CYCLE_LIMIT;
		} else if (numfCycles < 1) {
			numfCycles = 1;
		}

		chindex[channel - 1] = count;
		fcycles[count++] = numfCycles;
	}
}

int8_t getfSpeed(const int8_t channel, int16_t speed) {
	int16_t fspeed = 0;

	if (channel > 0 && channel <= MOTOR_LIMIT && chindex[channel - 1] != -1) {
		ch = chindex[channel - 1];

		if (speed > MAX_SPEED) {	// when calling motorSet, the speed must be between -127 and 127
			speed = MAX_SPEED;
		} else if (speed < MIN_SPEED) {
			speed = MIN_SPEED;
		}

		for (cy = fcycles[ch] - 1; cy > -1; --cy) {
			data[ch][cy] = (cy == 0) ? (int8_t) speed : data[ch][cy - 1];
			fspeed += data[ch][cy];
		}

		fspeed /= fcycles[ch];
	}

	return (int8_t) fspeed;
}

void lfilterClear(void) {
	for (ch = 0; ch < MOTOR_LIMIT; ++ch) {
		for (cy = 0; cy < fcycles[ch]; ++cy) {
			data[ch][cy] = 0;
		}
	}
}
