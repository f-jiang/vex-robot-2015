#include "main.h"

#define MOTOR_LIMIT 12
#define FILTER_CYCLE_LIMIT 10

int ch, cy;														// ideal type: int8_t
int data[MOTOR_LIMIT][FILTER_CYCLE_LIMIT] = { { 0 } };			// ideal type: int8_t
int chindex[MOTOR_LIMIT] = { [0 ... MOTOR_LIMIT - 1] = -1 };	// ideal type: int8_t
int fcycles[MOTOR_LIMIT] = { 0 };								// ideal type: uint8_t
int count = 0;													// ideal type: uint8_t

void lfilterInit(const int/*uint8_t*/ channel, int/*uint8_t*/ num_fcycles) {
	if (channel > 0 && channel <= MOTOR_LIMIT
			&& count < MOTOR_LIMIT
			&& chindex[channel - 1] == -1) {
		if (num_fcycles > FILTER_CYCLE_LIMIT) {
			num_fcycles = FILTER_CYCLE_LIMIT;
		} else if (num_fcycles <= 0) {
			num_fcycles = 1;
		}

		chindex[channel - 1] = count;
		fcycles[count++] = num_fcycles;
	}
}

int/*int8_t*/ getfSpeed(const int/*uint8_t*/ channel, int/*int16_t*/ speed) {
	int fspeed = 0;
	ch = chindex[channel - 1];

	if (ch != -1) {
		if (speed > INT8_MAX) {	// when calling motorSet, the speed must be between -127 and 127
			speed = INT8_MAX;
		} else if (speed < INT8_MIN) {
			speed = INT8_MIN;
		}

		for (cy = fcycles[ch] - 1; cy > -1; --cy) {
			data[ch][cy] = (cy == 0) ? speed : data[ch][cy - 1];
			fspeed += data[ch][cy];
		}

		fspeed /= fcycles[ch];
	}

	return fspeed;
}

void lfilterReset(void) {
	for (ch = 0; ch < MOTOR_LIMIT; ++ch) {
		for (cy = 0; cy < fcycles[ch]; ++cy) {
			data[ch][cy] = 0;
		}
	}
}
