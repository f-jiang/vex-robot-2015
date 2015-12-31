#ifndef LFILTER_H_
#define LFILTER_H_

#include <stdint.h>

/**
 * Initializes the linear filter for the specified motor channel. A motor channel cannot be
 * initialized more than once.
 *
 * Parameters:
 * channel - the motor channel to be initialized
 * num_fcycles - the duration of the filtering effect; a higher value will result in a more
 * 				 gradual acceleration
 */
void lfilterInit(const int8_t channel, int8_t numfCycles);

/**
 * Takes in a speed value for a specified channel and calculates a filtered speed. If the
 * channel hasn't been initialized, a speed of 0 will be returned.
 *
 * Because linear filters are meant to provide gradual acceleration, the filtered speed will
 * most likely be different from the target speed specified by the speed parameter.
 *
 * A linear filter provides accleration by calculating an average of the most recent speed
 * values. Each time a new speed is given, it is added to the list of recent speed
 * values; the oldest value on the list is then removed before a new average is calculated.
 * For a more detailed description, see this link:
 * http://www.vexforum.com/showpost.php?p=234355&postcount=14
 *
 * Parameters:
 * channel - the channel for which to calculate a filtered speed
 * speed - the speed value that is to be applied to the filter; must be between -127 and 127
 *
 * Returns: a filtered speed value between -127 and 127
 */
int8_t getfSpeed(const int8_t channel, int16_t speed);

/**
 * Clears the linear filter for each motor channel.
 */
void lfilterClear(void);


#endif /* LFILTER_H_ */
