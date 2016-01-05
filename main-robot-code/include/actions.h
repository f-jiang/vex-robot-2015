#ifndef ACTIONS_H_
#define ACTIONS_H_

#include <stdint.h>
#include <stdbool.h>

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
void drive(int8_t vx, int8_t vy, int8_t r, bool isFieldCentric);

void takeInInternal(int8_t ispeed);

void lifter(int8_t lspeed);

void shooter(int8_t sspeed);

void takeInFront(int8_t speed);

int8_t calculateShooterSpeed();

#endif /* ACTIONS_H_ */
