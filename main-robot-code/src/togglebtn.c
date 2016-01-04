/*
 * togglebtn.c
 *
 *  Created on: 2016-01-03
 *      Author: feilan
 */

#include "togglebtn.h"
#include "API.h"

#define BTN_LIMIT 12

struct toggleBtn {
	int8_t jstick, btnGroup, btn;
	bool prevState, curState;
};

static struct toggleBtn data[BTN_LIMIT] = { { 0 } };
static struct toggleBtn *btn;
static int8_t count = 0;

void toggleBtnInit(int8_t joystick, int8_t buttonGroup, int8_t button) {
	btn = data + count++;
	btn->jstick = joystick;
	btn->btnGroup = buttonGroup;
	btn->btn = button;
	btn->prevState = btn->curState = false;
}

enum btnState toggleBtnGet(int8_t joystick, int8_t buttonGroup, int8_t button) {
	enum btnState state = NO_STATE;
	btn = data;

	for (int8_t i = 0; i < count; ++i, ++btn) {
		if (btn->jstick == joystick && btn->btnGroup == buttonGroup && btn->btn == button) {
			if (btn->curState && btn->prevState) {
				state = BTN_HELD;
			} else if (btn->curState) {
				state = BTN_PRESSED;
			} else if (btn->prevState) {
				state = BTN_RELEASED;
			} else {
				state = BTN_NOT_PRESSED;
			}

			break;
		}
	}

	return state;
}

void toggleBtnUpdateAll(void) {
	btn = data;

	for (int8_t i = 0; i < count; ++i, ++btn) {
		btn->prevState = btn->curState;
		btn->curState = joystickGetDigital(btn->jstick, btn->btnGroup, btn->btn);
	}
}
