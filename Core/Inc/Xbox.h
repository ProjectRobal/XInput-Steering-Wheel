/*
 * Xbox.h
 *
 *  Created on: 16 lis 2021
 *      Author: patry
 */

#ifndef INC_XBOX_H_
#define INC_XBOX_H_


#include "usb_xinf.h"



enum XInputControl {
	BUTTON_LOGO = 0,
	BUTTON_A = 1,
	BUTTON_B = 2,
	BUTTON_X = 3,
	BUTTON_Y = 4,
	BUTTON_LB = 5,
	BUTTON_RB = 6,
	BUTTON_BACK = 7,
	BUTTON_START = 8,
	BUTTON_L3 = 9,
	BUTTON_R3 = 10,
	DPAD_UP = 11,
	DPAD_DOWN = 12,
	DPAD_LEFT = 13,
	DPAD_RIGHT = 14,
	TRIGGER_LEFT = 15,
	TRIGGER_RIGHT = 16,
	JOY_LEFT,
	JOY_RIGHT,
};

enum  XInputReceiveType  {
	Rumble = 0x00,
	LEDs = 0x01,
};

enum  XInputLEDPattern {
	Off = 0x00,
	Blinking = 0x01,
	Flash1 = 0x02,
	Flash2 = 0x03,
	Flash3 = 0x04,
	Flash4 = 0x05,
	On1 = 0x06,
	On2 = 0x07,
	On3 = 0x08,
	On4 = 0x09,
	Rotating = 0x0A,
	BlinkOnce = 0x0B,
	BlinkSlow = 0x0C,
	Alternating = 0x0D,
};

class Xbox {
public:
	Xbox();

	void begin();

		// Set Control Surfaces
		void press(uint8_t button);
		void release(uint8_t button);
		void setButton(uint8_t button, bool state);

		void setDpad(XInputControl pad, bool state);
		void setDpad(bool up, bool down, bool left, bool right, bool useSOCD=true);

		void setTrigger(XInputControl trigger, int32_t val);

		void setJoystick(XInputControl joy, int32_t x, int32_t y);
		void setJoystick(XInputControl joy, bool up, bool down, bool left, bool right, bool useSOCD=true);
		void setJoystickX(XInputControl joy, int32_t x, bool invert=false);
		void setJoystickY(XInputControl joy, int32_t y, bool invert=false);

		void releaseAll();

		// Auto-Send Data
		void setAutoSend(bool a);

		// Read Control Surfaces
		bool getButton(uint8_t button) const;
		bool getDpad(XInputControl dpad) const;
		uint8_t getTrigger(XInputControl trigger) const;
		int16_t getJoystickX(XInputControl joy) const;
		int16_t getJoystickY(XInputControl joy) const;

		// Received Data
		uint8_t getPlayer() const;  // Player # assigned to the controller (0 is unassigned)

		uint16_t getRumble() const;  // Rumble motors. MSB is large weight, LSB is small
		uint8_t  getRumbleLeft() const;  // Large rumble motor, left grip
		uint8_t  getRumbleRight() const; // Small rumble motor, right grip

		XInputLEDPattern getLEDPattern() const;  // Returns LED pattern type




		// USB IO

		int send();
		int receive();

		// Control Input Ranges
		struct Range { int32_t min; int32_t max; };

		void setTriggerRange(int32_t rangeMin, int32_t rangeMax);
		void setJoystickRange(int32_t rangeMin, int32_t rangeMax);
		void setRange(XInputControl ctrl, int32_t rangeMin, int32_t rangeMax);

		// Setup
		void reset();

	virtual ~Xbox();

private:

	// Sent Data
		uint8_t tx[20];  // USB transmit data
		bool newData;  // Flag for tx data changed
		bool autoSendOption;  // Flag for automatically sending data

		void setJoystickDirect(XInputControl joy, int16_t x, int16_t y);

		void inline autosend() {
			if (autoSendOption) { send(); }
		}

		// Received Data
		volatile uint8_t player;  // Gamepad player #, buffered
		volatile uint8_t rumble[2];  // Rumble motor data in, buffered
		volatile XInputLEDPattern ledPattern;  // LED pattern data in, buffered


		void parseLED(uint8_t leds);  // Parse LED data and set pattern/player data

		// Control Input Ranges
		Range rangeTrigLeft, rangeTrigRight, rangeJoyLeft, rangeJoyRight;
		Range * getRangeFromEnum(XInputControl ctrl);
		static int32_t rescaleInput(int32_t val, const Range& in, const Range &out);
		static int16_t invertInput(int16_t val, const Range& range);

};

#endif /* INC_XBOX_H_ */
