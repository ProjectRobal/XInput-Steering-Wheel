/*
 * Xbox.cpp
 *
 *  Created on: 16 lis 2021
 *      Author: patry
 */

#define lowByte(w) ((w) & 0xff)
#define highByte(w) ((w) >> 8)

#include "Xbox.h"



uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


struct XInputMap_Button {
	constexpr XInputMap_Button(uint8_t i, uint8_t o)
		: index(i), mask(BuildMask(o)) {}
	const uint8_t index;
	const uint8_t mask;

private:
	constexpr static uint8_t BuildMask(uint8_t offset) {
		return (1 << offset);  // Bitmask of bit to flip
	}
};

static const XInputMap_Button Map_DpadUp(2, 0);
static const XInputMap_Button Map_DpadDown(2, 1);
static const XInputMap_Button Map_DpadLeft(2, 2);
static const XInputMap_Button Map_DpadRight(2, 3);
static const XInputMap_Button Map_ButtonStart(2, 4);
static const XInputMap_Button Map_ButtonBack(2, 5);
static const XInputMap_Button Map_ButtonL3(2, 6);
static const XInputMap_Button Map_ButtonR3(2, 7);

static const XInputMap_Button Map_ButtonLB(3, 0);
static const XInputMap_Button Map_ButtonRB(3, 1);
static const XInputMap_Button Map_ButtonLogo(3, 2);
static const XInputMap_Button Map_ButtonA(3, 4);
static const XInputMap_Button Map_ButtonB(3, 5);
static const XInputMap_Button Map_ButtonX(3, 6);
static const XInputMap_Button Map_ButtonY(3, 7);

const XInputMap_Button * getButtonFromEnum(XInputControl ctrl) {
	switch (ctrl) {
	case(DPAD_UP):      return &Map_DpadUp;
	case(DPAD_DOWN):    return &Map_DpadDown;
	case(DPAD_LEFT):    return &Map_DpadLeft;
	case(DPAD_RIGHT):   return &Map_DpadRight;
	case(BUTTON_A):     return &Map_ButtonA;
	case(BUTTON_B):     return &Map_ButtonB;
	case(BUTTON_X):     return &Map_ButtonX;
	case(BUTTON_Y):     return &Map_ButtonY;
	case(BUTTON_LB):    return &Map_ButtonLB;
	case(BUTTON_RB):    return &Map_ButtonRB;
	case(JOY_LEFT):
	case(BUTTON_L3):    return &Map_ButtonL3;
	case(JOY_RIGHT):
	case(BUTTON_R3):    return &Map_ButtonR3;
	case(BUTTON_START): return &Map_ButtonStart;
	case(BUTTON_BACK):  return &Map_ButtonBack;
	case(BUTTON_LOGO):  return &Map_ButtonLogo;
	default: return nullptr;
	}
}

// --------------------------------------------------------
// XInput Trigger Maps                                    |
// (Matches ID to tx index)                               |
// --------------------------------------------------------

struct XInputMap_Trigger {
	constexpr XInputMap_Trigger(uint8_t i)
		: index(i) {}
	static const Xbox::Range range;
	const uint8_t index;
};

const Xbox::Range XInputMap_Trigger::range = { 0, 255 };  // uint8_t

static const XInputMap_Trigger Map_TriggerLeft(4);
static const XInputMap_Trigger Map_TriggerRight(5);

const XInputMap_Trigger * getTriggerFromEnum(XInputControl ctrl) {
	switch (ctrl) {
	case(TRIGGER_LEFT): return &Map_TriggerLeft;
	case(TRIGGER_RIGHT): return &Map_TriggerRight;
	default: return nullptr;
	}
}

// --------------------------------------------------------
// XInput Joystick Maps                                   |
// (Matches ID to tx x/y high/low indices)                |
// --------------------------------------------------------

struct XInputMap_Joystick {
	constexpr XInputMap_Joystick(uint8_t xl, uint8_t xh, uint8_t yl, uint8_t yh)
		: x_low(xl), x_high(xh), y_low(yl), y_high(yh) {}
	static const Xbox::Range range;
	const uint8_t x_low;
	const uint8_t x_high;
	const uint8_t y_low;
	const uint8_t y_high;
};

const Xbox::Range XInputMap_Joystick::range = { -32768, 32767 };  // int16_t

static const XInputMap_Joystick Map_JoystickLeft(6, 7, 8, 9);
static const XInputMap_Joystick Map_JoystickRight(10, 11, 12, 13);

const XInputMap_Joystick * getJoyFromEnum(XInputControl ctrl) {
	switch (ctrl) {
	case(JOY_LEFT): return &Map_JoystickLeft;
	case(JOY_RIGHT): return &Map_JoystickRight;
	default: return nullptr;
	}
}

// --------------------------------------------------------
// XInput Rumble Maps                                     |
// (Stores rx index and buffer index for each motor)      |
// --------------------------------------------------------

struct XInputMap_Rumble {
	constexpr XInputMap_Rumble(uint8_t rIndex, uint8_t bIndex)
		: rxIndex(rIndex), bufferIndex(bIndex) {}
	const uint8_t rxIndex;
	const uint8_t bufferIndex;
};

static const XInputMap_Rumble RumbleLeft(3, 0);   // Large motor
static const XInputMap_Rumble RumbleRight(4, 1);  // Small motor

// --------------------------------------------------------
// XInput USB Receive Callback                            |
// --------------------------------------------------------



// --------------------------------------------------------
// Xbox Class (API)                           |
// --------------------------------------------------------

Xbox::Xbox() :
	tx(), rumble() // Zero initialize arrays
{
	this->begin();
	reset();

	while(this->receive());  // flush USB OUT buffer

}

void Xbox::begin() {
	// Empty for now
	//usb_xinput_init();
	XUSB::init();
}

void Xbox::press(uint8_t button) {
	setButton(button, true);
}

void Xbox::release(uint8_t button) {
	setButton(button, false);
}

void Xbox::setButton(uint8_t button, bool state) {
	const XInputMap_Button * buttonData = getButtonFromEnum((XInputControl) button);
	if (buttonData != nullptr) {
		if (getButton(button) == state) return;  // Button hasn't changed

		if (state) { tx[buttonData->index] |= buttonData->mask; }  // Press
		else { tx[buttonData->index] &= ~(buttonData->mask); }  // Release
		newData = true;
		autosend();
	}
	else {
		Range * triggerRange = getRangeFromEnum((XInputControl) button);
		if (triggerRange == nullptr) return;  // Not a trigger (or joystick, but the trigger function will ignore that)
		setTrigger((XInputControl) button, state ? triggerRange->max : triggerRange->min);  // Treat trigger like a button
	}
}

void Xbox::setDpad(XInputControl pad, bool state) {
	setButton(pad, state);
}

void Xbox::setDpad(bool up, bool down, bool left, bool right, bool useSOCD) {
	// Simultaneous Opposite Cardinal Directions (SOCD) Cleaner
	if (useSOCD) {
		if (up && down) { down = false; }  // Up + Down = Up
		if (left && right) { left = false; right = false; }  // Left + Right = Neutral
	}

	const bool autoSendTemp = autoSendOption;  // Save autosend state
	autoSendOption = false;  // Disable temporarily

	setDpad(DPAD_UP, up);
	setDpad(DPAD_DOWN, down);
	setDpad(DPAD_LEFT, left);
	setDpad(DPAD_RIGHT, right);

	autoSendOption = autoSendTemp;  // Re-enable from option
	autosend();
}

void Xbox::setTrigger(XInputControl trigger, int32_t val) {
	const XInputMap_Trigger * triggerData = getTriggerFromEnum(trigger);
	if (triggerData == nullptr) return;  // Not a trigger

	val = rescaleInput(val, *getRangeFromEnum(trigger), XInputMap_Trigger::range);
	if (getTrigger(trigger) == val) return;  // Trigger hasn't changed

	tx[triggerData->index] = val;
	newData = true;
	autosend();
}

void Xbox::setJoystick(XInputControl joy, int32_t x, int32_t y) {
	const XInputMap_Joystick * joyData = getJoyFromEnum(joy);
	if (joyData == nullptr) return;  // Not a joystick

	x = rescaleInput(x, *getRangeFromEnum(joy), XInputMap_Joystick::range);
	y = rescaleInput(y, *getRangeFromEnum(joy), XInputMap_Joystick::range);

	setJoystickDirect(joy, x, y);
}

void Xbox::setJoystickX(XInputControl joy, int32_t x, bool invert) {
	const XInputMap_Joystick * joyData = getJoyFromEnum(joy);
	if (joyData == nullptr) return;  // Not a joystick

	x = rescaleInput(x, *getRangeFromEnum(joy), XInputMap_Joystick::range);
	if (invert) x = invertInput(x, XInputMap_Joystick::range);

	if (getJoystickX(joy) == x) return;  // Axis hasn't changed

	tx[joyData->x_low] = lowByte(x);
	tx[joyData->x_high] = highByte(x);

	newData = true;
	autosend();
}

void Xbox::setJoystickY(XInputControl joy, int32_t y, bool invert) {
	const XInputMap_Joystick * joyData = getJoyFromEnum(joy);
	if (joyData == nullptr) return;  // Not a joystick

	y = rescaleInput(y, *getRangeFromEnum(joy), XInputMap_Joystick::range);
	if (invert) y = invertInput(y, XInputMap_Joystick::range);

	if (getJoystickY(joy) == y) return;  // Axis hasn't changed

	tx[joyData->y_low] = lowByte(y);
	tx[joyData->y_high] = highByte(y);

	newData = true;
	autosend();
}

void Xbox::setJoystick(XInputControl joy, bool up, bool down, bool left, bool right, bool useSOCD) {
	const XInputMap_Joystick * joyData = getJoyFromEnum(joy);
	if (joyData == nullptr) return;  // Not a joystick

	const Range & range = XInputMap_Joystick::range;

	int16_t x = 0;
	int16_t y = 0;

	// Simultaneous Opposite Cardinal Directions (SOCD) Cleaner
	if (useSOCD) {
		if (up && down) { down = false; }  // Up + Down = Up
		if (left && right) { left = false; right = false; }  // Left + Right = Neutral
	}

	// Analog axis means directions are mutually exclusive. Only change the
	// output from '0' if the per-axis inputs are different, in order to
	// avoid the '-1' result from adding the int16 extremes
	if (left != right) {
		if (left == true) { x = range.min; }
		else if (right == true) { x = range.max; }
	}
	if (up != down) {
		if (up == true) { y = range.max; }
		else if (down == true) { y = range.min; }
	}

	setJoystickDirect(joy, x, y);
}

void Xbox::setJoystickDirect(XInputControl joy, int16_t x, int16_t y) {
	const XInputMap_Joystick * joyData = getJoyFromEnum(joy);
	if (joyData == nullptr) return;  // Not a joystick

	if (getJoystickX(joy) != x) {
		tx[joyData->x_low] = lowByte(x);
		tx[joyData->x_high] = highByte(x);
		newData = true;
	}

	if (getJoystickY(joy) != y) {
		tx[joyData->y_low] = lowByte(y);
		tx[joyData->y_high] = highByte(y);
		newData = true;
	}

	autosend();
}

void Xbox::releaseAll() {
	const uint8_t offset = 2;  // Skip message type and packet size
	memset(tx + offset, 0x00, sizeof(tx) - offset);  // Clear TX array
	newData = true;  // Data changed and is unsent
	autosend();
}

void Xbox::setAutoSend(bool a) {
	autoSendOption = a;
}

bool Xbox::getButton(uint8_t button) const {
	const XInputMap_Button* buttonData = getButtonFromEnum((XInputControl) button);
	if (buttonData != nullptr) {
		return tx[buttonData->index] & buttonData->mask;
	}
	const XInputMap_Trigger* triggerData = getTriggerFromEnum((XInputControl) button);
	if (triggerData != nullptr) {
		return getTrigger((XInputControl) button) != 0 ? 1 : 0;
	}
	return 0;  // Not a button or a trigger
}

bool Xbox::getDpad(XInputControl dpad) const {
	return getButton(dpad);
}

uint8_t Xbox::getTrigger(XInputControl trigger) const {
	const XInputMap_Trigger * triggerData = getTriggerFromEnum(trigger);
	if (triggerData == nullptr) return 0;  // Not a trigger
	return tx[triggerData->index];
}

int16_t Xbox::getJoystickX(XInputControl joy) const {
	const XInputMap_Joystick * joyData = getJoyFromEnum(joy);
	if (joyData == nullptr) return 0;  // Not a joystick
	return (tx[joyData->x_high] << 8) | tx[joyData->x_low];
}

int16_t Xbox::getJoystickY(XInputControl joy) const {
	const XInputMap_Joystick * joyData = getJoyFromEnum(joy);
	if (joyData == nullptr) return 0;  // Not a joystick
	return (tx[joyData->y_high] << 8) | tx[joyData->y_low];
}

uint8_t Xbox::getPlayer() const {
	return player;
}

uint16_t Xbox::getRumble() const {
	return rumble[RumbleLeft.bufferIndex] << 8 | rumble[RumbleRight.bufferIndex];
}

uint8_t Xbox::getRumbleLeft() const {
	return rumble[RumbleLeft.bufferIndex];
}

uint8_t Xbox::getRumbleRight() const {
	return rumble[RumbleRight.bufferIndex];
}

XInputLEDPattern Xbox::getLEDPattern() const {
	return ledPattern;
}


//Send an update packet to the PC
int Xbox::send() {
	//if (!newData) return 0;  // TX data hasn't changed //here is problem
	//newData = false;

	return XUSB::send(tx, 20);

}

int Xbox::receive() {

	if (XUSB::available() == 0) {
		return 0;  // No packet available
	}

	// Grab packet and store it in rx array
	uint8_t rx[8];
	const int bytesRecv = XUSB::recv(rx, sizeof(rx));

	// Only process if received 3 or more bytes (min valid packet size)
	if (bytesRecv >= 3) {
		const uint8_t PacketType = rx[0];

		// Rumble Packet
		if (PacketType == (uint8_t)XInputReceiveType::Rumble) {
			rumble[RumbleLeft.bufferIndex] = rx[RumbleLeft.rxIndex];   // Big weight (Left grip)
			rumble[RumbleRight.bufferIndex] = rx[RumbleRight.rxIndex];  // Small weight (Right grip)
		}
		// LED Packet
		else if (PacketType == (uint8_t)XInputReceiveType::LEDs) {
			parseLED(rx[2]);
		}


	}

	return bytesRecv;

}

void Xbox::parseLED(uint8_t leds) {
	if (leds > 0x0D) return;  // Not a known pattern

	ledPattern = (XInputLEDPattern) leds;  // Save pattern
	switch (ledPattern) {
	case(XInputLEDPattern::Off):
	case(XInputLEDPattern::Blinking):
		player = 0;  // Not connected
		break;
	case(XInputLEDPattern::On1):
	case(XInputLEDPattern::Flash1):
		player = 1;
		break;
	case(XInputLEDPattern::On2):
	case(XInputLEDPattern::Flash2):
		player = 2;
		break;
	case(XInputLEDPattern::On3):
	case(XInputLEDPattern::Flash3):
		player = 3;
		break;
	case(XInputLEDPattern::On4):
	case(XInputLEDPattern::Flash4):
		player = 4;
		break;
	default: return;  // Pattern doesn't affect player #
	}
}

Xbox::Range * Xbox::getRangeFromEnum(XInputControl ctrl) {
	switch (ctrl) {
	case(TRIGGER_LEFT): return &rangeTrigLeft;
	case(TRIGGER_RIGHT): return &rangeTrigRight;
	case(JOY_LEFT): return &rangeJoyLeft;
	case(JOY_RIGHT): return &rangeJoyRight;
	default: return nullptr;
	}
}

int32_t Xbox::rescaleInput(int32_t val, const Range& in, const Range& out) {
	if (val <= in.min) return out.min;  // Out of range -
	if (val >= in.max) return out.max;  // Out of range +
	if (in.min == out.min && in.max == out.max) return val;  // Ranges identical
	return map(val, in.min, in.max, out.min, out.max);
}

int16_t Xbox::invertInput(int16_t val, const Range& range) {
	return range.max - val + range.min;
}

void Xbox::setTriggerRange(int32_t rangeMin, int32_t rangeMax) {
	setRange(TRIGGER_LEFT, rangeMin, rangeMax);
	setRange(TRIGGER_RIGHT, rangeMin, rangeMax);
}

void Xbox::setJoystickRange(int32_t rangeMin, int32_t rangeMax) {
	setRange(JOY_LEFT, rangeMin, rangeMax);
	setRange(JOY_RIGHT, rangeMin, rangeMax);
}

void Xbox::setRange(XInputControl ctrl, int32_t rangeMin, int32_t rangeMax) {
	if (rangeMin >= rangeMax) return;  // Error: Max < Min

	Range * range = getRangeFromEnum(ctrl);
	if (range == nullptr) return;  // Not an addressable range

	range->min = rangeMin;
	range->max = rangeMax;
}

// Resets class back to initial values
void Xbox::reset() {
	// Reset control data (tx)
	autoSendOption = false;
	releaseAll();  // Clear TX buffer
	tx[0] = 0x00;  // Set tx message type
	tx[1] = 0x14;  // Set tx packet size (20)

	// Reset received data (rx)
	player = 0;  // Not connected, no player
	memset((void*) rumble, 0x00, sizeof(rumble));  // Clear rumble values
	ledPattern = XInputLEDPattern::Off;  // No LEDs on

	// Reset rescale ranges
	setTriggerRange(XInputMap_Trigger::range.min, XInputMap_Trigger::range.max);
	setJoystickRange(XInputMap_Joystick::range.min, XInputMap_Joystick::range.max);
	this->send();
	// Clear user-set options


}

Xbox::~Xbox() {
	// TODO Auto-generated destructor stub
}

