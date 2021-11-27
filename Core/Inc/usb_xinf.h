/*
 * usb_xinf.h
 *
 *  Created on: 17 lis 2021
 *      Author: patry
 */

#ifndef INC_USB_XINF_H_
#define INC_USB_XINF_H_


#include "stdint.h"

#include "usb_device.h"

//#if defined(XINPUT_INTERFACE)

#include <inttypes.h>
#include <stdbool.h>

#ifdef __cplusplus

extern "C" {

#endif
// C language implementation
void usb_xinput_init();
uint16_t usb_xinput_available();
uint8_t usb_xinput_send(uint8_t *buffer, uint8_t nbytes);
uint8_t usb_xinput_recv(uint8_t *buffer, uint8_t nbytes);

#ifdef __cplusplus

}

#endif

#ifdef __cplusplus

class XUSB
{
public:
	static void init(){ usb_xinput_init(); }
	static uint16_t available() { return usb_xinput_available(); }
	static uint8_t send(uint8_t *buffer, uint8_t nbytes) { return usb_xinput_send(buffer, nbytes); }
	static uint8_t recv(uint8_t *buffer, uint8_t nbytes) { return usb_xinput_recv(buffer, nbytes); }
};


#endif


#endif /* INC_USB_XINF_H_ */
