/*
 * usb_xinf.c
 *
 *  Created on: 17 lis 2021
 *      Author: patry
 */


#include "usb_xinf.h"
#include "usbd_customhid.h"

//#include "HardwareSerial.h"

//#ifdef XINPUT_INTERFACE // defined by usb_dev.h -> usb_desc.h
//#if F_CPU >= 20000000

//USBD_HandleTypeDef hUsbDeviceFS;


// Function returns whether the microcontroller's USB
// is configured or not (connected to driver)
/*bool usb_xinput_connected(void)
{
	return usb_configuration;
}*/

// Function to check if packets are available
// to be received on the RX endpoint
uint16_t usb_xinput_available()
{
	return USBD_LL_GetRxDataSize(&hUsbDeviceFS,2);
}


// Function receives packets from the RX endpoint
uint8_t usb_xinput_recv(uint8_t *buffer, uint8_t nbytes)
{
	if (hUsbDeviceFS.pClassData == NULL)
		  {
		    return (uint8_t)USBD_FAIL;
		  }

		 USBD_LL_PrepareReceive(&hUsbDeviceFS,CUSTOM_HID_EPOUT_ADDR,buffer,nbytes);

	return USBD_OK;
}


// Function used to send packets out of the TX endpoint
// This is used to send button reports
uint8_t usb_xinput_send(uint8_t *buffer, uint8_t nbytes)
{
	//while(USBD_LL_IsStallEP()){}

	if (hUsbDeviceFS.pClassData == NULL)
	  {
	    return (uint8_t)USBD_FAIL;
	  }

	USBD_CUSTOM_HID_HandleTypeDef *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData;

	if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
	  {
	    if (hhid->state == CUSTOM_HID_IDLE)
	    {
	      hhid->state = CUSTOM_HID_BUSY;
	      USBD_LL_Transmit(&hUsbDeviceFS,CUSTOM_HID_EPIN_ADDR,buffer,nbytes);
	    }
	    else
	    {
	      return (uint8_t)USBD_BUSY;
	    }
	  }


	return USBD_OK;
}

void usb_xinput_init()
{
	MX_USB_DEVICE_Init();
}
