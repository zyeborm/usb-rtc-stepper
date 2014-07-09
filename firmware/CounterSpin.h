/*
             LUFA Library
     Copyright (C) Dean Camera, 2013.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2013  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Header file for VirtualSerial.c.
 */

#ifndef _VIRTUALSERIAL_H_
#define _VIRTUALSERIAL_H_

	/* Includes: */
		#include <avr/io.h>
		#include <avr/wdt.h>
		#include <avr/power.h>
		#include <avr/interrupt.h>
		#include <string.h>
		#include <stdio.h>
    #include "avr035.h"
		#include "Descriptors.h"

		//#include <LUFA/Drivers/Board/LEDs.h>
		//#include <LUFA/Drivers/Board/Joystick.h>
		#include <LUFA/Drivers/USB/USB.h>

	/* Macros: */
		/** LED mask for the library LED driver, to indicate that the USB interface is not ready. */
		#define LEDMASK_USB_NOTREADY      LEDS_LED1

		/** LED mask for the library LED driver, to indicate that the USB interface is enumerating. */
		#define LEDMASK_USB_ENUMERATING  (LEDS_LED2 | LEDS_LED3)

		/** LED mask for the library LED driver, to indicate that the USB interface is ready. */
		#define LEDMASK_USB_READY        (LEDS_LED2 | LEDS_LED4)

		/** LED mask for the library LED driver, to indicate that an error has occurred in the USB interface. */
		#define LEDMASK_USB_ERROR        (LEDS_LED1 | LEDS_LED3)

	/* Function Prototypes: */
		void SetupHardware(void);
		void CheckJoystickMovement(void);

		void EVENT_USB_Device_Connect(void);
		void EVENT_USB_Device_Disconnect(void);
		void EVENT_USB_Device_ConfigurationChanged(void);
		void EVENT_USB_Device_ControlRequest(void);
		
  #define FLASHER_PORT  PORTB // ye olde "i'm alive" flasher
  #define FLASHER_PIN   PB4
  #define FLASHER_DDR   DDRB
  #define FLASHER_LED  FLASHER_PORT, FLASHER_PIN  		

  #define FLASHER2_PORT  PORTD // ye olde "i'm alive" flasher
  #define FLASHER2_PIN   PD7
  #define FLASHER2_DDR   DDRD
  #define FLASHER2_LED  FLASHER2_PORT, FLASHER2_PIN  	
  
  #define TIMER1_PWM_PORT  PORTB // ye olde "i'm alive" flasher
  #define TIMER1_PWM_PIN   PB5
  #define TIMER1_PWM_DDR   DDRB
  #define TIMER1_PWM  TIMER1_PWM_PORT, TIMER1_PWM_PIN  		

  #define INNER_DIRECTION_PORT  PORTC // ye olde "i'm alive" flasher
  #define INNER_DIRECTION_PIN   PC7
  #define INNER_DIRECTION_DDR   DDRC
  #define INNER_DIRECTION  INNER_DIRECTION_PORT, INNER_DIRECTION_PIN  		

  #define OUTER_DIRECTION_PORT  PORTB // ye olde "i'm alive" flasher
  #define OUTER_DIRECTION_PIN   PB6
  #define OUTER_DIRECTION_DDR   DDRB
  #define OUTER_DIRECTION  OUTER_DIRECTION_PORT, OUTER_DIRECTION_PIN 
    
  #define TIMER3_PWM_PORT  PORTC // ye olde "i'm alive" flasher
  #define TIMER3_PWM_PIN   PC6
  #define TIMER3_PWM_DDR   DDRC
  #define TIMER3_PWM  TIMER3_PWM_PORT, TIMER3_PWM_PIN  		

#define BT_OUTER_PORT  PORTF  //next (menu) button
#define BT_OUTER_PIN   PF4 
#define BT_OUTER_DDR   DDRF
#define BT_OUTER_OUT  BT_OUTER_PORT, BT_OUTER_PIN
#define BT_OUTER  PINF, BT_OUTER_PIN

#define BT_LEFT_AUX_PORT  PORTE  //next (menu) button
#define BT_LEFT_AUX_PIN   PE6
#define BT_LEFT_AUX_DDR   DDRE
#define BT_LEFT_AUX_OUT  BT_LEFT_AUX_PORT, BT_LEFT_AUX_PIN
#define BT_LEFT_AUX  PINE, BT_LEFT_AUX_PIN

#define BT_RIGHT_AUX_PORT  PORTF  //next (menu) button
#define BT_RIGHT_AUX_PIN   PF7
#define BT_RIGHT_AUX_DDR   DDRF
#define BT_RIGHT_AUX_OUT  BT_RIGHT_AUX_PORT, BT_RIGHT_AUX_PIN
#define BT_RIGHT_AUX  PINF, BT_RIGHT_AUX_PIN

  #define MAN_MODE_PORT  PORTF // ye olde "i'm alive" flasher
  #define MAN_MODE_PIN   PF5
  #define MAN_MODE_DDR   DDRF
  #define MAN_MODE_OUT   MAN_MODE_PORT, MAN_MODE_PIN
  #define MAN_MODE  PINF	, MAN_MODE_PIN

  #define BT_INNER_PORT  PORTF // ye olde "i'm alive" flasher
  #define BT_INNER_PIN   PF6
  #define BT_INNER_DDR   DDRF
  #define BT_INNER_OUT   BT_INNER_PORT, BT_INNER_PIN
  #define BT_INNER  PINF	, BT_INNER_PIN

 /* #define BT_OUTER_PORT  PORTF // ye olde "i'm alive" flasher
  #define BT_OUTER_PIN   PF6
  #define BT_OUTER_DDR   DDRF
  #define BT_OUTER_OUT   BT_OUTER_PORT, BT_OUTER_PIN
  #define BT_OUTER  PINF	, BT_OUTER_PIN
 		*/ 		
  

  
  
#endif

