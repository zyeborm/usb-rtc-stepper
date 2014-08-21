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
 *  Main source file for the VirtualSerial demo. This file contains the main tasks of
 *  the demo and is responsible for the initial application hardware configuration.
 */

#include "mains-powered-stepper.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include <util/delay.h>

#define RX_ARRAY_SIZE 15

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */

volatile uint8_t timer_ticked;
volatile uint16_t timer1_buffer;
volatile uint16_t timer3_buffer;
volatile uint8_t USB_Connected;

#define ACCELERATION 32 //RPS per 100hz tick, 
#define POT_L_MUX 0b01000001 //adc1 left adjust vcc vref
#define POT_R_MUX 0b01000000 //adc0 right adjust vcc vref

USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber   = 0,
				.DataINEndpoint           =
					{
						.Address          = CDC_TX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.DataOUTEndpoint =
					{
						.Address          = CDC_RX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.NotificationEndpoint =
					{
						.Address          = CDC_NOTIFICATION_EPADDR,
						.Size             = CDC_NOTIFICATION_EPSIZE,
						.Banks            = 1,
					},
			},
	};

/** Standard file stream for the CDC interface when set up, so that the virtual CDC COM port can be
 *  used like any regular character stream in the C APIs
 */
static FILE USBSerialStream;


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();
USB_Connected = 0;
char rx_buffer[RX_ARRAY_SIZE];
memset(rx_buffer,'\0',RX_ARRAY_SIZE); 
	
	/* Create a regular character stream for the interface so that it can be used with the stdio.h functions */
	CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);


	GlobalInterruptEnable();
  _delay_ms(1000); //settle
  if (USB_Connected == 1)
  {
    fprintf(&USBSerialStream,"Mains Powered stepper V 1\r\n");
  }
  

  CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
	USB_USBTask();
	for (;;)
	{
	  static uint8_t recieve_ptr = 0;
	  uint8_t packet_rxed = 0;
	  
    if (timer_ticked == 1)
    {
       static uint8_t flasher;  
       flasher++;
       if (flasher == 1) //10 Hz
       {
         /*human friendly output
         fprintf(&USBSerialStream,"%c[2J", 0x1B);
         fprintf(&USBSerialStream,"SET 1 = %li (%u) : 2 = %li (%u)\r\n", inner_current_RPS,ICR3,outer_current_RPS,ICR1);
         fprintf(&USBSerialStream,"CUR 1 = %li : 2 = %li\r\n", inner_set_RPS,outer_set_RPS);
         fprintf(&USBSerialStream,"POT L = %u : R = %u\r\n", pot_l,pot_r);           
         fprintf(&USBSerialStream,"Counterspin # : %s\r\n",rx_buffer);
         */

         C_FLIPBIT(FLASHER_LED);  
         C_FLIPBIT(TIMER1_PWM);  
         flasher = 0;
       } 
       timer_ticked = 0;
       
       ADCSRA |= (1 << ADSC);  // Enable ADC
    }

		/* Recieve Data*/
		if (CDC_Device_BytesReceived(&VirtualSerial_CDC_Interface)) 
    { 

	  
      rx_buffer[recieve_ptr] = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);   

      if (rx_buffer[recieve_ptr] == '\r')
      {
        rx_buffer[recieve_ptr] = '\0'; //end the string on rx of CR
        packet_rxed = 1;
      }


   	  if (recieve_ptr >= RX_ARRAY_SIZE) // too much data
	    {
	      recieve_ptr = 0;	      
	      fputs("\r\nRX Overflow\r\n", &USBSerialStream);
	    } else {
	      recieve_ptr++;
	    }
	    
    } 
   

    if (packet_rxed == 1)
    {
  	 /* if (sscanf(rx_buffer,"%li %li" SCNu16,&inner_set_RPS,&outer_set_RPS) == 2)
    	{
      //should handle sscanf better here
      } else {
        fprintf(&USBSerialStream,"\r\nInvalid Data\r\n");
      }
      recieve_ptr = 0;
      memset(rx_buffer,'\0',RX_ARRAY_SIZE); 
      packet_rxed = 0;      
    */
    }
    
		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Hardware Initialization */
	MCUCR=(1<<JTD); 
  MCUCR=(1<<JTD);
  
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

  SETBIT(FLASHER_DDR,FLASHER_PIN);
  C_SETBIT(FLASHER_LED);

  SETBIT(TIMER1_PWM_DDR,TIMER1_PWM_PIN);
  C_SETBIT(TIMER1_PWM);  

  SETBIT(DIRECTION_DDR,DIRECTION_PIN);
  C_SETBIT(DIRECTION);

  SETBIT(SLEEP_DDR,SLEEP_PIN);
  C_SETBIT(SLEEP_DRV);


//  SETBIT(FLASHER2_DDR,FLASHER2_PIN);
//  C_SETBIT(FLASHER2_LED);
  
 
 /* SETBIT(TIMER3_PWM_DDR,TIMER3_PWM_PIN);
  C_SETBIT(TIMER3_PWM);

  
  SETBIT(INNER_DIRECTION_DDR,INNER_DIRECTION_PIN);
  C_SETBIT(INNER_DIRECTION);

  SETBIT(OUTER_DIRECTION_DDR,OUTER_DIRECTION_PIN);
  C_SETBIT(OUTER_DIRECTION);
    
  CLEARBIT(BT_OUTER_DDR,BT_OUTER_PIN);
  C_SETBIT(BT_OUTER_OUT); //pullup  

  CLEARBIT(BT_LEFT_AUX_DDR,BT_LEFT_AUX_PIN);
  C_SETBIT(BT_LEFT_AUX_OUT); //pull up

  CLEARBIT(BT_RIGHT_AUX_DDR,BT_RIGHT_AUX_PIN);
  C_SETBIT(BT_RIGHT_AUX_OUT); //pull up
    
  CLEARBIT(MAN_MODE_DDR,MAN_MODE_PIN);
  C_SETBIT(MAN_MODE_OUT); //pullup  

  CLEARBIT(BT_INNER_DDR,BT_INNER_PIN);
  C_SETBIT(BT_INNER_OUT); //pullup   
  
  TCCR3A |= (1 << WGM31) | ( 1 << COM3A1);  //set for fast PWM with ICN3 as the top, output enabled set on top clear on match
  TCCR3B |= (1 << CS30)| (1 << WGM33)|(1 << WGM32); //no prescaler, set for fast PWM with ocr3a as the top
  TIMSK3 |= (1<<TOIE3); //enable overflow interrupt


  ICR3 = 65534; 
  OCR3A = 32; // put out a fixed pulse output
//  PRR1 |= (1<<PRTIM3);   // disable the timer3 module power     

  TCCR1A |= (1 << WGM11)| ( 1 << COM1A1);  //set for fast PWM with ICR1 as the top
  TCCR1B |= (1 << CS30)| (1 << WGM13)|(1 << WGM12); //no prescaler, set for fast PWM with ocr3a as the top
  TIMSK1 |= (1<<TOIE1); //enable overflow interrupt
  ICR1 = 65534;
  OCR1A = 32;
//  PRR0 |= (1<<PRTIM1);  // disable the timer1 module power    
*/
  
  //timer0 used for timekeeping
  TCCR0A |= (1 << WGM11); //set to CTC with top at OCR0A
  TCCR0B |= (1 << CS02)| ( 1 << CS00); //clck is fosc/1024
  TIMSK0 |= (1<<OCIE0A); //enable interrupt on compare (not overflow)

  OCR0A = 155; //set for 100.16Hz

    
	//Joystick_Init();
	//LEDs_Init();
	USB_Init();
}



/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	//LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	//LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

	//LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo) 
{ 
        static bool PreviousDTRState = false; 
        bool        CurrentDTRState  = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR); 

        /* Check if the DTR line has been asserted */ 
        if (!(PreviousDTRState) && CurrentDTRState) 
        { 
         // PreviousDTRState == False AND CurrentDTRState == True 
         // Host application has Connected to the COM port 
         // Set LED On    
            //LEDs_TurnOffLEDs(LEDS_LED1);         // this actually turns LED on 
                                        // (bug in LED driver for OLIMEX board - LUFA-120219) 
         
         USB_Connected = 1;
        } 
        else 
        { 
        if (PreviousDTRState && !(CurrentDTRState) ) 
        { 
          // PreviousDTRState == True AND CurrentDTRState == False 
          // Host application has Disconnected from the COM port 
          // Set LED Off    
            // LEDs_TurnOnLEDs(LEDS_LED1);      // this actually turns the LED off 
                                    // (bug in LED driver for OLIMEX board - LUFA-120219) 
         
         USB_Connected = 0;         
        }                  
      }          
        PreviousDTRState = CurrentDTRState; 
} 

ISR(TIMER3_OVF_vect)
{
  ICR3 = timer3_buffer;
}

ISR(TIMER1_OVF_vect)
{
  ICR1 = timer1_buffer;
}

ISR(TIMER0_COMPA_vect)
{

 timer_ticked = 1; 
}


