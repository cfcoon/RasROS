//Extracted from https://github.com/LdB-ECM/Raspberry-Pi/blob/master/FreeRTOSv10.1.1/loader/RaspberryPi/rpi-SmartStart.c

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{																			}
{       Filename: MemMapIO.c aka "rpi-SmartStart"							}
{       Copyright(c): Leon de Boer(LdB) 2017, 2018							}
{       Version: 2.12														}
{																			}
{***************[ THIS CODE IS FREEWARE UNDER CC Attribution]***************}
{																            }
{     This sourcecode is released for the purpose to promote programming    }
{  on the Raspberry Pi. You may redistribute it and/or modify with the      }
{  following disclaimer and condition.                                      }
{																            }
{      The SOURCE CODE is distributed "AS IS" WITHOUT WARRANTIES AS TO      }
{   PERFORMANCE OF MERCHANTABILITY WHETHER EXPRESSED OR IMPLIED.            }
{   Redistributions of source code must retain the copyright notices to     }
{   maintain the author credit (attribution) .								}
{																			}
{***************************************************************************}
{                                                                           }
{      This code provides a 32bit or 64bit C wrapper around the assembler   }
{  stub code. In AARCH32 that file is SmartStart32.S, while in AARCH64 the  }
{  file is SmartStart64.S.													}
{	   There file also provides access to the basic hardware of the PI.     }
{  It is also the easy point to insert a couple of important very useful    }
{  Macros that make C development much easier.		        				}
{																            }
{++++++++++++++++++++++++[ REVISIONS ]++++++++++++++++++++++++++++++++++++++}
{  2.08 Added setIrqFuncAddress & setFiqFuncAddress							}
{  2.09 Added Hard/Soft float compiler support								}
{  2.10 Context Switch support API calls added								}
{  2.11 MiniUart, PL011 Uart and console uart support added					}
{  2.12 New FIQ, DAIF flag support added									}
{++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

//Contains utility functions to access GPIO pins, Mailbox system, System timer and timer interrupts, 
//Framebuffer, CPU addresses to GPU (bus) address conversions and vice versa, GET/PUT functions to
//registers, UART and console using UART functions, LED light acceess, and CPU ID and CPU speed setting.


#include <stdbool.h>		// C standard unit needed for bool and true/false
#include <stdint.h>			// C standard unit needed for uint8_t, uint32_t, etc
#include <stdarg.h>			// C standard unit needed for variadic functions
#include <string.h>								// Needed for strlen	

#include "MemMapIO.h"						// This units header


/***************************************************************************}
{				   ARM CPU ID STRINGS THAT WILL BE RETURNED				    }
****************************************************************************/
static const char* ARM6_STR = "arm1176jzf-s";
static const char* ARM7_STR = "cortex-a7";
static const char* ARM8_STR = "cortex-a53";
static const char* ARMX_STR = "unknown ARM cpu";

/***************************************************************************}
{                       PUBLIC C INTERFACE ROUTINES                         }
{***************************************************************************/

//Screen Initialization (was part of PiConsole_Init in original LdB file -- this
//allows other graphics systems to be used).
//
//Set screen to given Width, Height and color depth in number of bits (or 0 width or height
//to set to the max), returning true if successful with the set size in Width/Height/Depth,
//framebuffer Pitch (pixels per horizontal line which could be more than width), and
//memory location of the GPU's memory-mapped frame buffer. False if something failed, with
//returned values undefined
bool InitScreen(uint32_t *pWidth, uint32_t *pHeight, uint32_t *pDepth, uint32_t *pPitch, uint32_t *pFrameBuffer)
{
	if (!pWidth || !pHeight || !pDepth || !pPitch || !pFrameBuffer)
		return false;
	
	uint32_t buffer[23];
	if ((*pWidth == 0) || (*pHeight == 0)) {// Has auto width or height been requested
		if(mailbox_tag_message(&buffer[0],
			5,
			MAILBOX_TAG_GET_PHYSICAL_WIDTH_HEIGHT,
			8,
			0,
			0,
			0)) {
			// Get current width and height of screen
			if(*pWidth == 0) *pWidth = buffer[3];    						// Width passed in as zero set set current screen width
			if(*pHeight == 0) *pHeight = buffer[4];    					// Height passed in as zero set set current screen height
		} else return false;    										// For some reason get screen physical failed
	}
	if (*pDepth == 0) {
		 // Has auto colour depth been requested
		if(mailbox_tag_message(&buffer[0],
			4,
			MAILBOX_TAG_GET_COLOUR_DEPTH,
			4,
			4,
			0)) {
			// Get current colour depth of screen
			*pDepth = buffer[3];    										// Depth passed in as zero set set current screen colour depth
		} else return false;    										// For some reason get screen depth failed
	}
	if (!mailbox_tag_message(&buffer[0],
		23,
		MAILBOX_TAG_SET_PHYSICAL_WIDTH_HEIGHT,
		8,
		8,
		*pWidth,
		*pHeight,
		MAILBOX_TAG_SET_VIRTUAL_WIDTH_HEIGHT,
		8,
		8,
		*pWidth,
		*pHeight,
		   //TODO could double size and use second as backbuffer...
		MAILBOX_TAG_SET_COLOUR_DEPTH,
		4,
		4,
		*pDepth,
		MAILBOX_TAG_ALLOCATE_FRAMEBUFFER,
		8,
		4,
		16,
		0,
		MAILBOX_TAG_GET_PITCH,
		4,
		0,
		0))							// Attempt to set the requested settings
		{	
			return false;    											// The requesting settings failed 
		}
	*pFrameBuffer = GPUaddrToARMaddr(buffer[17]);  				// Transfer the frame buffer
	*pPitch = buffer[22]; 											// Transfer the line pitch
	return true;
}


/*==========================================================================}
{			 PUBLIC CPU ID ROUTINES PROVIDED BY RPi-SmartStart API			}
{==========================================================================*/

/*-[ RPi_CpuIdString]-------------------------------------------------------}
. Returns the CPU id string of the CPU auto-detected by the SmartStart code 
.--------------------------------------------------------------------------*/
const char* RPi_CpuIdString(void)
{
	switch (RPi_CpuId.PartNumber) 
	{
	case 0xb76:														// ARM 6 CPU
		return ARM6_STR; 											// Return the ARM6 string
	case 0xc07 : 														// ARM 7 CPU
		return ARM7_STR; 											// Return the ARM7 string
	case 0xd03 : 														// ARM 8 CPU
		return ARM8_STR; 											// Return the ARM string
	default :
		return ARMX_STR; 											// Unknown CPU
	}																// End switch RPi_CpuId.PartNumber
}

/*==========================================================================}
{			 PUBLIC GPIO ROUTINES PROVIDED BY RPi-SmartStart API			}
{==========================================================================*/
#define MAX_GPIO_NUM 54												// GPIO 0..53 are valid


/*-[gpio_setup]-------------------------------------------------------------}
. Given a valid GPIO port number and mode sets GPIO to given mode
. RETURN: true for success, false for any failure
.--------------------------------------------------------------------------*/
bool gpio_setup(unsigned int gpio, GPIOMODE mode) 
{
	if ((gpio < MAX_GPIO_NUM) && (mode >= 0)  && (mode <= GPIO_ALTFUNC3))// Check GPIO pin number and mode valid
	{
		uint32_t bit = ((gpio % 10) * 3); 							// Create bit mask
		uint32_t mem = GPIO->GPFSEL[gpio / 10]; 						// Read register
		mem &= ~(7 << bit); 											// Clear GPIO mode bits for that port
		mem |= (mode << bit); 										// Logical OR GPIO mode bits
		GPIO->GPFSEL[gpio / 10] = mem; 								// Write value to register
		
		return true; 												// Return true
	}
	return false;													// Return false is something is invalid
}

/*-[gpio_output]------------------------------------------------------------}
. Given a valid GPIO port number the output is set high(true) or Low (false)
. RETURN: true for success, false for any failure
.--------------------------------------------------------------------------*/
bool gpio_output(unsigned int gpio, bool on) 
{
	if (gpio < MAX_GPIO_NUM) 										// Check GPIO pin number valid
		{
			volatile uint32_t* p;
			uint32_t bit = 1 << (gpio % 32); 							// Create mask bit
			p = (on) ? &GPIO->GPSET[0] : &GPIO->GPCLR[0]; 				// Set pointer depending on on/off state
			p[gpio / 32] = bit; 											// Output bit to register number	
			return true; 												// Return true
		}
	return false;													// Return false
}

/*-[gpio_input]-------------------------------------------------------------}
. Reads the actual level of the GPIO port number
. RETURN: true = GPIO input high, false = GPIO input low
.--------------------------------------------------------------------------*/
bool gpio_input(unsigned int gpio) 
{
	if (gpio < MAX_GPIO_NUM)										// Check GPIO pin number valid
		{
			uint32_t bit = 1 << (gpio % 32); 							// Create mask bit
			uint32_t mem = GPIO->GPLEV[gpio / 32]; 						// Read port level
			if(mem & bit) return true; 									// Return true if bit set
		}
	return false;													// Return false
}

/*-[gpio_checkEvent]-------------------------------------------------------}
. Checks the given GPIO port number for an event/irq flag.
. RETURN: true for event occured, false for no event
.-------------------------------------------------------------------------*/
bool gpio_checkEvent(unsigned int gpio) 
{
	if (gpio < MAX_GPIO_NUM)										// Check GPIO pin number valid
		{
			uint32_t bit = 1 << (gpio % 32); 							// Create mask bit
			uint32_t mem = GPIO->GPEDS[gpio / 32]; 						// Read event detect status register
			if(mem & bit) return true; 									// Return true if bit set
		}
	return false;													// Return false
}

/*-[gpio_clearEvent]-------------------------------------------------------}
. Clears the given GPIO port number event/irq flag.
. RETURN: true for success, false for any failure
.-------------------------------------------------------------------------*/
bool gpio_clearEvent(unsigned int gpio) 
{
	if (gpio < MAX_GPIO_NUM)										// Check GPIO pin number valid
		{
			uint32_t bit = 1 << (gpio % 32); 							// Create mask bit
			GPIO->GPEDS[gpio / 32] = bit; 								// Clear the event from GPIO register
			return true; 												// Return true
		}
	return false;													// Return false
}

/*-[gpio_edgeDetect]-------------------------------------------------------}
. Sets GPIO port number edge detection to lifting/falling in Async/Sync mode
. RETURN: true for success, false for any failure
.-------------------------------------------------------------------------*/
bool gpio_edgeDetect(unsigned int gpio, bool lifting, bool Async) 
{
	if (gpio < MAX_GPIO_NUM)										// Check GPIO pin number valid
		{
			volatile uint32_t* p;
			uint32_t bit = 1 << (gpio % 32); 							// Create mask bit
			if(lifting) {
																// Lifting edge detect
				p = (Async) ? &GPIO->GPAREN[0] : &GPIO->GPREN[0]; 		// Select Synchronous/Asynchronous lifting edge detect register
			}
			else {
																		// Falling edge detect
				p = (Async) ? &GPIO->GPAFEN[0] : &GPIO->GPFEN[0]; 		// Select Synchronous/Asynchronous falling edge detect register
			}
			p[gpio / 32] = bit; 											// Set the register with the mask
			return true; 												// Return true
		}
	return false;													// Return false
}

/*-[gpio_fixResistor]------------------------------------------------------}
. Set the GPIO port number with fix resistors to pull up/pull down.
. RETURN: true for success, false for any failure
.-------------------------------------------------------------------------*/
bool gpio_fixResistor(unsigned int gpio, GPIO_FIX_RESISTOR resistor) 
{
	if ((gpio < MAX_GPIO_NUM) && (resistor >= 0) && (resistor <= PULLDOWN))	// Check GPIO pin number and resistor mode valid
		{
			uint32_t regnum = gpio / 32; 								// Create register number
			uint32_t bit = 1 << (gpio % 32); 							// Create mask bit
			GPIO->GPPUD = resistor; 										// Set fixed resistor request to PUD register
			timer_wait(2); 												// Wait 2 microseconds
			GPIO->GPPUDCLK[regnum] = bit; 								// Set the PUD clock bit register
			timer_wait(2); 												// Wait 2 microseconds	
			GPIO->GPPUD = 0; 											// Clear GPIO resistor setting
			GPIO->GPPUDCLK[regnum] = 0; 									// Clear PUDCLK from GPIO
						
			return true; 												// Return true
		}
	return false;													// Return false
}


/*==========================================================================}
{		   PUBLIC TIMER ROUTINES PROVIDED BY RPi-SmartStart API				}
{==========================================================================*/

/*-[timer_getTickCount64]---------------------------------------------------}
. Get 1Mhz ARM system timer tick count in full 64 bit.
. The timer read is as per the Broadcom specification of two 32bit reads
. RETURN: tickcount value as an unsigned 64bit value in microseconds (usec)
.--------------------------------------------------------------------------*/
uint64_t timer_getTickCount64(void)
{
	uint32_t hiCount;
	uint32_t loCount;
	do {
		hiCount = SYSTEMTIMER->TimerHi;  							// Read Arm system timer high count
		loCount = SYSTEMTIMER->TimerLo; 								// Read Arm system timer low count
	} while (hiCount != SYSTEMTIMER->TimerHi);						// Check hi counter hasn't rolled as we did low read
	return (((uint64_t)hiCount << 32) | loCount); 					// Join the 32 bit values to a full 64 bit
}

/*-[timer_Wait]-------------------------------------------------------------}
. This will simply wait the requested number of microseconds before return.
.--------------------------------------------------------------------------*/
void timer_wait(uint64_t usec) 
{
	usec += timer_getTickCount64(); 									// Add current tickcount onto delay .. usec may roll but ok look at next test
	while(timer_getTickCount64() < usec) {}
	;						// Loop on timeout function until timeout 
}

/*-[tick_Difference]--------------------------------------------------------}
. Given two timer tick results it returns the time difference between them.
. If (us1 > us2) it is assumed the timer rolled as we expect (us2 > us1)
.--------------------------------------------------------------------------*/
uint64_t tick_difference(uint64_t us1, uint64_t us2) 
{
	if (us1 > us2) {
														// If timer one is greater than two then timer rolled
		uint64_t td = UINT64_MAX - us1 + 1; 							// Counts left to roll value
		return us2 + td; 											// Add that to new count
	}
	return us2 - us1;												// Return difference between values
}


/*==========================================================================}
{		  PUBLIC PI MAILBOX ROUTINES PROVIDED BY RPi-SmartStart API			}
{==========================================================================*/
#define MAIL_EMPTY	0x40000000		/* Mailbox Status Register: Mailbox Empty */
#define MAIL_FULL	0x80000000		/* Mailbox Status Register: Mailbox Full  */

/*-[mailbox_write]----------------------------------------------------------}
. This will execute the sending of the given data block message thru the
. mailbox system on the given channel. It is normal for a response back so
. usually you need to follow the write up with a read.
. RETURN: True for success, False for failure.
.--------------------------------------------------------------------------*/
bool mailbox_write(MAILBOX_CHANNEL channel, uint32_t message) 
{
	if ((channel >= 0) && (channel <= MB_CHANNEL_GPU))				// Check channel valid
		{
			volatile uint32_t value;
			message &= ~(0xF); 											// Make sure 4 low channel bits are clear 
			message |= channel; 											// OR the channel bits to the value							
			do {
				value = MAILBOX->Status1; 								// Read mailbox1 status from GPU	
			} while ((value & MAIL_FULL) != 0) ;							// Make sure arm mailbox is not full
			MAILBOX->Write1 = message; 									// Write value to mailbox
			return true; 												// Write success
		}
	return false;													// Return false
}

/*-[mailbox_read]-----------------------------------------------------------}
. This will read any pending data on the mailbox system on the given channel.
. RETURN: The read value for success, 0xFEEDDEAD for failure.
.--------------------------------------------------------------------------*/
uint32_t mailbox_read(MAILBOX_CHANNEL channel) 
{
	if ((channel >= 0) && (channel <= MB_CHANNEL_GPU))				// Check channel valid
		{
			volatile uint32_t value;
			do {
				do {
					value = MAILBOX->Status0; 							// Read mailbox0 status
				} while ((value & MAIL_EMPTY) != 0);					// Wait for data in mailbox
				value = MAILBOX->Read0; 									// Read the mailbox	
			} while ((value & 0xF) != channel);							// We have response back
			value &= ~(0xF); 											// Lower 4 low channel bits are not part of message
			return value; 												// Return the value
		}
	return 0xFEEDDEAD;												// Channel was invalid
}

/*-[mailbox_tag_message]----------------------------------------------------}
. This will post and execute the given variadic data onto the tags channel
. on the mailbox system. You must provide the correct number of response
. uint32_t variables and a pointer to the response buffer. You nominate the
. number of data uint32_t for the call and fill the variadic data in. If you
. do not want the response data back the use NULL for response_buf pointer.
. RETURN: True for success and the response data will be set with data
.         False for failure and the response buffer is untouched.
.--------------------------------------------------------------------------*/
bool mailbox_tag_message(uint32_t* response_buf,
						// Pointer to response buffer 
						  uint8_t data_count,
							// Number of uint32_t data following
						  ...)										// Variadic uint32_t values for call
{
	uint32_t __attribute__((aligned(16))) message[32];
	uint32_t addr = (uint32_t)(uintptr_t)&message[0];
	va_list list;
	va_start(list, data_count); 										// Start variadic argument
	message[0] = (data_count + 3) * 4; 								// Size of message needed
	message[data_count + 2] = 0; 									// Set end pointer to zero
	message[1] = 0; 													// Zero response message
	for(int i = 0 ; i < data_count ; i++) {
		message[2 + i] = va_arg(list, uint32_t); 					// Fetch next variadic
	}
	va_end(list); 													// variadic cleanup	
#if __aarch64__ == 1
	__asm volatile("dc civac, %0" : : "r" (addr) : "memory"); 		// Ensure coherence
#endif
	mailbox_write(MB_CHANNEL_TAGS, ARMaddrToGPUaddr((void*)(uintptr_t)addr)); // Write message to mailbox
	mailbox_read(MB_CHANNEL_TAGS); 									// Read the response
#if __aarch64__ == 1
	__asm volatile("dc civac, %0" : : "r" (addr) : "memory"); 		// Ensure coherence
#endif
	if(message[1] == 0x80000000) {
		if (response_buf) {
														// If buffer NULL used then don't want response
			for(int i = 0 ; i < data_count ; i++)
				response_buf[i] = message[2 + i]; 					// Transfer out each response message
		}
		return true;												// message success
	}
	return false;													// Message failed
}


/*==========================================================================}
{	  PUBLIC PI TIMER INTERRUPT ROUTINES PROVIDED BY RPi-SmartStart API		}
{==========================================================================*/

/*-[ClearTimerIrq]----------------------------------------------------------}
. Simply clear the timer interupt by hitting the clear register. Any timer
. irq/fiq interrupt should call this before exiting.
.--------------------------------------------------------------------------*/
void ClearTimerIrq(void)
{
	ARMTIMER->Clear = 0x1; 											// Hit clear register
}

/*-[TimerIrqSetup]----------------------------------------------------------}
. The irq interrupt rate is set to the period in usec between triggers.
. Largest period is around 16 million usec (16 sec) it varies on core speed
. RETURN: TRUE if successful,  FALSE for any failure
. NOTE: RasROS uses this for the freeRTOS main timer IRQ. 
.--------------------------------------------------------------------------*/
bool TimerIrqSetup(uint32_t period_in_us)							// Period between timer interrupts in usec
{
	uint32_t Buffer[5] = { 0 };
	bool resValue = false;
	ARMTIMER->Control.TimerEnable = 0; 								// Make sure clock is stopped, illegal to change anything while running
	if(mailbox_tag_message(&Buffer[0],
		5,
		MAILBOX_TAG_GET_CLOCK_RATE,
		8,
		8,
		4,
		0))												// Get GPU clock (it varies between 200-450Mhz)
	{
		Buffer[4] /= 250; 											// The prescaler divider is set to 250 (based on GPU=250MHz to give 1Mhz clock)
		Buffer[4] /= 10000; 											// Divid by 10000 we are trying to hold some precision should be in low hundreds (160'ish)
		Buffer[4] *= period_in_us; 									// Multiply by the micro seconds result
		Buffer[4] /= 100; 											// This completes the division by 1000000 (done as /10000 and /100)
		if(Buffer[4] != 0) {
													// Invalid divisor of zero will return with fail
			IRQ->EnableBasicIRQs.Enable_Timer_IRQ = 1; 				// Enable the timer interrupt IRQ
			ARMTIMER->Load = Buffer[4]; 								// Set the load value to divisor
			ARMTIMER->Control.Counter32Bit = 1; 						// Counter in 32 bit mode
			ARMTIMER->Control.Prescale = Clkdiv1; 					// Clock divider = 1
			ARMTIMER->Control.TimerIrqEnable = 1; 					// Enable timer irq
			resValue = true; 										// Set success result
		}
		ARMTIMER->Control.TimerEnable = 1; 							// Now start the clock
	}
	return resValue;												// Return result value	
}

/*-[TimerFiqSetup]----------------------------------------------------------}
. Allocates the given TimerFiqHandler function pointer to be the fiq call
. when a timer interrupt occurs. The interrupt rate is set by providing a
. period in usec between triggers of the interrupt. If the FIQ function is
. redirected by another means use 0 which causes handler set to be ignored.
. Largest period is around 16 million usec (16 sec) it varies on core speed
. RETURN: The old function pointer that was in use (will return 0 for 1st).
.--------------------------------------------------------------------------*/
uintptr_t TimerFiqSetup(uint32_t period_in_us,
	 					// Period between timer interrupts in usec
						 void(*ARMaddress)(void))					// Function to call (0 = ignored)
{
	uint32_t Buffer[5] = { 0 };
	uintptr_t oldHandler = 0;
	ARMTIMER->Control.TimerEnable = 0; 								// Make sure clock is stopped, illegal to change anything while running
	if(mailbox_tag_message(&Buffer[0],
		5,
		MAILBOX_TAG_GET_CLOCK_RATE,
		8,
		8,
		4,
		0))												// Get GPU clock (it varies between 200-450Mhz)
	{
		Buffer[4] /= 250; 											// The prescaler divider is set to 250 (based on GPU=250MHz to give 1Mhz clock)
		Buffer[4] /= 10000; 											// Divid by 10000 we are trying to hold some precision should be in low hundreds (160'ish)
		Buffer[4] *= period_in_us; 									// Multiply by the micro seconds result
		Buffer[4] /= 100; 											// This completes the division by 1000000 (done as /10000 and /100)
		if(Buffer[4] != 0) {
													// Invalid divisor of zero will return with fail
			if(ARMaddress) oldHandler = setFiqFuncAddress(ARMaddress); // Change the handler
			ARMTIMER->Load = Buffer[4]; 								// Set the load value to divisor
			ARMTIMER->Control.Counter32Bit = 1; 						// Counter in 32 bit mode
			ARMTIMER->Control.Prescale = Clkdiv1; 					// Clock divider = 1
			ARMTIMER->Control.TimerIrqEnable = 1; 					// Enable timer irq
			IRQ->FIQControl.SelectFIQSource = 64; 					// Select FIQ source as the ARM timer
			IRQ->FIQControl.EnableFIQ = 1; 							// Enable the FIQ
		}
		ARMTIMER->Control.TimerEnable = 1; 							// Now start the clock
	}
	return oldHandler;												// Return old handler	
}

/*-[ClearLocalTimerIrq]-----------------------------------------------------}
. Simply clear the local timer interupt by hitting the clear registers. Any
. irq/fiq local timer interrupt should call this before exiting.
.--------------------------------------------------------------------------*/
void ClearLocalTimerIrq(void)
{
	QA7->TimerClearReload.IntClear = 1; 								// Clear interrupt
	QA7->TimerClearReload.Reload = 1; 								// Reload now
}

/*-[LocalTimerIrqSetup]-----------------------------------------------------}
. The local timer irq interrupt rate is set to the period in usec between
. triggers. On BCM2835 (ARM6) it does not have core timer so this call fails.
. Largest period is around 20 million usec (20 sec) it varies on core speed
. RETURN: TRUE if successful, FALSE for any failure
.--------------------------------------------------------------------------*/
bool LocalTimerSetup(uint32_t period_in_us,
							// Period between timer interrupts in usec
					  uint8_t coreNum)								// Core number
{
	if ((RPi_CpuId.PartNumber != 0xB76) && (coreNum < RPi_CoresReady))// Not an ARM6 cpu and a valid core number
	{
		uint32_t divisor = 384 * period_in_us; 						// Transfer the period * 384
		divisor /= 10; 												// That is divisor required as clock is 38.4Mhz (2*19.2Mhz)
		QA7->TimerRouting.Routing = LOCALTIMER_TO_CORE0_IRQ + coreNum; // Route local timer IRQ to given Core
		QA7->TimerControlStatus.ReloadValue = divisor; 				// Timer period set
		QA7->TimerControlStatus.TimerEnable = 1; 					// Timer enabled
		QA7->TimerControlStatus.IntEnable = 1; 						// Timer IRQ enabled

		QA7->TimerClearReload.IntClear = 1; 							// Clear interrupt
		QA7->TimerClearReload.Reload = 1; 							// Reload now

		QA7->CoreTimerIntControl[coreNum].nCNTPNSIRQ_IRQ = 1; 		// We are in NS EL1 so enable IRQ to core0 that level
		QA7->CoreTimerIntControl[coreNum].nCNTPNSIRQ_FIQ = 0; 		// Make sure FIQ is zero
		return true; 												// Timer successfully set
	}
	return false;
}


/*==========================================================================}
{				           MINIUART ROUTINES								}
{==========================================================================*/

#define AUX_ENABLES     (RPi_IO_Base_Addr+0x00215004)

/*-[miniuart_init]----------------------------------------------------------}
. Initializes the miniuart (NS16550 compatible one) to the given baudrate
. with 8 data bits, no parity and 1 stop bit. On the PI models 1A, 1B, 1B+
. and PI ZeroW this is external GPIO14 & GPIO15 (Pin header 8 & 10).
. RETURN: true if successfuly set, false on any error
.--------------------------------------------------------------------------*/
bool miniuart_init(unsigned int baudrate)
{
	uint32_t Buffer[5] = { 0 };
	if (mailbox_tag_message(&Buffer[0],
		5, 
		MAILBOX_TAG_GET_CLOCK_RATE,
		8,
		8,
		CLK_CORE_ID,
		0)) 			// Get core clock frequency check for fail 
		{
			uint32_t Divisor = (Buffer[4] / (baudrate * 8)) - 1; 		// Calculate divisor
			if(Divisor <= 0xFFFF) {
				PUT32(AUX_ENABLES, 1); 									// Enable miniuart

				MINIUART->CNTL.RXE = 0; 									// Disable receiver
				MINIUART->CNTL.TXE = 0; 									// Disable transmitter

				MINIUART->LCR.DATA_LENGTH = 1; 							// Data length = 8 bits
				MINIUART->MCR.RTS = 0; 									// Set RTS line high
				MINIUART->IIR.RXFIFO_CLEAR = 1; 							// Clear RX FIFO
				MINIUART->IIR.TXFIFO_CLEAR = 1; 							// Clear TX FIFO

				MINIUART->BAUD.DIVISOR = Divisor; 						// Set the divisor

				gpio_setup(14, GPIO_ALTFUNC5); 							// GPIO 14 to ALT FUNC5 mode
				gpio_setup(15, GPIO_ALTFUNC5); 							// GPIO 15 to ALT FUNC5 mode

				MINIUART->CNTL.RXE = 1; 									// Enable receiver
				MINIUART->CNTL.TXE = 1; 									// Enable transmitter
				return true; 											// Return success
			}
		}
	return false;													// Invalid baudrate or mailbox clock error
}

/*-[miniuart_getc]----------------------------------------------------------}
. Wait for an retrieve a character from the uart.
.--------------------------------------------------------------------------*/
char miniuart_getc(void)
{
	while (MINIUART->LSR.RXFDA == 0) {}
	;							// Wait for a character to arrive
	return(MINIUART->IO.DATA); 										// Return that character
}

/*-[miniuart_putc]----------------------------------------------------------}
. Send a character out via uart.
.--------------------------------------------------------------------------*/
void miniuart_putc(char c)
{
	while (MINIUART->LSR.TXFE == 0) {}
	;								// Wait for transmitt buffer to have a space
	MINIUART->IO.DATA = c; 											// Write character to transmit buffer
}

/*-[miniuart_puts]----------------------------------------------------------}
. Send a '\0' terminated character string out via uart.
.--------------------------------------------------------------------------*/
void miniuart_puts(char *str)
{
	if (str) {
																// Make sure string is not null
		while(*str) {
															// For each character that is not \0 terminator
			miniuart_putc(*str++); 									// Output each character
		}
	}
}

/*==========================================================================}
{				           PL011 UART ROUTINES								}
{==========================================================================*/

/*-[pl011_uart_init]--------------------------------------------------------}
. Initializes the pl011 uart to the given baudrate with 8 data, no parity 
. and 1 stop bit. On the PI models 2B, Pi3, Pi3B+, Pi Zero and CM this is 
. external GPIO14 & GPIO15 (Pin header 8 & 10).
. RETURN: true if successfuly set, false on any error
.--------------------------------------------------------------------------*/
bool pl011_uart_init(unsigned int baudrate)
{
	uint32_t Buffer[5] = { 0 };
	PL011UART->CR.Raw32 = 0; 										// Disable all the UART
	gpio_setup(14, GPIO_ALTFUNC0); 									// GPIO 14 to ALT FUNC0 mode
	gpio_setup(15, GPIO_ALTFUNC0); 									// GPIO 15 to ALT FUNC0 mode
	if(mailbox_tag_message(&Buffer[0],
		5,
		MAILBOX_TAG_SET_CLOCK_RATE,
		8,
		8,
		CLK_UART_ID,
		4000000))								// Set UART clock frequency to 4Mhz
	{
		uint32_t divisor, fracpart;
		divisor = 4000000 / (baudrate * 16); 						// Calculate divisor
		PL011UART->IBRD.DIVISOR = divisor; 							// Set the 16 integer divisor
		fracpart = 4000000 - (divisor*baudrate * 16); 				// Fraction left
		fracpart *= 4; 												// fraction part *64 /16 = *4 																		
		fracpart += baudrate / 2; 									// Add half baudrate for rough round
		fracpart /= baudrate; 										// Divid the baud rate
		PL011UART->FBRD.DIVISOR = fracpart; 							// Write the 6bits of fraction
		PL011UART->LCRH.DATALEN = PL011_DATA_8BITS; 					// 8 data bits
		PL011UART->LCRH.PEN = 0; 									// No parity
		PL011UART->LCRH.STP2 = 0; 									// One stop bits AKA not 2 stop bits
		PL011UART->LCRH.FEN = 0; 									// Fifo's enabled
		PL011UART->CR.UARTEN = 1; 									// Uart enable
		PL011UART->CR.RXE = 1; 										// Transmit enable
		PL011UART->CR.TXE = 1; 										// Receive enable
		return true; 												// Return success
	}
	return false;													// Invalid baudrate or mailbox clock error
}

/*-[pl011_uart_getc]--------------------------------------------------------}
. Wait for an retrieve a character from the uart.
.--------------------------------------------------------------------------*/
char pl011_uart_getc(void) 
{
	while (PL011UART->FR.RXFE != 0) {}
	;								// Check recieve fifo is not empty
	return(PL011UART->DR.DATA); 										// Read the receive data
}

/*-[pl011_uart_putc]--------------------------------------------------------}
. Send a character out via uart.
.--------------------------------------------------------------------------*/
void pl011_uart_putc(char c)
{
	while (PL011UART->FR.TXFF != 0) {}
	;								// Check tx fifo is not full
	PL011UART->DR.DATA = c; 											// Transfer character
}

/*-[pl011_uart_puts]--------------------------------------------------------}
. Send a '\0' terminated character string out via uart.
.--------------------------------------------------------------------------*/
void pl011_uart_puts(char *str)
{
	if (str) {
																// Make sure string is not null
		while(*str) {
															// For each character that is not \0 terminator
			pl011_uart_putc(*str++); 								// Output that character
		}
	}
}

/*==========================================================================}
{				         UART CONSOLE ROUTINES								}
{==========================================================================*/
static uint32_t console_uart_baudrate = 0;
static bool(*detected_uart_init)(unsigned int baudrate);
static char(*detected_uart_getc)(void) = 0;
static void(*detected_uart_putc)(char c) = 0;
static void(*detected_uart_puts)(char *str) = 0;

/*-[console_uart_init]------------------------------------------------------}
. This will detect the Pi model at runtime and set the baud rate and console
. uart functions to pins 6 & 8 on header for the detected model. It could be
. either the PL011 or Miniuart port depending on Pi Model but all that is
. shielded from you. So you can just use these calls and know on whichever
. model the code is placed it will come out at header pins.
. RETURN: true if successfuly set, false on any error
.--------------------------------------------------------------------------*/
bool console_uart_init(unsigned int baudrate)
{
	if (detected_uart_init == 0)									// If function is NULL auto detect has not run yet
		{
			uint32_t model[4];
			if (mailbox_tag_message(&model[0],
				4,
				MAILBOX_TAG_GET_BOARD_REVISION,
				4,
				4,
				0))				// Fetch the model revision from mailbox   
				{
					switch (model[3])
					{
						/* These models the miniUART is console out*/
					case MODEL_1A:
					case MODEL_1B:
					case MODEL_1A_PLUS:
					case MODEL_1B_PLUS:
					case MODEL_PI_ZEROW:
						detected_uart_init = miniuart_init; 					// Set the initialize function pointer
						detected_uart_getc = miniuart_getc; 					// Set the getc function pointer
						detected_uart_putc = miniuart_putc; 					// Set the putc function pointer
						detected_uart_puts = miniuart_puts; 					// Set the puts function pointer
						break;

						/* These models the PL011 is console out*/
					case MODEL_PI3B_PLUS:
					case MODEL_2B:
					case MODEL_ALPHA:
					case MODEL_CM:
					case MODEL_PI3:
					case MODEL_PI_ZERO:
						detected_uart_init = pl011_uart_init; 				// Set the initialize function pointer
						detected_uart_getc = pl011_uart_getc; 				// Set the getc function pointer
						detected_uart_putc = pl011_uart_putc; 				// Set the putc function pointer
						detected_uart_puts = pl011_uart_puts; 				// Set the puts function pointer
						break;

						/* If we don't know model guess miniUART is console out*/
					default:
						detected_uart_init = miniuart_init; 					// Set the initialize function pointer
						detected_uart_getc = miniuart_getc; 					// Set the getc function pointer
						detected_uart_putc = miniuart_putc; 					// Set the putc function pointer
						detected_uart_puts = miniuart_puts; 					// Set the puts function pointer
					}
				}
		}
	console_uart_baudrate = baudrate; 								// Hold the baudrate set
	return (detected_uart_init(baudrate)); 							// Simple call redirection function pointer
}

/*-[console_uart_getc]------------------------------------------------------}
. Wait for an retrieve a character from the runtime detected uart that goes
. to Pins 6 & 8 on the 40 way header. It could be minuart or PL011 depending
. on Pi model detected at runtime. If console_uart_init has not been called
. the function will simply immediately return with a character 0.
.--------------------------------------------------------------------------*/
char console_uart_getc(void)
{
	if (detected_uart_getc) return (detected_uart_getc());			// If we have a redirect function call it
		else return 0; 												// Simply return 0
}

/*-[console_uart_putc]------------------------------------------------------}
. Writes a character to the runtime detected uart that goes to Pins 6 & 8 on 
. the 40 way header. It could be minuart or PL011 depending on Pi model that
. was detected at runtime. If console_uart_init has not been called then the
. function will simply immediately return.
.--------------------------------------------------------------------------*/
void console_uart_putc(char ch)
{
	if (detected_uart_putc) detected_uart_putc(ch); 					// If we have a redirect function call it
}

/*-[console_uart_puts]------------------------------------------------------}
. Writes a a '\0' terminated character string to the runtime detected uart 
. that goes to Pins 6 & 8 on the 40 way header. It could be minuart or PL011 
. depending on Pi model that was detected at runtime. If console_uart_init 
. has not been called then the function will simply immediately return.
.--------------------------------------------------------------------------*/
void console_uart_puts(char *str)
{
	if (detected_uart_puts) detected_uart_puts(str); 				// If we have a redirect function call it
}


/*==========================================================================}
{				     PUBLIC PI ACTIVITY LED ROUTINES						}
{==========================================================================*/
static bool ModelCheckHasRun = false; 								// Flag set if model check has run
static bool UseExpanderGPIO = false; 								// Flag set if we need to use GPIO expander								
static uint_fast8_t ActivityGPIOPort = 47; 							// Default GPIO for activity led is 47

/*-[set_Activity_LED]-------------------------------------------------------}
. This will set the PI activity LED on or off as requested. The SmartStart
. stub provides the Pi board autodetection so the right GPIO port is used.
. RETURN: True the LED state was successfully change, false otherwise
.--------------------------------------------------------------------------*/
bool set_Activity_LED(bool on) {
	// THIS IS ALL BASED ON PI MODEL HISTORY: 
    // https://www.raspberrypi.org/documentation/hardware/raspberrypi/revision-codes/README.md
	if(!ModelCheckHasRun) {
		uint32_t model[4];
		ModelCheckHasRun = true; 									// Set we have run the model check
		if(mailbox_tag_message(&model[0],
			4,
			MAILBOX_TAG_GET_BOARD_REVISION,
			4,
			4,
			0))				// Fetch the model revision from mailbox   
		{
			model[3] &= 0x00FFFFFF; 									// Mask off the warranty upper 8 bits
			if((model[3] >= 0x0002) && (model[3] <= 0x000f))		// These are Model A,B which use GPIO 16
			{
																		// Model A, B return 0x0002 to 0x000F
				ActivityGPIOPort = 16; 								// GPIO port 16 is activity led
				UseExpanderGPIO = false; 							// Dont use expander GPIO
			}
			else if(model[3] < 0xa02082) {
											// These are Pi2, PiZero or Compute models (They may be ARM7 or ARM8)
				ActivityGPIOPort = 47; 								// GPIO port 47 is activity led
				UseExpanderGPIO = false; 							// Dont use expander GPIO
			}
			else if((model[3] == 0xa02082) ||
					  (model[3] == 0xa020a0) ||
					  (model[3] == 0xa22082) ||
					  (model[3] == 0xa32082))						// These are Pi3B series originals (ARM8)
			{
				ActivityGPIOPort = 130; 								// GPIO port 130 is activity led
				UseExpanderGPIO = true; 								// Must use expander GPIO
			}
			else {
																	// These are Pi3B+ series (ARM8)
				ActivityGPIOPort = 29; 								// GPIO port 29 is activity led
				UseExpanderGPIO = false; 							// Don't use expander GPIO
			}
		} else return (false); 										// Model check message failed
	}
	if (UseExpanderGPIO) {
													// Activity LED uses expander GPIO
		return (mailbox_tag_message(0,
			5,
			MAILBOX_TAG_SET_GPIO_STATE,
			8,
			8,
			ActivityGPIOPort,
			(uint32_t)on)); 					// Mailbox message,set GPIO port 130, on/off
	}
	else gpio_output(ActivityGPIOPort, on); 						// Not using GPIO expander so use standard GPIO port
	return (true); 													// Return success
}


/*==========================================================================}
{				     PUBLIC ARM CPU SPEED SET ROUTINES						}
{==========================================================================*/

/*-[ARM_setmaxspeed]--------------------------------------------------------}
. This will set the ARM cpu to the maximum. You can optionally print confirm
. message to screen or uart etc by providing a print function handler.
. The print handler can be screen or a uart/usb/eth handler for monitoring.
. RETURN: True maxium speed was successfully set, false otherwise
.--------------------------------------------------------------------------*/
bool ARM_setmaxspeed(int(*prn_handler)(const char *fmt, ...)) {
	uint32_t Buffer[5] = { 0 };
	if (mailbox_tag_message(&Buffer[0], 5, MAILBOX_TAG_GET_MAX_CLOCK_RATE, 8, 8, CLK_ARM_ID, 0))
		if (mailbox_tag_message(&Buffer[0], 5, MAILBOX_TAG_SET_CLOCK_RATE, 8, 8, CLK_ARM_ID, Buffer[4])) {
			/* Clunky issue on PI baud rate can change after ARM speed changed */
			/* This resets baudrate on console uart if it has already been set */
			if (console_uart_baudrate != 0) {
										// If console baudrate was set
				console_uart_init(console_uart_baudrate); 			// Reset it for new clock speed
			}
			if (prn_handler) prn_handler("CPU frequency set to %u MHZ\n", Buffer[4]/ 1000000ul );
			return true;											// Return success
		}
	return false;													// Max speed set failed
}


/*==========================================================================}
{				      SMARTSTART DISPLAY ROUTINES							}
{==========================================================================*/

/*-[displaySmartStart]------------------------------------------------------}
. Will print 2 lines of basic smart start details to given print handler
. The print handler can be screen or a uart/usb/eth handler for monitoring.
.--------------------------------------------------------------------------*/
void displaySmartStart(int(*prn_handler)(const char *fmt, ...)) {
	if (prn_handler) {
		prn_handler("SmartStart v%x.%i%i, ARM%d AARCH%d code, CPU: %#03X, Cores: %u FPU: %s\n",
			(unsigned int)(RPi_SmartStartVer.HiVersion),
			(unsigned int)(RPi_SmartStartVer.LoVersion >> 8),
			(unsigned int)(RPi_SmartStartVer.LoVersion & 0xFF),
			(unsigned int)RPi_CompileMode.ArmCodeTarget,
			(unsigned int)RPi_CompileMode.AArchMode * 32 + 32,
			(unsigned int)RPi_CpuId.PartNumber,
			(unsigned int)RPi_CoresReady,
			RPi_UsingFPU ? "HARD" : "SOFT");
	}
}


/* Increase program data space. As malloc and related functions depend on this,
it is useful to have a working implementation. The following suffices for a
standalone system; it exploits the symbol _end automatically defined by the
GNU linker. It is defined with a weak reference so any actual implementation
will override */
#include <sys/types.h>
caddr_t __attribute__((weak)) _sbrk(int incr)
{
	extern char _end;
	static char* heap_end = 0;
	char* prev_heap_end;

	if (heap_end == 0)
		heap_end = &_end;

	prev_heap_end = heap_end;
	heap_end += incr;

	return (caddr_t)prev_heap_end;
}
