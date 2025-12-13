#ifndef _SMART_START_H_
#define _SMART_START_H_

//RasROS Also see WLAN/bcm2835.h for the Circle project's version of this... 

#ifdef __cplusplus								// If we are including to a C++
extern "C" {
										// Put extern C directive wrapper around
#endif

//Defines from circle project's synchronize.h, rasppi 1 specific
	
//
// Interrupt control
//
#define	EnableIRQs()		__asm volatile ("cpsie i")
#define	DisableIRQs()		__asm volatile ("cpsid i")
#define	EnableInterrupts()	EnableIRQs()			// deprecated
#define	DisableInterrupts()	DisableIRQs()			// deprecated

#define	EnableFIQs()		__asm volatile ("cpsie f")
#define	DisableFIQs()		__asm volatile ("cpsid f")

//
// Cache control
//
#define DATA_CACHE_LINE_LENGTH_MIN	32	// from CTR
#define DATA_CACHE_LINE_LENGTH_MAX	32

#define InvalidateInstructionCache()	\
					asm volatile ("mcr p15, 0, %0, c7, c5,  0" : : "r" (0) : "memory")
#define FlushPrefetchBuffer()	asm volatile ("mcr p15, 0, %0, c7, c5,  4" : : "r" (0) : "memory")
#define FlushBranchTargetCache()	\
									asm volatile ("mcr p15, 0, %0, c7, c5,  6" : : "r" (0) : "memory")

									// NOTE: Data cache operations include a DataSyncBarrier
#define InvalidateDataCache()	asm volatile ("mcr p15, 0, %0, c7, c6,  0\n" \
														      "mcr p15, 0, %0, c7, c10, 4\n" : : "r" (0) : "memory")
#define CleanDataCache()	asm volatile ("mcr p15, 0, %0, c7, c10, 0\n" \
														      					      "mcr p15, 0, %0, c7, c10, 4\n" : : "r" (0) : "memory")

//
// Barriers
//
#define DataSyncBarrier()	__asm volatile ("mcr p15, 0, %0, c7, c10, 4" : : "r" (0) : "memory")
#define DataMemBarrier() 	__asm volatile ("mcr p15, 0, %0, c7, c10, 5" : : "r" (0) : "memory")

#define InstructionSyncBarrier() FlushPrefetchBuffer()
#define InstructionMemBarrier()	FlushPrefetchBuffer()

#define PeripheralEntry()	DataSyncBarrier()
#define PeripheralExit()	DataMemBarrier()

#define CompilerBarrier()	asm volatile ("" ::: "memory")

//
// Cache alignment
//
#define CACHE_ALIGN			ALIGN (DATA_CACHE_LINE_LENGTH_MAX)

#define CACHE_ALIGN_SIZE(type, num)	(((  ((num)*sizeof (type) - 1)		\
					   | (DATA_CACHE_LINE_LENGTH_MAX-1)	\
					  ) + 1) / sizeof (type))

#define IS_CACHE_ALIGNED(ptr, size)	(   ((uintptr) (ptr) & (DATA_CACHE_LINE_LENGTH_MAX-1)) == 0 \
					 && ((size) & (DATA_CACHE_LINE_LENGTH_MAX-1)) == 0)

#define DMA_BUFFER(type, name, num)	type name[CACHE_ALIGN_SIZE (type, num)] CACHE_ALIGN

#define be2le32 __builtin_bswap32    //big-to-little endian
#define le2be32 __builtin_bswap32   
		
// Support for BE to LE conversion
#ifdef __GNUC__
#define byte_swap __builtin_bswap32
#else
	static inline uint32_t byte_swap(uint32_t in)
	{
		uint32_t b0 = in & 0xff;
		uint32_t b1 = (in >> 8) & 0xff;
		uint32_t b2 = (in >> 16) & 0xff;
		uint32_t b3 = (in >> 24) & 0xff;
		uint32_t ret = (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
		return ret;
	}
#endif // __GNUC__
	
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{																			}			
{       Filename: rpi-smartstart.h											}
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
{++++++++++++++++++++++++[ REVISIONS ]++++++++++++++++++++++++++++++++++++++}
{  2.08 Added setIrqFuncAddress & setFiqFuncAddress							}
{  2.09 Added Hard/Soft float compiler support								}
{  2.10 Context Switch support API calls added								}
{  2.11 MiniUart, PL011 Uart and console uart support added					}
{  2.12 New FIQ, DAIF flag support added									}
{++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <stdbool.h>		// C standard unit needed for bool and true/false
#include <stdint.h>			// C standard unit needed for uint8_t, uint32_t, etc
#include <stdarg.h>			// C standard unit needed for variadic functions

/***************************************************************************}
{		  PUBLIC MACROS MUCH AS WE HATE THEM SOMETIMES YOU NEED THEM        }
{***************************************************************************/

/* Most versions of C don't have _countof macro so provide it if not available */
#if !defined(_countof)
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0])) 
#endif

/* As we are compiling for Raspberry Pi if winmain make it main */
#define WinMain(...) kernel_main(uint32_t r0, uint32_t r1, uint32_t atags)



/***************************************************************************}
{					     PUBLIC ENUMERATION CONSTANTS			            }
****************************************************************************/

/*--------------------------------------------------------------------------}
;{				  ENUMERATED AN ID FOR DIFFERENT PI MODELS					}
;{-------------------------------------------------------------------------*/
typedef enum {
		MODEL_1A		= 0,
		MODEL_1B		= 1,
		MODEL_1A_PLUS	= 2,
		MODEL_1B_PLUS	= 3,
		MODEL_2B		= 4,
		MODEL_ALPHA		= 5,
		MODEL_CM		= 6, 		// Compute Module
		MODEL_2A        = 7,
		MODEL_PI3		= 8,
		MODEL_PI_ZERO	= 9,
		MODEL_CM3       = 0xA,       // Compute module 3
		MODEL_PI_ZEROW	= 0xC,
		MODEL_PI3B_PLUS = 0xD,
	} RPI_BOARD_TYPE;

	/*--------------------------------------------------------------------------}
	;{	      ENUMERATED FSEL REGISTERS ... BCM2835.PDF MANUAL see page 92		}
	;{-------------------------------------------------------------------------*/
	/* In binary so any error is obvious */
	typedef enum {
		GPIO_INPUT = 0b000,
		 									// 0
		GPIO_OUTPUT = 0b001, 
										// 1
		GPIO_ALTFUNC5 = 0b010,
		 								// 2
		GPIO_ALTFUNC4 = 0b011,
		 								// 3
		GPIO_ALTFUNC0 = 0b100, 
										// 4
		GPIO_ALTFUNC1 = 0b101,
		 								// 5
		GPIO_ALTFUNC2 = 0b110,
		 								// 6
		GPIO_ALTFUNC3 = 0b111,
		 								// 7
	} GPIOMODE;

	/*--------------------------------------------------------------------------}
	;{	    ENUMERATED GPIO FIX RESISTOR ... BCM2835.PDF MANUAL see page 101	}
	;{-------------------------------------------------------------------------*/
	/* In binary so any error is obvious */
	typedef enum {
		NO_RESISTOR = 0b00,
		 									// 0
		PULLUP = 0b01,
		 										// 1
		PULLDOWN = 0b10, 
											// 2
	} GPIO_FIX_RESISTOR;

	/*--------------------------------------------------------------------------}
	{	ENUMERATED TIMER CONTROL PRESCALE ... BCM2835.PDF MANUAL see page 197	}
	{--------------------------------------------------------------------------*/
	/* In binary so any error is obvious */
	typedef enum {
		Clkdiv1 = 0b00, 
												// 0
		Clkdiv16 = 0b01,
		 									// 1
		Clkdiv256 = 0b10,
		 									// 2
		Clkdiv_undefined = 0b11,
		 							// 3 
	} TIMER_PRESCALE;

	/*--------------------------------------------------------------------------}
	{	                  ENUMERATED MAILBOX CHANNELS							}
	{		  https://github.com/raspberrypi/firmware/wiki/Mailboxes			}
	{--------------------------------------------------------------------------*/
	typedef enum {
		MB_CHANNEL_POWER = 0x0,
		 								// Mailbox Channel 0: Power Management Interface 
		MB_CHANNEL_FB = 0x1,
		 								// Mailbox Channel 1: Frame Buffer
		MB_CHANNEL_VUART = 0x2, 
										// Mailbox Channel 2: Virtual UART
		MB_CHANNEL_VCHIQ = 0x3, 
										// Mailbox Channel 3: VCHIQ Interface
		MB_CHANNEL_LEDS = 0x4,
		 								// Mailbox Channel 4: LEDs Interface
		MB_CHANNEL_BUTTONS = 0x5, 
									// Mailbox Channel 5: Buttons Interface
		MB_CHANNEL_TOUCH = 0x6, 
										// Mailbox Channel 6: Touchscreen Interface
		MB_CHANNEL_COUNT = 0x7,
		 								// Mailbox Channel 7: Counter
		MB_CHANNEL_TAGS = 0x8,
		 								// Mailbox Channel 8: Tags (ARM to VC)
		MB_CHANNEL_GPU = 0x9,
		 								// Mailbox Channel 9: GPU (VC to ARM)
	} MAILBOX_CHANNEL;

	/*--------------------------------------------------------------------------}
	{	            ENUMERATED MAILBOX TAG CHANNEL COMMANDS						}
	{  https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface  }
	{--------------------------------------------------------------------------*/
	typedef enum {
		/* Videocore info commands */
		MAILBOX_TAG_GET_VERSION = 0x00000001, 
					// Get firmware revision

		/* Hardware info commands */
		MAILBOX_TAG_GET_BOARD_MODEL = 0x00010001, 
					// Get board model
		MAILBOX_TAG_GET_BOARD_REVISION = 0x00010002,
		 			// Get board revision
		MAILBOX_TAG_GET_BOARD_MAC_ADDRESS = 0x00010003, 
					// Get board MAC address
		MAILBOX_TAG_GET_BOARD_SERIAL = 0x00010004, 
					// Get board serial
		MAILBOX_TAG_GET_ARM_MEMORY = 0x00010005,
		 			// Get ARM memory
		MAILBOX_TAG_GET_VC_MEMORY = 0x00010006,
		 			// Get VC memory
		MAILBOX_TAG_GET_CLOCKS = 0x00010007,
		 			// Get clocks

		/* Power commands */
		MAILBOX_TAG_GET_POWER_STATE = 0x00020001, 
					// Get power state
		MAILBOX_TAG_GET_TIMING = 0x00020002,
		 			// Get timing
		MAILBOX_TAG_SET_POWER_STATE = 0x00028001,
		 			// Set power state

		/* GPIO commands */
		MAILBOX_TAG_GET_GET_GPIO_STATE = 0x00030041, 
					// Get GPIO state
		MAILBOX_TAG_SET_GPIO_STATE = 0x00038041,
		 			// Set GPIO state

		/* Clock commands */
		MAILBOX_TAG_GET_CLOCK_STATE = 0x00030001,
		 			// Get clock state
		MAILBOX_TAG_GET_CLOCK_RATE = 0x00030002,
		 			// Get clock rate
		MAILBOX_TAG_GET_MAX_CLOCK_RATE = 0x00030004, 
					// Get max clock rate
		MAILBOX_TAG_GET_MIN_CLOCK_RATE = 0x00030007, 
					// Get min clock rate
		MAILBOX_TAG_GET_TURBO = 0x00030009,
		 			// Get turbo

		MAILBOX_TAG_SET_CLOCK_STATE = 0x00038001,
		 			// Set clock state
		MAILBOX_TAG_SET_CLOCK_RATE = 0x00038002,
		 			// Set clock rate
		MAILBOX_TAG_SET_TURBO = 0x00038009,
		 			// Set turbo

		/* Voltage commands */
		MAILBOX_TAG_GET_VOLTAGE = 0x00030003, 
					// Get voltage
		MAILBOX_TAG_GET_MAX_VOLTAGE = 0x00030005,
		 			// Get max voltage
		MAILBOX_TAG_GET_MIN_VOLTAGE = 0x00030008, 
					// Get min voltage

		MAILBOX_TAG_SET_VOLTAGE = 0x00038003, 
					// Set voltage

		/* Temperature commands */
		MAILBOX_TAG_GET_TEMPERATURE = 0x00030006,
		 			// Get temperature
		MAILBOX_TAG_GET_MAX_TEMPERATURE = 0x0003000A,
		 			// Get max temperature

		/* Memory commands */
		MAILBOX_TAG_ALLOCATE_MEMORY = 0x0003000C,
		 			// Allocate Memory
		MAILBOX_TAG_LOCK_MEMORY = 0x0003000D, 
					// Lock memory
		MAILBOX_TAG_UNLOCK_MEMORY = 0x0003000E, 
					// Unlock memory
		MAILBOX_TAG_RELEASE_MEMORY = 0x0003000F,
		 			// Release Memory
																	
		/* Execute code commands */
		MAILBOX_TAG_EXECUTE_CODE = 0x00030010,
		 			// Execute code

		/* QPU control commands */
		MAILBOX_TAG_EXECUTE_QPU = 0x00030011,
		 			// Execute code on QPU
		MAILBOX_TAG_ENABLE_QPU = 0x00030012,
		 			// QPU enable

		/* Displaymax commands */
		MAILBOX_TAG_GET_DISPMANX_HANDLE = 0x00030014,
		 			// Get displaymax handle
		MAILBOX_TAG_GET_EDID_BLOCK = 0x00030020,
		 			// Get HDMI EDID block

		/* SD Card commands */
		MAILBOX_GET_SDHOST_CLOCK = 0x00030042,
		 			// Get SD Card EMCC clock
		MAILBOX_SET_SDHOST_CLOCK = 0x00038042, 
					// Set SD Card EMCC clock

		/* Framebuffer commands */
		MAILBOX_TAG_ALLOCATE_FRAMEBUFFER = 0x00040001,
		 			// Allocate Framebuffer address
		MAILBOX_TAG_BLANK_SCREEN = 0x00040002,
		 			// Blank screen
		MAILBOX_TAG_GET_PHYSICAL_WIDTH_HEIGHT = 0x00040003, 
					// Get physical screen width/height
		MAILBOX_TAG_GET_VIRTUAL_WIDTH_HEIGHT = 0x00040004, 
					// Get virtual screen width/height
		MAILBOX_TAG_GET_COLOUR_DEPTH = 0x00040005,
		 			// Get screen colour depth
		MAILBOX_TAG_GET_PIXEL_ORDER = 0x00040006, 
					// Get screen pixel order
		MAILBOX_TAG_GET_ALPHA_MODE = 0x00040007,
		 			// Get screen alpha mode
		MAILBOX_TAG_GET_PITCH = 0x00040008, 
					// Get screen line to line pitch
		MAILBOX_TAG_GET_VIRTUAL_OFFSET = 0x00040009, 
					// Get screen virtual offset
		MAILBOX_TAG_GET_OVERSCAN = 0x0004000A, 
					// Get screen overscan value
		MAILBOX_TAG_GET_PALETTE = 0x0004000B,
		 			// Get screen palette

		MAILBOX_TAG_RELEASE_FRAMEBUFFER = 0x00048001,
		 			// Release Framebuffer address
		MAILBOX_TAG_SET_PHYSICAL_WIDTH_HEIGHT = 0x00048003, 
					// Set physical screen width/heigh
		MAILBOX_TAG_SET_VIRTUAL_WIDTH_HEIGHT = 0x00048004,
		 			// Set virtual screen width/height
		MAILBOX_TAG_SET_COLOUR_DEPTH = 0x00048005,
		 			// Set screen colour depth
		MAILBOX_TAG_SET_PIXEL_ORDER = 0x00048006,
		 			// Set screen pixel order
		MAILBOX_TAG_SET_ALPHA_MODE = 0x00048007, 
					// Set screen alpha mode
		MAILBOX_TAG_SET_VIRTUAL_OFFSET = 0x00048009,
		 			// Set screen virtual offset
		MAILBOX_TAG_SET_OVERSCAN = 0x0004800A, 
					// Set screen overscan value
		MAILBOX_TAG_SET_PALETTE = 0x0004800B,
		 			// Set screen palette
		MAILBOX_TAG_SET_VSYNC = 0x0004800E, 
					// Set screen VSync
		MAILBOX_TAG_SET_BACKLIGHT = 0x0004800F,
		 			// Set screen backlight

		/* VCHIQ commands */
		MAILBOX_TAG_VCHIQ_INIT = 0x00048010,
		 			// Enable VCHIQ

		/* Config commands */
		MAILBOX_TAG_GET_COMMAND_LINE = 0x00050001, 
					// Get command line 

		/* Shared resource management commands */
		MAILBOX_TAG_GET_DMA_CHANNELS = 0x00060001, 
					// Get DMA channels

		/* Cursor commands */
		MAILBOX_TAG_SET_CURSOR_INFO = 0x00008010, 
					// Set cursor info
		MAILBOX_TAG_SET_CURSOR_STATE = 0x00008011, 
					// Set cursor state
	} TAG_CHANNEL_COMMAND;

	/*--------------------------------------------------------------------------}
	{					    ENUMERATED MAILBOX CLOCK ID							}
	{		  https://github.com/raspberrypi/firmware/wiki/Mailboxes			}
	{--------------------------------------------------------------------------*/
	typedef enum {
		CLK_EMMC_ID = 0x1,
		 								// Mailbox Tag Channel EMMC clock ID 
		CLK_UART_ID = 0x2, 
										// Mailbox Tag Channel uart clock ID
		CLK_ARM_ID = 0x3,
		 								// Mailbox Tag Channel ARM clock ID
		CLK_CORE_ID = 0x4,
		 								// Mailbox Tag Channel SOC core clock ID
		CLK_V3D_ID = 0x5, 
										// Mailbox Tag Channel V3D clock ID
		CLK_H264_ID = 0x6,
		 								// Mailbox Tag Channel H264 clock ID
		CLK_ISP_ID = 0x7,
		 								// Mailbox Tag Channel ISP clock ID
		CLK_SDRAM_ID = 0x8, 
										// Mailbox Tag Channel SDRAM clock ID
		CLK_PIXEL_ID = 0x9,
		 								// Mailbox Tag Channel PIXEL clock ID
		CLK_PWM_ID = 0xA, 
										// Mailbox Tag Channel PWM clock ID
	} MB_CLOCK_ID;


	/*--------------------------------------------------------------------------}
	{			      ENUMERATED MAILBOX POWER BLOCK ID							}
	{		  https://github.com/raspberrypi/firmware/wiki/Mailboxes			}
	{--------------------------------------------------------------------------*/
	typedef enum {
		PB_SDCARD = 0x0, 
										// Mailbox Tag Channel SD Card power block 
		PB_UART0 = 0x1, 
										// Mailbox Tag Channel UART0 power block 
		PB_UART1 = 0x2,
		 								// Mailbox Tag Channel UART1 power block 
		PB_USBHCD = 0x3, 
										// Mailbox Tag Channel USB_HCD power block 
		PB_I2C0 = 0x4, 
										// Mailbox Tag Channel I2C0 power block 
		PB_I2C1 = 0x5, 
										// Mailbox Tag Channel I2C1 power block 
		PB_I2C2 = 0x6, 
										// Mailbox Tag Channel I2C2 power block 
		PB_SPI = 0x7, 
										// Mailbox Tag Channel SPI power block 
		PB_CCP2TX = 0x8,
		 								// Mailbox Tag Channel CCP2TX power block 
	} MB_POWER_ID;

	/*--------------------------------------------------------------------------}
	;{	  ENUMERATED CODE TARGET ... WHICH ARM CPU THE CODE IS COMPILED FOR		}
	;{-------------------------------------------------------------------------*/
	typedef enum {
		ARM5_CODE = 5, 
												// ARM 5 CPU is targetted
		ARM6_CODE = 6,
		 										// ARM 6 CPU is targetted
		ARM7_CODE = 7,
		 										// ARM 7 CPU is targetted
		ARM8_CODE = 8,
		 										// ARM 8 CPU is targetted
	} ARM_CODE_TYPE;

	/*--------------------------------------------------------------------------}
	;{	 ENUMERATED AARCH TARGET ... WHICH AARCH TARGET CODE IS COMPILED FOR	}
	;{-------------------------------------------------------------------------*/
	typedef enum {
		AARCH32 = 0, 
												// AARCH32 - 32 bit
		AARCH64 = 1,
		 										// AARCH64 - 64 bit
	} AARCH_MODE;



	/***************************************************************************}
	{		 		    PUBLIC STRUCTURE DEFINITIONS				            }
	****************************************************************************/

	/*--------------------------------------------------------------------------}
	{				 COMPILER TARGET SETTING STRUCTURE DEFINED					}
	{--------------------------------------------------------------------------*/
	typedef union 
	{
		struct 
		{
			ARM_CODE_TYPE ArmCodeTarget : 4; 							// @0  Compiler code target
			AARCH_MODE AArchMode : 1; 									// @5  Code AARCH type compiler is producing
			unsigned CoresSupported : 3; 								// @6  Cores the code is setup to support
			unsigned reserved : 23; 										// @9-31 reserved
			unsigned HardFloats : 1; 									// @31	 Compiler code for hard floats -- NOT USED RasROS
		};
		uint32_t Raw32; 													// Union to access all 32 bits as a uint32_t
	} CODE_TYPE;

	/*--------------------------------------------------------------------------}
	{						ARM CPU ID STRUCTURE DEFINED						}
	{--------------------------------------------------------------------------*/
	typedef union 
	{
		struct 
		{
			unsigned Revision : 4; 										// @0-3  CPU minor revision 
			unsigned PartNumber : 12; 									// @4-15  Partnumber
			unsigned Architecture : 4; 									// @16-19 Architecture
			unsigned Variant : 4; 										// @20-23 Variant
			unsigned Implementer : 8; 									// @24-31 reserved
		};
		uint32_t Raw32; 													// Union to access all 32 bits as a uint32_t
	} CPU_ID;

	/*--------------------------------------------------------------------------}
	{				SMARTSTART VERSION STRUCTURE DEFINED						}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct
		{
			unsigned LoVersion : 16; 									// @0-15  SmartStart minor version 
			unsigned HiVersion : 8; 										// @16-23 SmartStart major version
			unsigned _reserved : 8; 										// @24-31 reserved
		};
		uint32_t Raw32; 													// Union to access all 32 bits as a uint32_t
	} SMARTSTART_VER;

	/***************************************************************************}
	{                      PUBLIC INTERFACE MEMORY VARIABLES                    }
	{***************************************************************************/
	
	//Set in SmartStart32.S on bootup
	
	extern uint32_t RPi_IO_Base_Addr; 				// RPI IO base address auto-detected by SmartStartxx.S
	extern uint32_t RPi_ARM_TO_GPU_Alias; 			// RPI ARM_TO_GPU_Alias auto-detected by SmartStartxx.S
	extern uint32_t RPi_BootAddr; 					// RPI address processor booted from auto-detected by SmartStartxx.S
	extern uint32_t RPi_CoresReady; 					// RPI cpu cores made read for use by SmartStartxx.S
	extern uint32_t RPi_CPUBootMode; 				// RPI cpu mode it was in when it booted
	extern CPU_ID RPi_CpuId; 						// RPI CPU type auto-detected by SmartStartxx.S
	extern CODE_TYPE RPi_CompileMode; 				// RPI code type that compiler produced
	extern uint32_t RPi_CPUCurrentMode; 				// RPI cpu current operation mode
	extern SMARTSTART_VER RPi_SmartStartVer; 		// SmartStart version
	extern uint32_t RPi_HeapStart;					//technically BSS end, see linker.ld. Needs to be rounded up to 4 byte alignment before being used!
	extern uint32_t RPi_UsingFPU;                    //1 if hardware, 0 if software
	
	/***************************************************************************}
	{                       PUBLIC C INTERFACE ROUTINES                         }
	{***************************************************************************/

	bool InitScreen(uint32_t *pWidth, uint32_t *pHeight, uint32_t *pDepth, uint32_t *pPitch, uint32_t *pFrameBuffer);

	/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
	{		  INTERRUPT HELPER ROUTINES PROVIDE BY RPi-SmartStart API	        }
	{++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	/*-[setFiqFuncAddress]------------------------------------------------------}
	. NOTE: Public C interface only to code located in SmartsStartxx.S
	. Sets the function pointer to be the called when an Fiq interrupt occurs.
	. CPU fiq interrupts will be disabled so they can't trigger while changing.
	. RETURN: Old function pointer that was in use (will return 0 if never set).
	.--------------------------------------------------------------------------*/
	uintptr_t setFiqFuncAddress(void(*ARMaddress)(void));

	/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
	{			GLOBAL INTERRUPT CONTROL PROVIDE BY RPi-SmartStart API		    }
	{++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	/*-[EnableInterrupts]-------------------------------------------------------}
	. NOTE: Public C interface only to code located in SmartsStartxx.S
	. Enable global interrupts on any CPU core calling this function.
	.--------------------------------------------------------------------------*/
	//Call EnableIRQs() instead RasROS void EnableInterrupts(void);

	/*-[DisableInterrupts]------------------------------------------------------}
	. NOTE: Public C interface only to code located in SmartsStartxx.S
	. Disable global interrupts on any CPU core calling this function.
	.--------------------------------------------------------------------------*/
	//Call DisableIRQs() instead void DisableInterrupts(void);

	/*-[EnableFIQ]--------------------------------------------------------------}
	. NOTE: Public C interface only to code located in SmartsStartxx.S
	. Enable global fast interrupts on any CPU core calling this function.
	.--------------------------------------------------------------------------*/
	void EnableFIQ(void);

	/*-[DisableFIQ]-------------------------------------------------------------}
	. NOTE: Public C interface only to code located in SmartsStartxx.S
	. Disable global fast interrupts on any CPU core calling this function.
	.--------------------------------------------------------------------------*/
	void DisableFIQ(void);

	/*-[getDAIF]----------------------------------------------------------------}
	. NOTE: Public C interface only to code located in SmartsStartxx.S
	. Return the DAIF flags for any CPU core calling this function.
	.--------------------------------------------------------------------------*/
	unsigned long getDAIF(void);

	/*-[setDAIF]----------------------------------------------------------------}
	. NOTE: Public C interface only to code located in SmartsStartxx.S
	. Sets the DAIF flags for any CPU core calling this function.
	.--------------------------------------------------------------------------*/
	void setDAIF(unsigned long flags);

	/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
	{			 RPi-SmartStart API TO MULTICORE FUNCTIONS					    }
	{++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	/*-[getCoreID]------------------------------------------------------------}
	. NOTE: Public C interface only to code located in SmartsStartxx.S
	. Returns the multicore id of the core calling the function
	.--------------------------------------------------------------------------*/
	unsigned int getCoreID(void);

	/*-[CoreExecute]------------------------------------------------------------}
	. NOTE: Public C interface only to code located in SmartsStartxx.S
	. Commands the given parked core to execute the function provided. The core
	. called must be parked in the secondary spinloop. All secondary cores are
	. automatically parked by the normal SmartStart boot so are ready to deploy
	.--------------------------------------------------------------------------*/
	bool CoreExecute(uint8_t coreNum, void(*func)(void));


	/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
	{		VC4 GPU ADDRESS HELPER ROUTINES PROVIDE BY RPi-SmartStart API	    }
	{++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	/*-[ARMaddrToGPUaddr]-------------------------------------------------------}
	. NOTE: Public C interface only to code located in SmartsStartxx.S
	. Converts an ARM address to GPU address by using the GPU_alias offset
	.--------------------------------------------------------------------------*/
	uint32_t ARMaddrToGPUaddr(void* ARMaddress);

	/*-[GPUaddrToARMaddr]-------------------------------------------------------}
	. NOTE: Public C interface only to code located in SmartsStartxx.S
	. Converts a GPU address to an ARM address by using the GPU_alias offset
	.--------------------------------------------------------------------------*/
	uint32_t GPUaddrToARMaddr(uint32_t GPUaddress);

	/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
	{	  RPi-SmartStart Compatability for David Welch CALLS he always uses	    }
	{++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	/*-[ PUT32 ]----------------------------------------------------------------}
	. NOTE: Public C interface only to code located in SmartsStartxx.S
	. Put the 32bit value out to the given address (Match Davids calls)
	.--------------------------------------------------------------------------*/
	void PUT32(uint32_t addr, uint32_t value);

	/*-[ GET32 ]----------------------------------------------------------------}
	. NOTE: Public C interface only to code located in SmartsStartxx.S
	. Get the 32bit value from the given address (Match Davids calls)
	.--------------------------------------------------------------------------*/
	uint32_t GET32(uint32_t addr);

	/*==========================================================================}
	{			 PUBLIC CPU ID ROUTINES PROVIDED BY RPi-SmartStart API			}
	{==========================================================================*/

	/*-[ RPi_CpuIdString]-------------------------------------------------------}
	. Returns the CPU id string of the CPU auto-detected by the SmartStart code
	.--------------------------------------------------------------------------*/
	const char* RPi_CpuIdString(void);

	/*==========================================================================}
	{			 PUBLIC GPIO ROUTINES PROVIDED BY RPi-SmartStart API			}
	{==========================================================================*/

	/*-[gpio_setup]-------------------------------------------------------------}
	. Given a valid GPIO port number and mode sets GPIO to given mode
	. RETURN: true for success, false for any failure
	.--------------------------------------------------------------------------*/
	bool gpio_setup(unsigned int gpio, GPIOMODE mode);

	/*-[gpio_output]------------------------------------------------------------}
	. Given a valid GPIO port number the output is set high(true) or Low (false)
	. RETURN: true for success, false for any failure
	.--------------------------------------------------------------------------*/
	bool gpio_output(unsigned int gpio, bool on);

	/*-[gpio_input]-------------------------------------------------------------}
	. Reads the actual level of the GPIO port number
	. RETURN: true = GPIO input high, false = GPIO input low
	.--------------------------------------------------------------------------*/
	bool gpio_input(unsigned int gpio);

	/*-[gpio_checkEvent]--------------------------------------------------------}
	. Checks the given GPIO port number for an event/irq flag.
	. RETURN: true for event occured, false for no event
	.--------------------------------------------------------------------------*/
	bool gpio_checkEvent(unsigned int gpio);

	/*-[gpio_clearEvent]--------------------------------------------------------}
	. Clears the given GPIO port number event/irq flag.
	. RETURN: true for success, false for any failure
	.--------------------------------------------------------------------------*/
	bool gpio_clearEvent(unsigned int gpio);

	/*-[gpio_edgeDetect]--------------------------------------------------------}
	. Sets GPIO port number edge detection to lifting/falling in Async/Sync mode
	. RETURN: true for success, false for any failure
	.--------------------------------------------------------------------------*/
	bool gpio_edgeDetect(unsigned int gpio, bool lifting, bool Async);

	/*-[gpio_fixResistor]-------------------------------------------------------}
	. Set the GPIO port number with fix resistors to pull up/pull down.
	. RETURN: true for success, false for any failure
	.--------------------------------------------------------------------------*/
	bool gpio_fixResistor(unsigned int gpio, GPIO_FIX_RESISTOR resistor);

	/*==========================================================================}
	{		   PUBLIC TIMER ROUTINES PROVIDED BY RPi-SmartStart API				}
	{==========================================================================*/

	/*-[timer_getTickCount64]---------------------------------------------------}
	. Get 1Mhz ARM system timer tick count in full 64 bit.
	. The timer read is as per the Broadcom specification of two 32bit reads
	. RETURN: tickcount value as an unsigned 64bit value in microseconds (usec)
	.--------------------------------------------------------------------------*/
	uint64_t timer_getTickCount64(void);

	/*-[timer_Wait]-------------------------------------------------------------}
	. This will simply wait the requested number of microseconds before return.
	.--------------------------------------------------------------------------*/
	void timer_wait(uint64_t usec);

	/*-[tick_Difference]--------------------------------------------------------}
	. Given two timer tick results it returns the time difference between them.
	. If (us1 > us2) it is assumed the timer rolled as we expect (us2 > us1)
	.--------------------------------------------------------------------------*/
	uint64_t tick_difference(uint64_t us1, uint64_t us2);

	/*==========================================================================}
	{		  PUBLIC PI MAILBOX ROUTINES PROVIDED BY RPi-SmartStart API			}
	{==========================================================================*/

	/*-[mailbox_write]----------------------------------------------------------}
	. This will execute the sending of the given data block message thru the
	. mailbox system on the given channel. It is normal for a response back so
	. usually you need to follow the write up with a read.
	. RETURN: True for success, False for failure.
	.--------------------------------------------------------------------------*/
	bool mailbox_write(MAILBOX_CHANNEL channel, uint32_t message);

	/*-[mailbox_read]-----------------------------------------------------------}
	. This will read any pending data on the mailbox system on the given channel.
	. RETURN: The read value for success, 0xFEEDDEAD for failure.
	.--------------------------------------------------------------------------*/
	uint32_t mailbox_read(MAILBOX_CHANNEL channel);

	/*-[mailbox_tag_message]----------------------------------------------------}
	. This will post and execute the given variadic data onto the tags channel
	. on the mailbox system. You must provide the correct number of response
	. uint32_t variables and a pointer to the response buffer. You nominate the
	. number of data uint32_t for the call and fill the variadic data in. If you
	. do not want the response data back the use NULL for response_buffer.
	. RETURN: True for success and the response data will be set with data
	.         False for failure and the response buffer is untouched.
	.--------------------------------------------------------------------------*/
	bool mailbox_tag_message(uint32_t* response_buf,
							// Pointer to response buffer (NULL = no response wanted)
							  uint8_t data_count,
								// Number of uint32_t data to be set for call
							  ...); 										// Variadic uint32_t values for call

	/*==========================================================================}
	{	  PUBLIC PI TIMER INTERRUPT ROUTINES PROVIDED BY RPi-SmartStart API		}
	{==========================================================================*/

	/*-[ClearTimerIrq]----------------------------------------------------------}
	. Simply clear the timer interupt by hitting the clear register. Any timer
	. fiq/irq interrupt should call this before exiting.
	.--------------------------------------------------------------------------*/
	void ClearTimerIrq(void);

	/*-[TimerIrqSetup]----------------------------------------------------------}
	. The timer irq interrupt rate is set to the period in usec between triggers.
	. Largest period is around 16 million usec (16 sec) it varies on core speed
	. RETURN: TRUE if successful,  FALSE for any failure
	.--------------------------------------------------------------------------*/
	bool TimerIrqSetup(uint32_t period_in_us); 							// Period between timer interrupts in usec

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
							 void(*ARMaddress)(void)); 					// Function to call (0 = ignored)

	/*-[ClearLocalTimerIrq]-----------------------------------------------------}
	. Simply clear the local timer interupt by hitting the clear registers. Any
	. local timer irq/fiq interrupt should call this before exiting.
	.--------------------------------------------------------------------------*/
	void ClearLocalTimerIrq(void);

	/*-[LocalTimerIrqSetup]-----------------------------------------------------}
	. The local timer irq interrupt rate is set to the period in usec between 
	. triggers. On BCM2835 (ARM6) it does not have core timer so call fails. 
	. Largest period is around 16 million usec (16 sec) it varies on core speed
	. RETURN: TRUE if successful, FALSE for any failure
	.--------------------------------------------------------------------------*/
	bool LocalTimerSetup(uint32_t period_in_us,
								// Period between timer interrupts in usec
						  uint8_t coreNum); 								// Core number

	/*==========================================================================}
	{				           MINIUART ROUTINES								}
	{==========================================================================*/

	/*-[miniuart_init]----------------------------------------------------------}
	. Initializes the miniuart (NS16550 compatible one) to the given baudrate
	. with 8 data bits, no parity and 1 stop bit. On the PI models 1A, 1B, 1B+
	. and PI ZeroW this is external GPIO14 & GPIO15 (Pin header 8 & 10).
	. RETURN: true if successfuly set, false on any error
	.--------------------------------------------------------------------------*/
	bool miniuart_init(unsigned int baudrate);

	/*-[miniuart_getc]----------------------------------------------------------}
	. Wait for an retrieve a character from the uart.
	.--------------------------------------------------------------------------*/
	char miniuart_getc(void);

	/*-[miniuart_putc]----------------------------------------------------------}
	. Send a character out via uart.
	.--------------------------------------------------------------------------*/
	void miniuart_putc(char c);

	/*-[miniuart_puts]----------------------------------------------------------}
	. Send a '\0' terminated character string out via uart.
	.--------------------------------------------------------------------------*/
	void miniuart_puts(char *str);

	/*==========================================================================}
	{				           PL011 UART ROUTINES								}
	{==========================================================================*/

	/*-[pl011_uart_init]--------------------------------------------------------}
	. Initializes the pl011 uart to the given baudrate with 8 data, no parity
	. and 1 stop bit. On the PI models 2B, Pi3, Pi3B+, Pi Zero and CM this is
	. external GPIO14 & GPIO15 (Pin header 8 & 10).
	. RETURN: true if successfuly set, false on any error
	.--------------------------------------------------------------------------*/
	bool pl011_uart_init(unsigned int baudrate);

	/*-[pl011_uart_getc]--------------------------------------------------------}
	. Wait for an retrieve a character from the uart.
	.--------------------------------------------------------------------------*/
	char pl011_uart_getc(void);

	/*-[pl011_uart_putc]--------------------------------------------------------}
	. Send a character out via uart.
	.--------------------------------------------------------------------------*/
	void pl011_uart_putc(char c);

	/*-[pl011_uart_puts]--------------------------------------------------------}
	. Send a '\0' terminated character string out via uart.
	.--------------------------------------------------------------------------*/
	void pl011_uart_puts(char *str);

	/*==========================================================================}
	{				         UART CONSOLE ROUTINES								}
	{==========================================================================*/

	/*-[console_uart_init]------------------------------------------------------}
	. This will detect the Pi model at runtime and set the baud rate and console 
	. uart functions to pins 6 & 8 on header for the detected model. It could be 
	. either the PL011 or Miniuart port depending on Pi Model but all that is 
	. shielded from you. So you can just use these calls and know on whichever 
	. model the code is placed it will come out at header pins.
	. RETURN: true if successfuly set, false on any error
	.--------------------------------------------------------------------------*/
	bool console_uart_init(unsigned int baudrate);

	/*-[console_uart_getc]------------------------------------------------------}
	. Wait for an retrieve a character from the runtime detected uart that goes
	. to Pins 6 & 8 on the 40 way header. It could be minuart or PL011 depending 
	. on Pi model detected at runtime. If console_uart_init has not been called
	. the function will simply immediately return with a character 0.
	.--------------------------------------------------------------------------*/
	char console_uart_getc(void);

	/*-[console_uart_putc]------------------------------------------------------}
	. Writes a character to the runtime detected uart that goes to Pins 6 & 8 on
	. the 40 way header. It could be minuart or PL011 depending on Pi model that
	. was detected at runtime. If console_uart_init has not been called then the
	. function will simply immediately return.
	.--------------------------------------------------------------------------*/
	void console_uart_putc(char ch);

	/*-[console_uart_puts]------------------------------------------------------}
	. Writes a a '\0' terminated character string to the runtime detected uart
	. that goes to Pins 6 & 8 on the 40 way header. It could be minuart or PL011
	. depending on Pi model that was detected at runtime. If console_uart_init
	. has not been called then the function will simply immediately return.
	.--------------------------------------------------------------------------*/
	void console_uart_puts(char *str);

	/*==========================================================================}
	{	   PUBLIC PI ACTIVITY LED ROUTINE PROVIDED BY RPi-SmartStart API		}
	{==========================================================================*/

	/*-[set_Activity_LED]-------------------------------------------------------}
	. This will set the PI activity LED on or off as requested. The SmartStart
	. stub provides the Pi board autodetection so the right GPIO port is used.
	. RETURN: True the LED state was successfully change, false otherwise
	.--------------------------------------------------------------------------*/
	bool set_Activity_LED(bool on);

	/*==========================================================================}
	{	   PUBLIC ARM CPU SPEED SET ROUTINES PROVIDED BY RPi-SmartStart API	 	}
	{==========================================================================*/

	/*-[ARM_setmaxspeed]--------------------------------------------------------}
	. This will set the ARM cpu to the maximum. You can optionally print confirm
	. message to screen or uart etc by providing a print function handler.
	. The print handler can be screen or a uart/usb/eth handler for monitoring.
	. RETURN: True maxium speed was successfully set, false otherwise
	.--------------------------------------------------------------------------*/
	bool ARM_setmaxspeed(int(*prn_handler)(const char *fmt, ...));

	/*==========================================================================}
	{				      SMARTSTART DISPLAY ROUTINES							}
	{==========================================================================*/

	/*-[displaySmartStart]------------------------------------------------------}
	. Will print 2 lines of basic smart start details to given print handler
	. The print handler can be screen or a uart/usb/eth handler for monitoring.
	.--------------------------------------------------------------------------*/
	void displaySmartStart(int(*prn_handler)(const char *fmt, ...));


/***************************************************************************}
{       RASPBERRY PI REGISTER STRUCTURE DEFINITIONS                         }
****************************************************************************/

/*--------------------------------------------------------------------------}
{    RASPBERRY PI GPIO HARDWARE REGISTERS - BCM2835.PDF Manual Section 6	}
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(4))) GPIORegisters {
		uint32_t GPFSEL[6];  												// 0x00  GPFSEL0 - GPFSEL5
		uint32_t reserved1;  												// 0x18  reserved
		uint32_t GPSET[2];  												// 0x1C  GPSET0 - GPSET1;
		uint32_t reserved2;  												// 0x24  reserved
		uint32_t GPCLR[2];  												// 0x28  GPCLR0 - GPCLR1
		uint32_t reserved3;  												// 0x30  reserved
		const uint32_t GPLEV[2];  										// 0x34  GPLEV0 - GPLEV1   ** Read only hence const
		uint32_t reserved4;  												// 0x3C  reserved
		uint32_t GPEDS[2];  												// 0x40  GPEDS0 - GPEDS1 
		uint32_t reserved5;  												// 0x48  reserved
		uint32_t GPREN[2];  												// 0x4C  GPREN0 - GPREN1;	 
		uint32_t reserved6;  												// 0x54  reserved
		uint32_t GPFEN[2];  												// 0x58  GPFEN0 - GPFEN1;
		uint32_t reserved7;  												// 0x60  reserved
		uint32_t GPHEN[2];  												// 0x64  GPHEN0 - GPHEN1;
		uint32_t reserved8;  												// 0x6c  reserved
		uint32_t GPLEN[2];  												// 0x70  GPLEN0 - GPLEN1;
		uint32_t reserved9;  												// 0x78  reserved
		uint32_t GPAREN[2];  												// 0x7C  GPAREN0 - GPAREN1;
		uint32_t reserved10;  											// 0x84  reserved
		uint32_t GPAFEN[2];   											// 0x88  GPAFEN0 - GPAFEN1;
		uint32_t reserved11;  											// 0x90  reserved
		uint32_t GPPUD;   												// 0x94  GPPUD 
		uint32_t GPPUDCLK[2];   											// 0x98  GPPUDCLK0 - GPPUDCLK1;
	}
	;

	/*--------------------------------------------------------------------------}
	{  RASPBERRY PI SYSTEM TIMER HARDWARE REGISTERS - BCM2835 Manual Section 12	}
	{--------------------------------------------------------------------------*/
	struct __attribute__((__packed__, aligned(4))) SystemTimerRegisters {
		uint32_t ControlStatus;  											// 0x00
		uint32_t TimerLo;  												// 0x04
		uint32_t TimerHi;  												// 0x08
		uint32_t Compare0;  												// 0x0C
		uint32_t Compare1;  												// 0x10
		uint32_t Compare2;  												// 0x14
		uint32_t Compare3;  												// 0x18
	};

	/*--------------------------------------------------------------------------}
	{	   TIMER_CONTROL REGISTER BCM2835 ARM Peripheral manual page 197		}
	{--------------------------------------------------------------------------*/
	typedef union  
	{
		struct 
		{
			unsigned unused : 1;  										// @0 Unused bit
			unsigned Counter32Bit : 1;  									// @1 Counter32 bit (16bit if false)
			TIMER_PRESCALE Prescale : 2;  								// @2-3 Prescale  
			unsigned unused1 : 1;  										// @4 Unused bit
			unsigned TimerIrqEnable : 1;  								// @5 Timer irq enable
			unsigned unused2 : 1;  										// @6 Unused bit
			unsigned TimerEnable : 1;  									// @7 Timer enable
			unsigned reserved : 24;  										// @8-31 reserved
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} time_ctrl_reg_t;


	/*--------------------------------------------------------------------------}
	{   RASPBERRY PI ARM TIMER HARDWARE REGISTERS - BCM2835 Manual Section 14	}
	{--------------------------------------------------------------------------*/
	struct __attribute__((__packed__, aligned(4))) ArmTimerRegisters {
		uint32_t Load;  													// 0x00
		const uint32_t Value;  											// 0x04  ** Read only hence const
		time_ctrl_reg_t Control;  										// 0x08
		uint32_t Clear;  													// 0x0C
		const uint32_t RawIRQ;  											// 0x10  ** Read only hence const
		const uint32_t MaskedIRQ;  										// 0x14  ** Read only hence const
		uint32_t Reload;  												// 0x18
	};

	/*--------------------------------------------------------------------------}
	{   IRQ BASIC PENDING REGISTER - BCM2835.PDF Manual Section 7 page 113/114  }
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__, aligned(4)))
		{
			const unsigned Timer_IRQ_pending : 1;  						// @0 Timer Irq pending  ** Read only
			const unsigned Mailbox_IRQ_pending : 1;  						// @1 Mailbox Irq pending  ** Read only
			const unsigned Doorbell0_IRQ_pending : 1;  					// @2 Arm Doorbell0 Irq pending  ** Read only
			const unsigned Doorbell1_IRQ_pending : 1;  					// @3 Arm Doorbell0 Irq pending  ** Read only
			const unsigned GPU0_halted_IRQ_pending : 1;  					// @4 GPU0 halted IRQ pending  ** Read only
			const unsigned GPU1_halted_IRQ_pending : 1;  					// @5 GPU1 halted IRQ pending  ** Read only
			const unsigned Illegal_access_type1_pending : 1;  			// @6 Illegal access type 1 IRQ pending  ** Read only
			const unsigned Illegal_access_type0_pending : 1;  			// @7 Illegal access type 0 IRQ pending  ** Read only
			const unsigned Bits_set_in_pending_register_1 : 1;  			// @8 One or more bits set in pending register 1  ** Read only
			const unsigned Bits_set_in_pending_register_2 : 1;  			// @9 One or more bits set in pending register 2  ** Read only
			const unsigned GPU_IRQ_7_pending : 1;  						// @10 GPU irq 7 pending  ** Read only
			const unsigned GPU_IRQ_9_pending : 1;  						// @11 GPU irq 9 pending  ** Read only
			const unsigned GPU_IRQ_10_pending : 1;  						// @12 GPU irq 10 pending  ** Read only
			const unsigned GPU_IRQ_18_pending : 1;  						// @13 GPU irq 18 pending  ** Read only
			const unsigned GPU_IRQ_19_pending : 1;  						// @14 GPU irq 19 pending  ** Read only
			const unsigned GPU_IRQ_53_pending : 1;  						// @15 GPU irq 53 pending  ** Read only
			const unsigned GPU_IRQ_54_pending : 1;  						// @16 GPU irq 54 pending  ** Read only
			const unsigned GPU_IRQ_55_pending : 1;  						// @17 GPU irq 55 pending  ** Read only
			const unsigned GPU_IRQ_56_pending : 1;  						// @18 GPU irq 56 pending  ** Read only
			const unsigned GPU_IRQ_57_pending : 1;  						// @19 GPU irq 57 pending  ** Read only
			const unsigned GPU_IRQ_62_pending : 1;  						// @20 GPU irq 62 pending  ** Read only
			unsigned reserved : 10;  										// @21-31 reserved
		};
		const uint32_t Raw32;  											// Union to access all 32 bits as a uint32_t  ** Read only
	} irq_basic_pending_reg_t;

	/*--------------------------------------------------------------------------}
	{	   FIQ CONTROL REGISTER BCM2835.PDF ARM Peripheral manual page 116		}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__, aligned(4)))
		{
			unsigned SelectFIQSource : 7;  								// @0-6 Select FIQ source
			unsigned EnableFIQ : 1;  										// @7 enable FIQ
			unsigned reserved : 24;  										// @8-31 reserved
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} fiq_control_reg_t;

	/*--------------------------------------------------------------------------}
	{	 ENABLE BASIC IRQ REGISTER BCM2835 ARM Peripheral manual page 117		}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__, aligned(4)))
		{
			unsigned Enable_Timer_IRQ : 1;  								// @0 Timer Irq enable
			unsigned Enable_Mailbox_IRQ : 1;  							// @1 Mailbox Irq enable
			unsigned Enable_Doorbell0_IRQ : 1;  							// @2 Arm Doorbell0 Irq enable
			unsigned Enable_Doorbell1_IRQ : 1;  							// @3 Arm Doorbell0 Irq enable
			unsigned Enable_GPU0_halted_IRQ : 1;  						// @4 GPU0 halted IRQ enable
			unsigned Enable_GPU1_halted_IRQ : 1;  						// @5 GPU1 halted IRQ enable
			unsigned Enable_Illegal_access_type1 : 1;  					// @6 Illegal access type 1 IRQ enable
			unsigned Enable_Illegal_access_type0 : 1;  					// @7 Illegal access type 0 IRQ enable
			unsigned reserved : 24;  										// @8-31 reserved
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} irq_enable_basic_reg_t;

	/*--------------------------------------------------------------------------}
	{	DISABLE BASIC IRQ REGISTER BCM2835 ARM Peripheral manual page 117		}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__, aligned(4)))
		{
			unsigned Disable_Timer_IRQ : 1;  								// @0 Timer Irq disable
			unsigned Disable_Mailbox_IRQ : 1;  							// @1 Mailbox Irq disable
			unsigned Disable_Doorbell0_IRQ : 1;  							// @2 Arm Doorbell0 Irq disable
			unsigned Disable_Doorbell1_IRQ : 1;  							// @3 Arm Doorbell0 Irq disable
			unsigned Disable_GPU0_halted_IRQ : 1;  						// @4 GPU0 halted IRQ disable
			unsigned Disable_GPU1_halted_IRQ : 1;  						// @5 GPU1 halted IRQ disable
			unsigned Disable_Illegal_access_type1 : 1;  					// @6 Illegal access type 1 IRQ disable
			unsigned Disable_Illegal_access_type0 : 1;  					// @7 Illegal access type 0 IRQ disable
			unsigned reserved : 24;  										// @8-31 reserved
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} irq_disable_basic_reg_t;

	/*--------------------------------------------------------------------------}
	{	   RASPBERRY PI IRQ HARDWARE REGISTERS - BCM2835 Manual Section 7	    }
	{--------------------------------------------------------------------------*/


	
	//AND masks for IRQ 32..63 registers other than Basic IRQ (which are defined above as irq_basic_pending_reg_t)
	//such as IRQPending2 below
	#define I2C_SPI_SLV_INT 1<<11  
	#define PWA_0           1<<13
	#define PWA_1           1<<14
	#define SMI             1<<16
	#define GPIO_INT0		1<<17
	#define GPIO_INT1       1<<18
	#define GPIO_INT2       1<<19
	#define GPIO_INT3       1<<20
	#define I2C_INT			1<<21
	#define SPI_INT			1<<22
	#define PCM_INT			1<<23
	#define UART_INT        1<<25
	
	struct __attribute__((__packed__, aligned(4))) IrqControlRegisters {
		const irq_basic_pending_reg_t IRQBasicPending;  					// 0x200   ** Read only hence const
		uint32_t IRQPending1;  											// 0x204
		uint32_t IRQPending2;  											// 0x208
		fiq_control_reg_t FIQControl;  									// 0x20C
		uint32_t EnableIRQs1;  											// 0x210
		uint32_t EnableIRQs2;  											// 0x214
		irq_enable_basic_reg_t EnableBasicIRQs;  							// 0x218
		uint32_t DisableIRQs1;  											// 0x21C
		uint32_t DisableIRQs2;  											// 0x220
		irq_disable_basic_reg_t DisableBasicIRQs;  						// 0x224
	};

	/*--------------------------------------------------------------------------}
	;{               RASPBERRY PI MAILBOX HARRDWARE REGISTERS					}
	;{-------------------------------------------------------------------------*/
	struct __attribute__((__packed__, aligned(4))) MailBoxRegisters {
		const uint32_t Read0;  											// 0x00         Read data from VC to ARM
		uint32_t Unused[3];  												// 0x04-0x0F
		uint32_t Peek0;  													// 0x10
		uint32_t Sender0;  												// 0x14
		uint32_t Status0;  												// 0x18         Status of VC to ARM
		uint32_t Config0;  												// 0x1C        
		uint32_t Write1;  												// 0x20         Write data from ARM to VC
		uint32_t Unused2[3];  											// 0x24-0x2F
		uint32_t Peek1;  													// 0x30
		uint32_t Sender1;  												// 0x34
		uint32_t Status1;  												// 0x38         Status of ARM to VC
		uint32_t Config1;  												// 0x3C 
	};

	/*--------------------------------------------------------------------------}
	{         MINI UART IO Register BCM2835 ARM Peripheral manual page 11		}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__))
		{
			unsigned DATA : 8;  											// @0-7 Transmit Read/write data if DLAB=0, DLAB = 1 Lower 8 bits of 16 bit baud rate generator 
			unsigned reserved : 24;  										// @8-31 Reserved - Write as 0, read as don't care 
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} mu_io_reg_t;

	/*--------------------------------------------------------------------------}
	{ MINI UART INTERRUPT ENABLE Register BCM2835 ARM Peripheral manual page 12	}
	{   PAGE HAS ERRORS: https://elinux.org/BCM2835_datasheet_errata            }
	{   It is essentially same as standard 16550 register IER                   }
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__))
		{
			unsigned RXDI : 1;  											// @0	 If this bit is set the interrupt line is asserted if the receive FIFO holds at least 1 byte.
			unsigned TXEI : 1;  											// @1	 If this bit is set the interrupt line is asserted if the transmit FIFO is empty.  
			unsigned LSI : 1;  											// @2	 If this bit is set the Receiver Line Status interrupt is asserted on overrun error, parity error, framing error etc
			unsigned MSI : 1;  											// @3	 If this bit is set the Modem Status interrupt is asserted on a change To Send(CTS), Data Set Ready(DSR)
			unsigned reserved : 28;  										// @4-31 Reserved - Write as 0, read as don't care 
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} mu_ie_reg_t;

	/*--------------------------------------------------------------------------}
	{   MINI UART INTERRUPT ID Register BCM2835 ARM Peripheral manual page 13	}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		const struct __attribute__((__packed__))
		{
			// THE REGISTER READ DEFINITIONS 
	unsigned PENDING : 1;  										// @0	 This bit is clear whenever an interrupt is pending 
//	enum {  rasROS won't compile
//				MU_NO_INTERRUPTS = 0,  									//		 No interrupts pending
//				MU_TXE_INTERRUPT = 1,  									//		 Transmit buffer empty causing interrupt
//				MU_RXD_INTERRUPTS = 2,  									//		 receive fifa has data causing interrupt 
//			} SOURCE : 2;  												// @1-2	 READ this register shows the interrupt ID bits 
#define MU_NO_INTERRUPTS 0
#define MU_TXE_INTERRUPT 1
#define MU_RXD_INTERRUPTS 2
			unsigned SOURCE : 2;
			
			unsigned reserved_rd : 29;  									// @3-31 Reserved - Write as 0, read as don't care 
		}
		;
		struct __attribute__((__packed__))
		{
			// THE REGISTER WRITE DEFINITIONS 
	unsigned unused : 1;  										// @0	 This bit has no use when writing 
	unsigned RXFIFO_CLEAR : 1;  									// @1	 Clear the receive fifo by writing a 1
	unsigned TXFIFO_CLEAR : 1;  									// @2	 Clear the transmit fifo by writing a 1
	unsigned reserved_wr : 29;  									// @3-31 Reserved - Write as 0, read as don't care 
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} mu_ii_reg_t;

	/*--------------------------------------------------------------------------}
	{	MINI UART LINE CONTROL Register BCM2835 ARM Peripheral manual page 14	}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__))
		{
			unsigned DATA_LENGTH : 1;  									// @0	 If clear the UART works in 7-bit mode, If set the UART works in 8-bit mode 
			unsigned reserved : 5;  										// @1-5	 Reserved, write zero, read as dont care Some of these bits have functions in a 16550 compatible UART but are ignored here
			unsigned BREAK : 1;  											// @6	 If set high the UART1_TX line is pulled low continuously
			unsigned DLAB : 1;  											// @7	 DLAB access control bit.
			unsigned reserved1 : 24;  									// @8-31 Reserved - Write as 0, read as don't care 
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} mu_lcr_reg_t;

	/*--------------------------------------------------------------------------}
	{	MINI UART MODEM CONTROL Register BCM2835 ARM Peripheral manual page 14	}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__))
		{
			unsigned reserved : 1;  										// @0	 Reserved, write zero, read as dont care 
			unsigned RTS : 1;  											// @1	 If clear the UART1_RTS line is high, If set the UART1_RTS line is low 
			unsigned reserved1 : 30;  									// @2-31 Reserved - Write as 0, read as don't care 
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} mu_mcr_reg_t;

	/*--------------------------------------------------------------------------}
	{	MINI UART LINE STATUS Register BCM2835 ARM Peripheral manual page 15	}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__))
		{
			unsigned RXFDA : 1;  											// @0	 This bit is set if the receive FIFO holds at least 1 
			unsigned RXOE : 1;  											// @1	 This bit is set if there was a receiver overrun
			unsigned reserved : 3;  										// @2-4	 Reserved, write zero, read as dont care 
			unsigned TXFE : 1;  											// @5	 This bit is set if the transmit FIFO can accept at least one byte
			unsigned TXIdle : 1;  										// @6	 This bit is set if the transmit FIFO is empty and the transmitter is idle. (Finished shifting out the last bit). 
			unsigned reserved1 : 25;  									// @7-31 Reserved - Write as 0, read as don't care 
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} mu_lsr_reg_t;

	/*--------------------------------------------------------------------------}
	{	MINI UART MODEM STATUS Register BCM2835 ARM Peripheral manual page 15	}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__))
		{
			unsigned reserved : 4;  										// @0-3	 Reserved, write zero, read as dont care 
			unsigned CTS : 1;  											// @4	 This bit is the inverse of the CTS input, If set the UART1_CTS pin is low If clear the UART1_CTS pin is high 
			unsigned reserved1 : 27;  									// @5-31 Reserved - Write as 0, read as don't care 
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} mu_msr_reg_t;

	/*--------------------------------------------------------------------------}
	{	  MINI UART SCRATCH Register BCM2835 ARM Peripheral manual page 16	    }
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__))
		{
			unsigned USER_DATA : 8;  										// @0-7	 One whole byte extra on top of the 134217728 provided by the SDC  
			unsigned reserved : 24;  										// @8-31 Reserved - Write as 0, read as don't care 
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} mu_scratch_reg_t;

	/*--------------------------------------------------------------------------}
	{     MINI UART CONTROL Register BCM2835 ARM Peripheral manual page 16	    }
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__))
		{
			unsigned RXE : 1;  											// @0	 If this bit is set the mini UART receiver is enabled, If this bit is clear the mini UART receiver is disabled  
			unsigned TXE : 1;  											// @1	 If this bit is set the mini UART transmitter is enabled, If this bit is clear the mini UART transmitter is disabled
			unsigned EnableRTS : 1;  										// @2	 If this bit is set the RTS line will de-assert if the rc FIFO reaches it 'auto flow' level. If this bit is clear RTS is controlled by the AUX_MU_MCR_REG register bit 1. 
			unsigned EnableCTS : 1;  										// @3	 If this bit is set the transmitter will stop if the CTS line is de-asserted. If this bit is clear the transmitter will ignore the status of the CTS line
/*RasROS	enum {
				FIFOhas3spaces = 0,
				FIFOhas2spaces = 1,
				FIFOhas1spaces = 2,
				FIFOhas4spaces = 3,
			} RTSflowLevel : 2;  											// @4-5	 These two bits specify at what receiver FIFO level the RTS line is de-asserted in auto-flow mode
*/
#define	FIFOhas3spaces 0
#define	FIFOhas2spaces 1
#define	FIFOhas1spaces 2
#define	FIFOhas4spaces 3
			unsigned RTSflowLevel : 2;
			
			unsigned RTSassertLevel : 1;  								// @6	 If set the RTS auto flow assert level is low, If clear the RTS auto flow assert level is high
			unsigned CTSassertLevel : 1;  								// @7	 If set the CTS auto flow assert level is low, If clear the CTS auto flow assert level is high
			unsigned reserved : 24;  										// @8-31 Reserved - Write as 0, read as don't care 
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} mu_cntl_reg_t;

	/*--------------------------------------------------------------------------}
	{	  MINI UART STATUS Register BCM2835 ARM Peripheral manual page 18	    }
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__))
		{
			unsigned RXFDA : 1;  											// @0	 This bit is set if the receive FIFO holds at least 1
			unsigned TXFE : 1;  											// @1	 If this bit is set the mini UART transmitter FIFO can accept at least one more symbol
			unsigned RXIdle : 1;  										// @2	 If this bit is set the receiver is idle. If this bit is clear the receiver is busy
			unsigned TXIdle : 1;  										// @3	 If this bit is set the transmitter is idle. If this bit is clear the transmitter is idle
			unsigned RXOE : 1;  											// @4	 This bit is set if there was a receiver overrun
			unsigned TXFF : 1;  											// @5	 The inverse of bit 0
			unsigned RTS : 1;  											// @6	 This bit shows the status of the RTS line
			unsigned CTS : 1;  											// @7	 This bit shows the status of the CTS line
			unsigned TXFCE : 1;  											// @8	 If this bit is set the transmitter FIFO is empty. Thus it can accept 8 symbols
			unsigned TX_DONE : 1;  										// @9	 This bit is set if the transmitter is idle and the transmit FIFO is empty. It is a logic AND of bits 2 and 8 
			unsigned reserved : 6;  										// @10-15 Reserved - Write as 0, read as don't care 
			unsigned RXFIFOLEVEL : 4;  									// @16-19 These bits shows how many symbols are stored in the receive FIFO The value is in the range 0-8 
			unsigned reserved1 : 4;  										// @20-23 Reserved - Write as 0, read as don't care 
			unsigned TXFIFOLEVEL : 4;  									// @24-27 These bits shows how many symbols are stored in the transmit FIFO The value is in the range 0-8
			unsigned reserved2 : 4;  										// @28-31 Reserved - Write as 0, read as don't care 
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} mu_stat_reg_t;

	/*--------------------------------------------------------------------------}
	{	  MINI UART BAUDRATE Register BCM2835 ARM Peripheral manual page 19  	}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__))
		{
			unsigned DIVISOR : 16;  										// @0-15	 Baudrate divisor  
			unsigned reserved : 16;  										// @16-31	 Reserved - Write as 0, read as don't care 
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} mu_baudrate_reg_t;

	/*--------------------------------------------------------------------------}
	{	  MINIUART STRUCTURE LAYOUT BCM2835 ARM Peripheral manual page 8	    }
	{--------------------------------------------------------------------------*/
	struct __attribute__((__packed__, aligned(4))) MiniUARTRegisters {
		mu_io_reg_t IO;  													// +0x0
		mu_ie_reg_t IER;  												// +0x4
		mu_ii_reg_t IIR;  												// +0x8
		mu_lcr_reg_t LCR;  												// +0xC
		mu_mcr_reg_t MCR;  												// +0x10
		const mu_lsr_reg_t LSR;  											// +0x14	** READ ONLY HENCE CONST **
		const mu_msr_reg_t MSR;  											// +0x18	** READ ONLY HENCE CONST **
		mu_scratch_reg_t SCRATCH;  										// +0x1C
		mu_cntl_reg_t CNTL;  												// +0x20
		const mu_stat_reg_t STAT;  										// +0x24	** READ ONLY HENCE CONST **
		mu_baudrate_reg_t BAUD;  											// +0x28;
	};

	/*--------------------------------------------------------------------------}
	{     PL011 UART DATA Register BCM2835 ARM Peripheral manual page 179/180	}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__))
		{
			unsigned DATA : 8;  											// @0-7 Transmit Read/write data
			unsigned FE : 1;  											// @8	Framing error
			unsigned PE : 1;  											// @9	Parity error
			unsigned BE : 1;  											// @10	Break error
			unsigned OE : 1;  											// @11	Overrun error
			unsigned _reserved : 20;  									// @12-31 Reserved - Write as 0, read as don't care 
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} pl011_data_reg_t;

	/*--------------------------------------------------------------------------}
	{     PL011 UART FR Register BCM2835 ARM Peripheral manual page 181/182	    }
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__))
		{
			unsigned CTS : 1;  											// @0	  Clear to send. This bit is the complement of the UART clear to send, nUARTCTS
			unsigned DSR : 1;  											// @1	  Unsupported, write zero, read as don't care 
			unsigned DCD : 1;  											// @2	  Unsupported, write zero, read as don't care  
			unsigned BUSY : 1;  											// @3	  UART busy. If this bit is set to 1, the UART is busy transmitting data.
			unsigned RXFE : 1;  											// @4	  Receive FIFO empty. The meaning of this bit depends on the state of the FEN bit
			unsigned TXFF : 1;  											// @5	  Transmit FIFO full. The meaning of this bit depends on the state of the FEN bit
			unsigned RXFF : 1;  											// @6	  Receive FIFO full. The meaning of this bit depends on the state of the FEN bit 
			unsigned TXFE : 1;  											// @7	  Transmit FIFO empty. The meaning of this bit depends on the state of the FEN bit 
			unsigned _reserved : 24;  									// @8-31  Reserved - Write as 0, read as don't care 
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} pl011_fr_reg_t;

	/*--------------------------------------------------------------------------}
	{      PL011 UART IBRD Register BCM2835 ARM Peripheral manual page 183	    }
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__))
		{
			unsigned DIVISOR : 16;  										// @0-15 Integer baud rate divisor
			unsigned _reserved : 16;  									// @12-31 Reserved - Write as 0, read as don't care 
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} pl011_ibrd_reg_t;

	/*--------------------------------------------------------------------------}
	{      PL011 UART FBRD Register BCM2835 ARM Peripheral manual page 184	    }
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__))
		{
			unsigned DIVISOR : 6;  										// @0-5	  Factional baud rate divisor
			unsigned _reserved : 26;  									// @6-31  Reserved - Write as 0, read as don't care 
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} pl011_fbrd_reg_t;

	/*--------------------------------------------------------------------------}
	{    PL011 UART LRCH Register BCM2835 ARM Peripheral manual page 184/185	}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__))
		{
			unsigned BRK : 1;  											// @0	  Send break. If this bit is set to 1, a low-level is continually output on the TXD output
			unsigned PEN : 1;  											// @1	  Parity enable, 0 = parity is disabled, 1 = parity via bit 2. 
			unsigned EPS : 1;  											// @2	  If PEN = 1 then 0 = odd parity, 1 = even parity
			unsigned STP2 : 1;  											// @3	  Two stops = 1, 1 stop bit = 0
			unsigned FEN : 1;  											// @4	  FIFO's enable = 1, No FIFO's uart becomes 1 character deep = 0
/*RasROS			enum {
				PL011_DATA_5BITS = 0,
				PL011_DATA_6BITS = 1,
				PL011_DATA_7BITS = 2,
				PL011_DATA_8BITS = 3,
			} DATALEN : 2;  												// @5-6	  Data length for transmission
*/
#define PL011_DATA_5BITS 0
#define PL011_DATA_6BITS 1
#define PL011_DATA_7BITS 2
#define PL011_DATA_8BITS 3
			unsigned DATALEN : 2;
			
			unsigned SPS : 1;  											// @7	  Stick parity select 1 = enabled, 0 = stick parity is disabled 
			unsigned _reserved : 24;  									// @8-31  Reserved - Write as 0, read as don't care 
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} pl011_lrch_reg_t;

	/*--------------------------------------------------------------------------}
	{    PL011 UART CR Register BCM2835 ARM Peripheral manual page 185/186/187	}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__))
		{
			unsigned UARTEN : 1;  										// @0	  UART enable = 1, disabled = 0
			unsigned _unused : 6;  										// @2-6	  unused bits
			unsigned LBE : 1;  											// @7	  Loop back enable = 1
			unsigned TXE : 1;  											// @8	  TX enabled = 1, disabled = 0
			unsigned RXE : 1;  											// @9	  RX enabled = 1, disabled = 0
			unsigned DTR_unused : 1;  									// @10	  DTR unused
			unsigned RTS : 1;  											// @11	  RTS complement when the bit is programmed to a 1 then nUARTRTS is LOW.
			unsigned OUT : 2;  											// @12-13 Unsupported
			unsigned RTSEN : 1;  											// @14	  RTS hardware flow control enable if this bit is set to 1. 
			unsigned CTSEN : 1;  											// @15	  CTS hardware flow control enable if this bit is set to 1.
			unsigned _reserved : 16;  									// @16-31 Reserved - Write as 0, read as don't care 
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} pl011_cr_reg_t;

	/*--------------------------------------------------------------------------}
	{       PL011 UART ICR Register BCM2835 ARM Peripheral manual page 192	    }
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct __attribute__((__packed__))
		{
			unsigned RIMIC : 1;  											// @0	  Unsupported, write zero, read as don't care 
			unsigned CTSMIC : 1;  										// @1	  nUARTCTS modem masked interrupt status 
			unsigned DCDMIC : 1;  										// @2	  Unsupported, write zero, read as don't care 
			unsigned DSRMIC : 1;  										// @3	  Unsupported, write zero, read as don't care 
			unsigned RXIC : 1;  											// @4	  Receive interrupt clear. 
			unsigned TXIC : 1;  											// @5	  Transmit interrupt clear
			unsigned RTIC : 1;  											// @6	  Receive timeout interrupt clear. 
			unsigned FEIC : 1;  											// @7	  Framing error interrupt clear.
			unsigned PEIC : 1;  											// @8	  Parity error interrupt clear.
			unsigned BEIC : 1;  											// @9	  Break error interrupt clear.
			unsigned OEIC : 1;  											// @10	  Overrun error interrupt clear.
			unsigned _reserved : 21;  									// @11-31 Reserved - Write as 0, read as don't care 
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} pl011_icr_reg_t;

	/*--------------------------------------------------------------------------}
	{	 PL011 UART STRUCTURE LAYOUT BCM2835 ARM Peripheral manual page 175	    }
	{--------------------------------------------------------------------------*/
	struct __attribute__((__packed__, aligned(4))) PL011UARTRegisters {
		pl011_data_reg_t DR;  											// +0x0
		uint32_t RSRECR;  												// +0x4
		uint32_t _unused[4];  											// +0x8, +0xC, +0x10, +0x14
		pl011_fr_reg_t FR;  												// +0x18
		uint32_t _unused1[2];  											// +0x1C, 0x20
		pl011_ibrd_reg_t IBRD;  											// +0x24
		pl011_fbrd_reg_t FBRD;  											// +0x28
		pl011_lrch_reg_t LCRH;  											// +0x2C
		pl011_cr_reg_t CR;  												// +0x30
		uint32_t IFLS;  													// +0x34
		uint32_t IMSC;  													// +0x38
		uint32_t RIS;  													// +0x3C
		uint32_t MIS;  													// +0x40
		pl011_icr_reg_t ICR;  											// +0x44
		uint32_t DMACR;  													// +0x48
	};

	/*--------------------------------------------------------------------------}
	{    LOCAL TIMER INTERRUPT ROUTING REGISTER - QA7_rev3.4.pdf page 18		}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct
		{
/*RasROS			enum {
				LOCALTIMER_TO_CORE0_IRQ = 0,
				LOCALTIMER_TO_CORE1_IRQ = 1,
				LOCALTIMER_TO_CORE2_IRQ = 2,
				LOCALTIMER_TO_CORE3_IRQ = 3,
				LOCALTIMER_TO_CORE0_FIQ = 4,
				LOCALTIMER_TO_CORE1_FIQ = 5,
				LOCALTIMER_TO_CORE2_FIQ = 6,
				LOCALTIMER_TO_CORE3_FIQ = 7,
			} Routing : 3;												// @0-2	  Local Timer routing 
*/
#define	LOCALTIMER_TO_CORE0_IRQ  0
#define LOCALTIMER_TO_CORE1_IRQ  1
#define LOCALTIMER_TO_CORE2_IRQ  2
#define LOCALTIMER_TO_CORE3_IRQ  3
#define LOCALTIMER_TO_CORE0_FIQ  4
#define LOCALTIMER_TO_CORE1_FIQ  5
#define LOCALTIMER_TO_CORE2_FIQ  6
#define LOCALTIMER_TO_CORE3_FIQ  7
	
			unsigned Routing : 3;
			
			
			unsigned unused : 29;  										// @3-31
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} local_timer_int_route_reg_t;

	/*--------------------------------------------------------------------------}
	{    LOCAL TIMER CONTROL AND STATUS REGISTER - QA7_rev3.4.pdf page 17		}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct
		{
			unsigned ReloadValue : 28;  									// @0-27  Reload value 
			unsigned TimerEnable : 1;  									// @28	  Timer enable (1 = enabled) 
			unsigned IntEnable : 1;  										// @29	  Interrupt enable (1= enabled)
			unsigned unused : 1;  										// @30	  Unused
			unsigned IntPending : 1;  									// @31	  Timer Interrupt flag (Read-Only) 
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} local_timer_ctrl_status_reg_t;

	/*--------------------------------------------------------------------------}
	{     LOCAL TIMER CLEAR AND RELOAD REGISTER - QA7_rev3.4.pdf page 18		}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct
		{
			unsigned unused : 30;  										// @0-29  unused 
			unsigned Reload : 1;  										// @30	  Local timer-reloaded when written as 1 
			unsigned IntClear : 1;  										// @31	  Interrupt flag clear when written as 1  
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} local_timer_clr_reload_reg_t;

	/*--------------------------------------------------------------------------}
	{    GENERIC TIMER INTERRUPT CONTROL REGISTER - QA7_rev3.4.pdf page 13		}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct
		{
			unsigned nCNTPSIRQ_IRQ : 1;  									// @0	Secure physical timer event IRQ enabled, This bit is only valid if bit 4 is clear otherwise it is ignored. 
			unsigned nCNTPNSIRQ_IRQ : 1;  								// @1	Non-secure physical timer event IRQ enabled, This bit is only valid if bit 5 is clear otherwise it is ignored
			unsigned nCNTHPIRQ_IRQ : 1;  									// @2	Hypervisor physical timer event IRQ enabled, This bit is only valid if bit 6 is clear otherwise it is ignored
			unsigned nCNTVIRQ_IRQ : 1;  									// @3	Virtual physical timer event IRQ enabled, This bit is only valid if bit 7 is clear otherwise it is ignored
			unsigned nCNTPSIRQ_FIQ : 1;  									// @4	Secure physical timer event FIQ enabled, If set, this bit overrides the IRQ bit (0) 
			unsigned nCNTPNSIRQ_FIQ : 1;  								// @5	Non-secure physical timer event FIQ enabled, If set, this bit overrides the IRQ bit (1)
			unsigned nCNTHPIRQ_FIQ : 1;  									// @6	Hypervisor physical timer event FIQ enabled, If set, this bit overrides the IRQ bit (2)
			unsigned nCNTVIRQ_FIQ : 1;  									// @7	Virtual physical timer event FIQ enabled, If set, this bit overrides the IRQ bit (3)
			unsigned reserved : 24;  										// @8-31 reserved
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} generic_timer_int_ctrl_reg_t;


	/*--------------------------------------------------------------------------}
	{       MAILBOX INTERRUPT CONTROL REGISTER - QA7_rev3.4.pdf page 14			}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct
		{
			unsigned Mailbox0_IRQ : 1;  									// @0	Set IRQ enabled, This bit is only valid if bit 4 is clear otherwise it is ignored. 
			unsigned Mailbox1_IRQ : 1;  									// @1	Set IRQ enabled, This bit is only valid if bit 5 is clear otherwise it is ignored
			unsigned Mailbox2_IRQ : 1;  									// @2	Set IRQ enabled, This bit is only valid if bit 6 is clear otherwise it is ignored
			unsigned Mailbox3_IRQ : 1;  									// @3	Set IRQ enabled, This bit is only valid if bit 7 is clear otherwise it is ignored
			unsigned Mailbox0_FIQ : 1;  									// @4	Set FIQ enabled, If set, this bit overrides the IRQ bit (0) 
			unsigned Mailbox1_FIQ : 1;  									// @5	Set FIQ enabled, If set, this bit overrides the IRQ bit (1)
			unsigned Mailbox2_FIQ : 1;  									// @6	Set FIQ enabled, If set, this bit overrides the IRQ bit (2)
			unsigned Mailbox3_FIQ : 1;  									// @7	Set FIQ enabled, If set, this bit overrides the IRQ bit (3)
			unsigned reserved : 24;  										// @8-31 reserved
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} mailbox_int_ctrl_reg_t;

	/*--------------------------------------------------------------------------}
	{		 CORE INTERRUPT SOURCE REGISTER - QA7_rev3.4.pdf page 16			}
	{--------------------------------------------------------------------------*/
	typedef union
	{
		struct
		{
			unsigned CNTPSIRQ : 1;  										// @0	  CNTPSIRQ  interrupt 
			unsigned CNTPNSIRQ : 1;  										// @1	  CNTPNSIRQ  interrupt 
			unsigned CNTHPIRQ : 1;  										// @2	  CNTHPIRQ  interrupt 
			unsigned CNTVIRQ : 1;  										// @3	  CNTVIRQ  interrupt 
			unsigned Mailbox0_Int : 1;  									// @4	  Set FIQ enabled, If set, this bit overrides the IRQ bit (0) 
			unsigned Mailbox1_Int : 1;  									// @5	  Set FIQ enabled, If set, this bit overrides the IRQ bit (1)
			unsigned Mailbox2_Int : 1;  									// @6	  Set FIQ enabled, If set, this bit overrides the IRQ bit (2)
			unsigned Mailbox3_Int : 1;  									// @7	  Set FIQ enabled, If set, this bit overrides the IRQ bit (3)
			unsigned GPU_Int : 1;  										// @8	  GPU interrupt <Can be high in one core only> 
			unsigned PMU_Int : 1;  										// @9	  PMU interrupt 
			unsigned AXI_Int : 1;  										// @10	  AXI-outstanding interrupt <For core 0 only!> all others are 0 
			unsigned Timer_Int : 1;  										// @11	  Local timer interrupt
			unsigned GPIO_Int : 16;  										// @12-27 Peripheral 1..15 interrupt (Currently not used
			unsigned reserved : 4;  										// @28-31 reserved
		};
		uint32_t Raw32;  													// Union to access all 32 bits as a uint32_t
	} core_int_source_reg_t;

	/*--------------------------------------------------------------------------}
	{					 ALL QA7 REGISTERS IN ONE BIG STRUCTURE					}
	{--------------------------------------------------------------------------*/
	struct __attribute__((__packed__, aligned(4))) QA7Registers {
		local_timer_int_route_reg_t TimerRouting;  						// 0x24
		uint32_t GPIORouting;  											// 0x28
		uint32_t AXIOutstandingCounters;  								// 0x2C
		uint32_t AXIOutstandingIrq;  										// 0x30
		local_timer_ctrl_status_reg_t TimerControlStatus;  				// 0x34
		local_timer_clr_reload_reg_t TimerClearReload;  					// 0x38
		uint32_t unused;  												// 0x3C
		generic_timer_int_ctrl_reg_t CoreTimerIntControl[4];  			// 0x40, 0x44, 0x48, 0x4C  .. One per core
		mailbox_int_ctrl_reg_t  CoreMailboxIntControl[4];  				// 0x50, 0x54, 0x58, 0x5C  .. One per core
		core_int_source_reg_t CoreIRQSource[4];  							// 0x60, 0x64, 0x68, 0x6C  .. One per core
		core_int_source_reg_t CoreFIQSource[4];  							// 0x70, 0x74, 0x78, 0x7C  .. One per core
	};

	/***************************************************************************}
	{        PRIVATE INTERNAL RASPBERRY PI REGISTER STRUCTURE CHECKS            }
	****************************************************************************/

	/*--------------------------------------------------------------------------}
	{					 CODE TYPE STRUCTURE COMPILE TIME CHECKS	            }
	{--------------------------------------------------------------------------*/
	/* If you have never seen compile time assertions it's worth google search */
	/* on "Compile Time Assertions". It is part of the C11++ specification and */
	/* all compilers that support the standard will have them (GCC, MSC inc)   */
	/*-------------------------------------------------------------------------*/
	//#include <assert.h>								// Need for compile time static_assert
	/*
	// Check the code type structure size 
	static_assert(sizeof(CODE_TYPE) == 0x04, "Structure CODE_TYPE should be 0x04 bytes in size");
	static_assert(sizeof(CPU_ID) == 0x04, "Structure CPU_ID should be 0x04 bytes in size");
	static_assert(sizeof(struct GPIORegisters) == 0xA0, "Structure GPIORegisters should be 0xA0 bytes in size");
	static_assert(sizeof(struct SystemTimerRegisters) == 0x1C, "Structure SystemTimerRegisters should be 0x1C bytes in size");
	static_assert(sizeof(struct ArmTimerRegisters) == 0x1C, "Structure ArmTimerRegisters should be 0x1C bytes in size");
	static_assert(sizeof(struct IrqControlRegisters) == 0x28, "Structure IrqControlRegisters should be 0x28 bytes in size");
	static_assert(sizeof(struct MailBoxRegisters) == 0x40, "Structure MailBoxRegisters should be 0x40 bytes in size");
	static_assert(sizeof(struct MiniUARTRegisters) == 0x2C, "MiniUARTRegisters should be 0x2C bytes in size");
	static_assert(sizeof(struct PL011UARTRegisters) == 0x4C, "PL011UARTRegisters should be 0x4C bytes in size");
	static_assert(sizeof(struct QA7Registers) == 0x5C, "QA7Registers should be 0x5C bytes in size");
	*/

	/***************************************************************************}
	{     PRIVATE POINTERS TO ALL OUR RASPBERRY PI REGISTER BANK STRUCTURES	    }
	****************************************************************************/
#define GPIO ((volatile __attribute__((aligned(4))) struct GPIORegisters*)(uintptr_t)(RPi_IO_Base_Addr + 0x200000))
#define SYSTEMTIMER ((volatile __attribute__((aligned(4))) struct SystemTimerRegisters*)(uintptr_t)(RPi_IO_Base_Addr + 0x3000))
#define IRQ ((volatile __attribute__((aligned(4))) struct IrqControlRegisters*)(uintptr_t)(RPi_IO_Base_Addr + 0xB200))
#define ARMTIMER ((volatile __attribute__((aligned(4))) struct ArmTimerRegisters*)(uintptr_t)(RPi_IO_Base_Addr + 0xB400))
#define MAILBOX ((volatile __attribute__((aligned(4))) struct MailBoxRegisters*)(uintptr_t)(RPi_IO_Base_Addr + 0xB880))
#define MINIUART ((volatile struct MiniUARTRegisters*)(uintptr_t)(RPi_IO_Base_Addr + 0x00215040))
#define PL011UART ((volatile struct PL011UARTRegisters*)(uintptr_t)(RPi_IO_Base_Addr + 0x00201000))
#define QA7 ((volatile __attribute__((aligned(4))) struct QA7Registers*)(uintptr_t)(0x40000024))



	
#ifdef __cplusplus								// If we are including to a C++ file
}												// Close the extern C directive wrapper
#endif

#endif