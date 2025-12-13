#include <stdbool.h>		// C standard unit needed for bool and true/false
#include <stdint.h>			// C standard unit needed for uint8_t, uint32_t, etc
#include "MemMapIO.h"
#include "Emb-stdio.h"
#include "Interrupts.h"
#include "FreeRTOS.h"
#include "task.h"


//Public functions

//Enable processing GPIO interrupts also I2C and SPI could go here
void SetupGPIOInterrupts()
{
	DisableInterrupts();
	IRQ->EnableIRQs2 |= GPIO_INT0 | GPIO_INT1 | GPIO_INT2 | GPIO_INT3;
	EnableInterrupts();
	return;
}

//Both enables processing and starts ARM timer to fire IRQs at TICK_RATE_HZ intervals
void SetupTickInterrupt()
{
	DisableInterrupts();  											// Make sure interrupts are off while we do irq registration
	TimerIrqSetup(1000000 / configTICK_RATE_HZ);
	EnableInterrupts();
	return;
}

//This only happens if freeRTOS exits... which should be never. But
//we could maybe go to some kind of debug mode? 
void StopTickInterrupt()
{
	DisableInterrupts();
	ARMTIMER->Control.TimerEnable = 0 ; 	
	ARMTIMER->Control.TimerIrqEnable = 0 ; 					
	IRQ->EnableBasicIRQs.Enable_Timer_IRQ = 0 ; 				
	EnableInterrupts();
}

//Standard handler for regular tick timer
void ARMTimerHandler(void)
{
	xTaskIncrementTick();
#if configUSE_PREEMPTION == 1
	vTaskSwitchContext();
#endif
	ClearTimerIrq();
	
	/*
	unsigned int coreId = getCoreID();    								// Fetch core id
	// Call interrupt handler, if enabled and function valid
	if(coreICB[coreId].pfnHandler)
		coreICB[coreId].pfnHandler(coreId, coreICB[coreId].pParam);
	// Call interrupt clear handler, if enabled and function valid
	if(coreICB[coreId].pfnClear) 
		coreICB[coreId].pfnClear();
	*/
}

void GPIOHandler(int Num)
{
	
}

//These are entry points to CPU interrupts, called from SmartStart via CPU interrupt vector table
//then they call the appropriate "real" handler 
void irqHandler(void)
{
	bool bHandled = false;
	static bool bUnhandledWarned = false;
	
	//Dispatch according to the source
	if(IRQ->IRQBasicPending.Timer_IRQ_pending)
	{
		ARMTimerHandler();
		bHandled = true;
	}
		
	// this isn't being set? if (IRQ->IRQBasicPending.Bits_set_in_pending_register_2)
	{
		if (IRQ->IRQPending2 & GPIO_INT0)
		{
			GPIOHandler(0);
			bHandled = true;
		}
		if (IRQ->IRQPending2 & GPIO_INT1)
		{
			GPIOHandler(1);
			bHandled = true;
		}
		if (IRQ->IRQPending2 & GPIO_INT2)
		{
			GPIOHandler(2);
			bHandled = true; 
		}
		if (IRQ->IRQPending2 & GPIO_INT3)
		{
			GPIOHandler(3);
			bHandled = true;
		}

		//Other interrupt dispatchers go here e.g. SPI, I2C... see definition of GPIO_INT0 etc for the others
	}
	
	//Unhandled interrupts need to be cleared or we'll keep being called and hang... make sure any IRQ is handled above 
	if(!bHandled)
	{
		if (!bUnhandledWarned)
		{
			//This printf might hang because it's accessed by a semaphore (?) not sure if FreeRTOS handles interrupt-accessed semaphores REVISIT (could also just ignore unhandled...) 
			printf("*** Unhandled interrupt! IRQPending1 = $%x IRQPending2 = $%x\n", IRQ->IRQPending1, IRQ->IRQPending2);
			bUnhandledWarned = true;
		}
	}
	else
	{
		bUnhandledWarned = false;
	}

}

void swiHandler(void)
{
	vTaskSwitchContext();
}

void fiqHandler(void)
{
		
}




	
	


		