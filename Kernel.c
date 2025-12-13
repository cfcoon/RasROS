#include <stdint.h>
#include <string.h>

//For providing C std lib functions
#include <errno.h>
#include <sys/stat.h>
#undef errno
extern int errno;

#include "MemMapIO.h"
#include "Emb-stdio.h"
#include "Interrupts.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "SmartGraph.h"

#define STOP printf("STOPPED");while(1){timer_wait(1000000);};

extern void Reset(); //in SmartStart32.S

#define MS_TO_TICKS( xTimeInMs )    ( ( TickType_t ) ( ( ( TickType_t ) ( xTimeInMs ) * ( TickType_t ) configTICK_RATE_HZ ) / ( TickType_t ) 1000U ) ) 

//Initial screen setting
uint32_t Width = 1280, Height = 720, Depth = 32, Pitch = 0, pFramebuffer = 0;

//Demo variables
static const char Spin[4] = { '|', '/', '-', '\\' };

//This animates a rotating slash to the right of the screen, every 1000 milliseconds
void task1(void *pParam) 
{
	int i = 0;
	int col = (Width / BitFontWth) * 0.55f;
	int row = (Height / BitFontHt) * 0.1f;
	const TickType_t xDelay = MS_TO_TICKS(1000);
	while (1) 
	{
		GotoXY(col, row);
		printf("Task 1:   %c", Spin[i]);
		i++;
		i %= 4;
		vTaskDelay(xDelay);
	}
}

//This animates a rotating slash every 250 ms a little further down from task1 
void task2(void *pParam)
{
	int i = 0;
	int col = (Width / BitFontWth) * 0.55f;
	int row = (Height / BitFontHt) * 0.1f + 4;
	const TickType_t xDelay = MS_TO_TICKS(250);
	while (1) 
	{	
		GotoXY(col , row);
		printf("Task 2:   %c", Spin[i]);
		i++;
		i %= 4;
		vTaskDelay(xDelay);
	}
}

void kernel_main(uint32_t r0, uint32_t r1, uint32_t atags)
{
	InitScreen(&Width, &Height, &Depth, &Pitch, &pFramebuffer);
	PiConsole_Init(Width, Height, Depth, Pitch, pFramebuffer);  
	Init_EmbStdio(WriteText);     									// Initialize embedded stdio with printf going to screen
	displaySmartStart(printf);  									
	printf("\nRasROS V0.1\n\n");  
	printf("Screen set to %i x %i x %i\n", (int)Width, (int)Height, (int)Depth);
	ARM_setmaxspeed(printf); 										// ARM CPU to max speed                                      
	
	xTaskCreate(task1, "Num 1", 2048, NULL, 1, NULL);
	xTaskCreate(task2, "Num 2", 2048, NULL, 1, NULL);
	vTaskStartScheduler();
	
	//Should never get here, but if it does, loop.  
	while (1) {
		vTaskDelay(MS_TO_TICKS(1000000));
	}
}


//C std library OS specific implementations
void _exit()
{
	//No real "exiting" on this OS, so soft reboot instead
	Reset();
}
int _kill(int pid, int sig) {
	errno = EINVAL;
	return -1;
}
int _getpid(void) {
	TaskHandle_t h = xTaskGetCurrentTaskHandle();
	return (int)h;
}

//this is for all C library output including stdout... 
int _write(int file, char *ptr, int len) 
{
	printf(ptr);
	return len;
}

int _close(int file) 
{
	return -1;
}

int _fstat(int file, struct stat *st) {
	st->st_mode = S_IFCHR;
	return 0;
}

//Is output stream a TTY terminal...
int _isatty(int file) {
	return 1;
}

//Set position in a file
int _lseek(int file, int ptr, int dir) {
	return 0;
}

//Read from a file/stdin
int _read(int file, char *ptr, int len) {
	return 0;
}
