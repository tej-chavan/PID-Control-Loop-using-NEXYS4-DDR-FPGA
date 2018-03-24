/**
 *
 * @file freertos_hello_world.c
 *
 * @author Rithvik Mitresh Ballal (rithvik@pdx.edu)
 		   Tejas Tushar Chavan	  (techavan@pdx.edu)
 * @copyright Portland State University, 2018
 *
 * This file implements the PID control system using Queues, Semaphores and tasks. It also has a watchdog timer, which will reset the system,
 * if the force crash switch is on.
 *
 * @note
 * The  hardware configuration for this test is a Microblaze-based system with 128 kb of memory,
 *  an instance of the pmodOLEDrgb AXI slave peripheral, and instance of the pmodENC AXI
 * slave peripheral, an instance of AXI GPIO, an instance of AXI timer and an instance of the AXI UARTLite
 * (used for xil_printf() console output). It also has 128 MB of DDR memory with caches
 *
 * @note
 * The driver code and test application(s) for the pmodOLDrgb are based on code provided by Digilent, Inc.
 ******************************************************************************/
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* BSP includes. */
#include "xtmrctr.h"
#include "xgpio.h"
#include "sleep.h"
#include "xil_printf.h"
#include "xwdttb.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "xparameters.h"
#include "xstatus.h"
#include "pmodOLEDrgb.h"
#include "pmodENC.h"

#include "platform.h"
#include "pwm_tmrctr.h"

// Definitions for peripheral PMODENC
#define PMODENC_DEVICE_ID		       XPAR_PMODENC_0_DEVICE_ID
#define PMODENC_BASEADDR		       XPAR_PMODENC_0_S00_AXI_BASEADDR
#define PMODENC_HIGHADDR		       XPAR_PMODENC_0_S00_AXI_HIGHADDR

// Definitions for peripheral PMODOLEDRGB
#define RGBDSPLY_DEVICE_ID		       XPAR_PMODOLEDRGB_0_DEVICE_ID
#define RGBDSPLY_GPIO_BASEADDR	       XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR
#define RGBDSPLY_GPIO_HIGHADDR	       XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_HIGHADD
#define RGBDSPLY_SPI_BASEADDR	       XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR
#define RGBDSPLY_SPI_HIGHADDR	       XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_HIGHADDR

//Direction GPIO parameter for HB3
#define DIR_GPIO_2_BASEADDR            XPAR_AXI_GPIO_2_BASEADDR
#define DIR_GPIO_2_HIGHADDR            XPAR_AXI_GPIO_2_HIGHADDR
#define DIR_GPIO_2_DEVID               XPAR_AXI_GPIO_2_DEVICE_ID
#define DIR_GPIO_2_CHANNEL_0           1

// 16 LED gpio parameter
#define LED_GPIO_1_BASEADDR            XPAR_AXI_GPIO_1_BASEADDR
#define LED_GPIO_1_HIGHADDR            XPAR_AXI_GPIO_1_HIGHADDR
#define LED_GPIO_1_DEVID               XPAR_AXI_GPIO_1_DEVICE_ID
#define LED_GPIO_1_CHANNEL_0           1
#define IS_GPIO_1_DUAL_CHANNEL		   XPAR_AXI_GPIO_1_IS_DUAL

//AXI Timer(for calculating the seconds)parameters
#define SEC_TIMER_DEVICE_ID		       XPAR_AXI_TIMER_2_DEVICE_ID
#define SEC_TIMER_BASEADDR		       XPAR_AXI_TIMER_2_BASEADDR
#define SEC_TIMER_HIGHADDR		       XPAR_AXI_TIMER_2_HIGHADDR
#define TmrCtrNumber			       0

//Push button and Switch GPIO parameter
#define BUTTON_SWITCH_GPIO_0_BASEADDR  XPAR_AXI_GPIO_0_BASEADDR
#define BUTTON_SWITCH_GPIO_0_HIGHADDR  XPAR_AXI_GPIO_0_HIGHADDR
#define BUTTON_SWITCH_GPIO_0_DEVID     XPAR_AXI_GPIO_0_DEVICE_ID
#define BUTTON_GPIO_0_CHANNEL_1        1
#define SWITCH_GPIO_0_CHANNEL_2        2
#define IS_GPIO_0_DUAL_CHANNEL		   XPAR_AXI_GPIO_0_IS_DUAL

//Definitions of interrupt controller parameters
#define INTC_DEVICE_ID                 XPAR_INTC_0_DEVICE_ID
#define SENSOR_A_INTERUPT_ID           XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_3_IP2INTC_IRPT_INTR
#define SEC_TIMER_INTERUPT_ID          XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMER_2_INTERRUPT_INTR
#define FIT_INTERRUPT_ID		       XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR
#define BUTTON_SWITCH_INTERRUPT_ID     XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR
#define WDT_INTERRUPT_ID               XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMEBASE_WDT_0_WDT_INTERRUPT_INTR

//Definitions of  gpio and its interupt
#define SENSOR_A_DEVICE_ID               XPAR_AXI_GPIO_3_DEVICE_ID
#define GPIO_3_INPUT_0_CHANNEL         1
#define SENSOR_A_INTERUPT_MASK         XGPIO_IR_CH1_MASK
#define BUTTON_INTERUPT_MASK           XGPIO_IR_CH1_MASK
#define SWITCH_INTERUPT_MASK           XGPIO_IR_CH2_MASK

//Definition of watch dog timer device id
#define WDT_DEVICEID                  XPAR_WDTTB_0_DEVICE_ID

#define mainQUEUE_LENGTH					8

/* A block time of 0 simply means, "don't block". */
#define mainDONT_BLOCK						( portTickType ) 0

/*Give a delay of 1 second*/
#define DELAY_1_SECOND		1000UL

//===========================Functions Declarations======================================
int Sec_Timer_initialize(void);
static void Button_Switch_Handler (void *pvUnused);
void master_thread (void *p);
int Button_Switch_Intialization();
int Sec_Timer_initialize(void);
int Sensor_A_Initialize();
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix);
void PMDIO_itoa(int32_t value, char *string, int32_t radix);
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num);
void Second_Timer_Handler(void *p);
int	do_init();
int Gpio_Direction_Interrupt_Initialization(XGpio gpioInstance,bool isDual,int interruptID,int channel_1_ID, int channel_2_ID,XInterruptHandler interrupt_handler);
void Button_Switch_Handler_task(void *p);
void SENSOR_A_Handler(void *p);
void display_task(void *p);
void PID_task(void *p);
void WDT_handler(void *p);
//=======================================================================================

//===========================Instance Declarations=======================================
static XTmrCtr InstancePtr;
static PmodOLEDrgb	pmodOLEDrgb_inst;			// Instance declaration for PmodOLED
static PmodENC 	pmodENC_inst;
static XGpio      xDIRGPIOInstance;					// For DIR pin on HB3
static XGpio      xLEDGPIOInstance;					// LED
static XGpio      xButtonSwitchGPIOInstance;   // push buttons and switch
static XGpio      xSensorGPIOInstance;         // Sensor A value
static XTmrCtr    SecTimerInst;                // Timer Instances
static xSemaphoreHandle binary_sem;            //Declare a Sempahore
static xQueueHandle xDisplayQueue = NULL, /*Que handler for display*/xPIDQueue = NULL, /*Que handler for PID*/  xCounterQueue = NULL; /*Que handler for counter*/           // Handler for the Que we create
static TaskHandle_t xMasterTaskHandler=NULL,xReadHandler=NULL,xDisplayHandler=NULL,xPIDHandler=NULL;  // Instance of the task handler
XWdtTb 		WDTInst;                       //Instance of the watch dog timer
//========================================================================================

/*=========================================================================================
The following variables are shared between non-interrupt processing and
interrupt processing such that they must be global(and declared volatile).
The reason for declaring these variables volatile is that these variables are going to change
frequently in the later functions.
These variables are controlled by the FIT timer interrupt handler , Second timer interrupt handler,Buttton switch
handler and Sensor A handler.
===========================================================================================*/
	 static u8  					RotaryCnt=0;						 // Variable to increment the rotary count
     volatile u32                    counter=0;                     // counts for 1 seconds
     volatile u32                    prev_counter=1;                // holds the previous count of the counter
     volatile u32                    process_value=0;              //RPM of the motor
     u16                             gpio_in;                     // take the input from the gpio pin
     u16                             gpio_switch;                 // checks the status of the switch
   typedef struct
    {
    	volatile int  kp; //propotional constant
    	volatile int ki;  //integral constant
    	volatile int kd;  //differential constant
    	volatile u8 duty_cycle; // new duty cycle
    	volatile int i; //position on the screen
    	volatile u8 DC_Process_Value;  // The Processed DC from the sensor
    	volatile short  diff_error;  //differential error
    	volatile u8    RC_Process_Value; // rotary count corresponding to the sensor value
    	volatile u8	   final_rotary_count; // this is the rotary count got from adding the pid constants
    	volatile short error;  // the error for kp
    	volatile short integrate_error ;  //the error associated with integral part
    	volatile u32 set_RPM; // the set value of PID.


    }design_parameters;

    volatile u32 system_running=1;                                          //System running flag
    volatile u32 force_crash=0;                                  //Force crash flag

//==============================================================================================

static void Button_Switch_Handler (void *pvUnused)
{
	//xil_printf("Inside the Button_switch_handler\n");
	xSemaphoreGiveFromISR(binary_sem,NULL);
    XGpio_InterruptClear( &xButtonSwitchGPIOInstance, 1 );
    XGpio_InterruptClear(&xButtonSwitchGPIOInstance,SWITCH_INTERUPT_MASK);

}



//This the main task which will create the other task require for the application
void master_thread (void *p)
{
	portBASE_TYPE xStatus;
	binary_sem=xSemaphoreCreateBinary();
	//binary_sem1=xSemaphoreCreate
	xDisplayQueue = xQueueCreate(1, sizeof(design_parameters) );
	configASSERT( xDisplayQueue );

	xPIDQueue = xQueueCreate(1, sizeof(design_parameters));
	configASSERT( xPIDQueue);

	xCounterQueue = xQueueCreate(1,sizeof(u32));
	configASSERT( xCounterQueue );

    xStatus=xTaskCreate(Button_Switch_Handler_task,"RX",1024,NULL,tskIDLE_PRIORITY+2,&xReadHandler);  // Its is used to create a task for button and switch handler
    if(xStatus==pdFAIL)
    {
    	xil_printf("The read task was not created. Please try again...\n");
    }
    xStatus = xTaskCreate(display_task,"display task",1024,NULL,tskIDLE_PRIORITY+2,&xDisplayHandler); // Its is used to create display task
    	    if(xStatus == pdFAIL)
    	    {
    	    	xil_printf("The system reset task is not created");
    	    }
   xStatus = xTaskCreate(PID_task,"PID task",1024,NULL,tskIDLE_PRIORITY+2,&xPIDHandler);  // Its is used to create pid task
    if(xStatus == pdFAIL)
    {
    	xil_printf("The PID task is not created");
    }

    //starting watch dog timer
    XWdtTb_Start(&WDTInst);
while(1)
{
	system_running = 1;
}

}



int main(void)
{
	//Initialize the HW
    portBASE_TYPE xStatus;
    init_platform();
    xStatus=do_init();

	//Create Task1
	xil_printf("Inside main function");

	if (XWdtTb_IsWdtExpired(&WDTInst))
	{
		XWdtTb_Stop(&WDTInst);
		xStatus = XWdtTb_Initialize(&WDTInst, WDT_DEVICEID);
		if (xStatus != XST_SUCCESS)
			{
			 xil_printf("Could not initialize the pmodENC");
			 return XST_FAILURE;
			}
	}

    xStatus = xTaskCreate( master_thread,( const char * ) "TX",2048,NULL,tskIDLE_PRIORITY+2,&xMasterTaskHandler);
	if(xStatus == pdFAIL)
	{
		xil_printf("The master tast  was not created. Please try again ...\n");
	}
	microblaze_enable_interrupts();

    //Start the Secheduler
	vTaskStartScheduler();
	cleanup_platform();
	return 0;
}




/*************************************************Initialization**********************************************************************************/
int	 do_init()
{
	portBASE_TYPE xStatus;				// status from Xilinx Lib calls

	//initialize the pmodENC and hardware
	xStatus = pmodENC_initialize(&pmodENC_inst, PMODENC_BASEADDR);
	if (xStatus != XST_SUCCESS)
	{
	 xil_printf("Could not initialize the pmodENC");
	 return XST_FAILURE;
	}

	//Initializing the OLEDrgb
	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst,OLEDrgb_BuildRGB(255,0,0));
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 1);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"DC:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 6, 1);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"RPM:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0,4);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"KP:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0,5);
	PMDIO_putnum(&pmodOLEDrgb_inst,0,10);
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4,4);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"KI:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4,5);
	PMDIO_putnum(&pmodOLEDrgb_inst,0,10);
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 8,4);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"KD:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 8,5);
	PMDIO_putnum(&pmodOLEDrgb_inst,0,10);

	// Initialize the PmodENC and Clear the count.
    pmodENC_init(&pmodENC_inst, 1, true);
    pmodENC_clear_count(&pmodENC_inst);

			//return XST_SUCCESS;

    // Initializing the DIR gpio
    xStatus = XGpio_Initialize(&xDIRGPIOInstance,DIR_GPIO_2_DEVID);
    if(xStatus != XST_SUCCESS)
    {
    	xil_printf("The Direction GPIO was not initialized");
    	return XST_FAILURE;
    }
    XGpio_SetDataDirection(&xDIRGPIOInstance,DIR_GPIO_2_CHANNEL_0 , 0);


    //Initializing the Button and the LED
    xStatus = Button_Switch_Intialization();
    if(xStatus != XST_SUCCESS)
    {
    	xil_printf("Could not initialize the Button and Switch port\n");
    	return XST_FAILURE;
    }

	// Initializing the LED gpio
	xStatus = XGpio_Initialize(&xLEDGPIOInstance,LED_GPIO_1_DEVID);
	if(xStatus != XST_SUCCESS)
	{
		xil_printf("Could not initialize the LED port\n");
		return XST_FAILURE;
	}

	//Initializing the watchdog timer
	xStatus = XWdtTb_Initialize(&WDTInst,WDT_DEVICEID);
	if(xStatus != XST_SUCCESS)
	{
      xil_printf("The watch dog timer is not initialized");
      return XST_FAILURE;
	}

	//Installing and initializing the handler
	xStatus = xPortInstallInterruptHandler(WDT_INTERRUPT_ID,WDT_handler, NULL );
	if(xStatus != pdPASS)
	{
		xil_printf("Failed to initialize the Sec timer interrupt handler");
		return XST_FAILURE;
	}
		vPortEnableInterrupt(WDT_INTERRUPT_ID);

	//Initializing the pwm Timer

	xStatus = PWM_Initialize(&InstancePtr,XPAR_TMRCTR_1_DEVICE_ID,true,100000000);
    if (xStatus != 0)
    {
    	xil_printf("Could not initialize the PWM generator\n");
		return XST_FAILURE;
	 }
    xStatus =  PWM_Start(&InstancePtr);
     if (xStatus != 0)
     {
    	xil_printf("Could not start the PWM\n");
    	return XST_FAILURE;
     }

	//Setting the LED gpio channel 1 as output
	XGpio_SetDataDirection(&xLEDGPIOInstance,LED_GPIO_1_CHANNEL_0 , 0);

	//Initializing the Sensor A interrupt.
    xStatus = Sensor_A_Initialize();
    if(xStatus!=XST_SUCCESS)
    {
      xil_printf("Could not initialize the sensor\n");
      return XST_FAILURE;
    }

    //starting the sec timer
       xStatus=Sec_Timer_initialize();
       if (xStatus != XST_SUCCESS)
       {
    	  xil_printf("Couldn't initialize the sec timer");
    	   return XST_FAILURE;
       }

       //starting watch dog timer
          // XWdtTb_Start(&WDTInst);
        xil_printf("The watch dog timer is started!!");

    return XST_SUCCESS;
}

/*
 * Sec_Timer_initialization  is an AXI Timer which count down for 1 second
 * DO NOT MODIFY
 */
int Sec_Timer_initialize(void)
{

	portBASE_TYPE xStatus;				// status from Xilinx Lib calls
	u32		ctlsts;		// control/status register or mask

	xStatus = XTmrCtr_Initialize(&SecTimerInst,SEC_TIMER_DEVICE_ID);
	if (xStatus != XST_SUCCESS) {
		return XST_FAILURE;
	}
	xStatus = XTmrCtr_SelfTest(&SecTimerInst, TmrCtrNumber);
	if (xStatus != XST_SUCCESS) {
		return XST_FAILURE;
	}
	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_LOAD_MASK |XTC_CSR_DOWN_COUNT_MASK |XTC_CSR_ENABLE_INT_MASK ;
	XTmrCtr_SetControlStatusReg(SEC_TIMER_BASEADDR, TmrCtrNumber,ctlsts);

	//Set the value that is loaded into the timer counter and cause it to be loaded into the timer counter
	XTmrCtr_SetLoadReg(SEC_TIMER_BASEADDR, TmrCtrNumber,99999998);
	XTmrCtr_LoadTimerCounterReg(SEC_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts = XTmrCtr_GetControlStatusReg(SEC_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(SEC_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(SEC_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts |= XTC_CSR_ENABLE_TMR_MASK;
	XTmrCtr_SetControlStatusReg(SEC_TIMER_BASEADDR, TmrCtrNumber, ctlsts);
	XTmrCtr_Enable(SEC_TIMER_BASEADDR, TmrCtrNumber);

	xStatus = xPortInstallInterruptHandler(SEC_TIMER_INTERUPT_ID,Second_Timer_Handler, NULL );
	if(xStatus != pdPASS)
	{
		xil_printf("Failed to initialize the Sec timer interrupt handler");
		return XST_FAILURE;
	}
	vPortEnableInterrupt(SEC_TIMER_INTERUPT_ID);

	return XST_SUCCESS;

}

/* Push button and switch interupt
 */
int Button_Switch_Intialization()
{
	portBASE_TYPE xStatus;

	 //Initialinzing the push button and switch
		xStatus = XGpio_Initialize(&xButtonSwitchGPIOInstance,BUTTON_SWITCH_GPIO_0_DEVID);
		if(xStatus != XST_SUCCESS)
		{
		  xil_printf("Could not initialize the Button and Switch GPIO\n");
		  return XST_FAILURE;
		}

		xStatus=Gpio_Direction_Interrupt_Initialization(xButtonSwitchGPIOInstance,true,BUTTON_SWITCH_INTERRUPT_ID,BUTTON_INTERUPT_MASK,SWITCH_INTERUPT_MASK,Button_Switch_Handler);

		if(xStatus != pdPASS)
		{
			xil_printf("Could not initialize the button interrupt\n");
			return XST_FAILURE;
		}
        return XST_SUCCESS;
}

/*
 * Sensor A  gpio and enable the interrupt initialization
 */
int Sensor_A_Initialize()
{
	portBASE_TYPE xStatus;				// status from Xilinx Lib calls

	// initialize the GPIO instances
	xStatus = XGpio_Initialize(&xSensorGPIOInstance, SENSOR_A_DEVICE_ID);
	    if (xStatus != XST_SUCCESS)
	    {
	      xil_printf("Could not initialize the sensor GPIO\n");
		  return XST_FAILURE;
	    }

	   xStatus=Gpio_Direction_Interrupt_Initialization(xSensorGPIOInstance,false,SENSOR_A_INTERUPT_ID,SENSOR_A_INTERUPT_MASK,0,SENSOR_A_Handler);

	   if(xStatus != pdPASS)
	   {
	 	   xil_printf("Could not initialize the sensor interrupt");
	 	   return XST_FAILURE;
	   }



	return XST_SUCCESS;
}

/*********************** DISPLAY-RELATED FUNCTIONS ***********************************/

/****************************************************************************/
/**
 * Converts an integer to ASCII characters
 *
 * algorithm borrowed from ReactOS system libraries
 *
 * Converts an integer to ASCII in the specified base.  Assumes string[] is
 * long enough to hold the result plus the terminating null
 *
 * @param 	value is the integer to convert
 * @param 	*string is a pointer to a buffer large enough to hold the converted number plus
 *  			the terminating null
 * @param	radix is the base to use in conversion,
 *
 * @return  *NONE*
 *
 * @note
 * No size check is done on the return string size.  Make sure you leave room
 * for the full string plus the terminating null in string
 *****************************************************************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix)
{
	char tmp[33];
	char *tp = tmp;
	int32_t i;
	uint32_t v;
	int32_t  sign;
	char *sp;

	if (radix > 36 || radix <= 1)
	{
		return;
	}

	sign = ((10 == radix) && (value < 0));
	if (sign)
	{
		v = -value;
	}
	else
	{
		v = (uint32_t) value;
	}

	while (v || tp == tmp)
	{
		i = v % radix;
		v = v / radix;
		if (i < 10)
		{
			*tp++ = i+'0';
		}
		else
		{
			*tp++ = i + 'a' - 10;
		}
	}
	sp = string;

	if (sign)
		*sp++ = '-';

	while (tp > tmp)
		*sp++ = *--tp;
	*sp = 0;

	return;
}


/****************************************************************************/
/**
 * Write a 32-bit unsigned hex number to PmodOLEDrgb in Hex
 *
 * Writes  32-bit unsigned number to the pmodOLEDrgb display starting at the current
 * cursor position.
 *
 * @param num is the number to display as a hex value
 *
 * @return  *NONE*
 *
 * @note
 * No size checking is done to make sure the string will fit into a single line,
 * or the entire display, for that matter.  Watch your string sizes.
 *****************************************************************************/
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num)
{
	char  buf[9];
	int32_t   cnt;
	char  *ptr;
	int32_t  digit;

	ptr = buf;
	for (cnt = 7; cnt >= 0; cnt--) {
		digit = (num >> (cnt * 4)) & 0xF;

		if (digit <= 9)
		{
			*ptr++ = (char) ('0' + digit);
		}
		else
		{
			*ptr++ = (char) ('a' - 10 + digit);
		}
	}

	*ptr = (char) 0;
	OLEDrgb_PutString(InstancePtr,buf);

	return;
}


/****************************************************************************/
/**
 * Write a 32-bit number in Radix "radix" to LCD display
 *
 * Writes a 32-bit number to the LCD display starting at the current
 * cursor position. "radix" is the base to output the number in.
 *
 * @param num is the number to display
 *
 * @param radix is the radix to display number in
 *
 * @return *NONE*
 *
 * @note
 * No size checking is done to make sure the string will fit into a single line,
 * or the entire display, for that matter.  Watch your string sizes.
 *****************************************************************************/
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix)
{
	char  buf[16];

	PMDIO_itoa(num, buf, radix);
	OLEDrgb_PutString(InstancePtr,buf);

	return;
}

/*Gets called when there is change in gpio input*/

void SENSOR_A_Handler(void *p)
{
    counter++;
	XGpio_InterruptClear(&xSensorGPIOInstance,SENSOR_A_INTERUPT_MASK);
	//xil_printf("inside the sensor_A_handler\n");

}
/*Gets called when a 1 sec time is completed*/
void Second_Timer_Handler(void *p)
{
	u32 ctlsts;
	ctlsts=XTmrCtr_GetControlStatusReg(XPAR_AXI_TIMER_2_BASEADDR, TmrCtrNumber);
	ctlsts|=XTC_CSR_INT_OCCURED_MASK;
    XTmrCtr_SetControlStatusReg(SEC_TIMER_BASEADDR, TmrCtrNumber, ctlsts);
  if(prev_counter!=counter)
  {
    process_value=counter*60/2;
    OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 6, 2);
    OLEDrgb_PutString(&pmodOLEDrgb_inst,"         ");
    OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 6, 2);
    PMDIO_putnum(&pmodOLEDrgb_inst, process_value, 10);
  }

 prev_counter=counter;
 counter=0;

}

/**
Handlers the interrupt from the WDT timer
 */
void WDT_handler(void *p)
{


	xil_printf("Inside the WDT_handler");
	if(!force_crash && system_running)
	{
		XWdtTb_RestartWdt(&WDTInst);
		xil_printf("\n Inside the !force_crach && system_running\n");
		system_running = 0;
	}
	else
	{
		xil_printf("Force crash Successful. CPU Reset");
	}
	XWdtTb_IntrClear(&WDTInst);
}

/*Used to take the reading from the push button and the switch
 */
void Button_Switch_Handler_task(void *p)
{ design_parameters design_params;
  while(1)
  {
     if(xSemaphoreTake(binary_sem,0))
     {
    	 XGpio_InterruptClear(&xButtonSwitchGPIOInstance,BUTTON_INTERUPT_MASK);
    	 XGpio_InterruptClear(&xButtonSwitchGPIOInstance,SWITCH_INTERUPT_MASK);
    	 gpio_in=XGpio_DiscreteRead(&xButtonSwitchGPIOInstance, BUTTON_GPIO_0_CHANNEL_1);
    	 gpio_switch=XGpio_DiscreteRead(&xButtonSwitchGPIOInstance, SWITCH_GPIO_0_CHANNEL_2);
    	 XGpio_DiscreteWrite(&xLEDGPIOInstance,LED_GPIO_1_CHANNEL_0,gpio_switch);

    	 /*checked whether switch 15 is on. if it is on, the system force crash is enabled*/
    	 if((gpio_switch & 0x8000)  == 0x8000)
    	 {
    	 	xil_printf("Force crush inside switch 15\n");
    	 	force_crash = 1;
    	 }
    	 else
    	 {
    	 	xil_printf("Not a force crash\n");
    	 	force_crash = 0;
    	 }

    	 /*Checks whether switch 0 or 1 is on. If switch 0 is on, the speed is incremented by 5,else if switch 1 is on, the speed is increased by 10 else the speed is increased by 1*/
    	 if((gpio_switch&1)==1)
    	 {
    		 pmodENC_init(&pmodENC_inst,5, true);
    	 }
    	 else if((gpio_switch&2)==2)
    	 {
    		 pmodENC_init(&pmodENC_inst, 10, true);
    	 }
    	 else
    	 {
    		 pmodENC_init(&pmodENC_inst,1, true);
    	 }

       /*If btC is pressed, the system is resetted*/
    	 if((gpio_in&1)==1)
    	 	   {

    		    pmodENC_clear_count(&pmodENC_inst); // clears the count in pmod encoder.
    		 	design_params.kp=0;
    		 	design_params.ki=0;
    		 	design_params.kd=0;

    	 	   }

    	      /*If btR is pressed, the propotional constant is increased*/
    	 	   else if((gpio_in&8)==8)
    	 	   {
    	 		 /*The kd propotional constant is increased by 1 or 5 or 10*/
    	 	     if((gpio_switch&12)==12)
    	 	     {  if((gpio_switch&32)==32)
    	 	        {

    	 	          design_params.kd=design_params.kd+10;
    	 	        }
    	 	        else if((gpio_switch&16)==16)
    	 	        {

    	 	    	  design_params.kd=design_params.kd+5;
    	 	        }
    	 	        else
    	 	    	 design_params.kd=design_params.kd+1;

    	 	     }

    	 	    /*The ki propotional constant is increased by 1 or 5 or 10*/
    	 	     else if((gpio_switch&8)==8)
    	 	    {  if((gpio_switch&32)==32)
    	 	       {

    	 	         design_params.ki=design_params.ki+10;
    	 	       }
    	 	       else if((gpio_switch&16)==16)
    	 	      {
    	 	   	    design_params.ki=design_params.ki+5;
    	 	      }
    	 	      else
    	 	   	    ++design_params.ki;

    	 	    }

    	 	    /*The kp propotional constant is increased by 1 or 5 or 10*/
    	 	    else if((gpio_switch&4)==4)
    	 	    { if((gpio_switch&32)==32)
    	 	      {

    	 	        design_params.kp=design_params.kp+10;
    	 	      }
    	 	      else if((gpio_switch&16)==16)
    	 	      {

    	 	        design_params.kp=design_params.kp+5;
    	 	      }
    	 	      else
    	 	   	    ++design_params.kp;

    	 	    }
    	 	  }

    	      /*The kd propotional constant is decremented by 1 or 5 or 10*/
    	 	  else if((gpio_in&4)==4)
    	 	  {
    	 	    if((gpio_switch&12)==12)
    	 	    { if((gpio_switch&32)==32)
    	 	      {
    	 	    	  design_params.kd=((design_params.kd-10)<0)?0:(design_params.kd-10);

    	 	      }
    	 	      else if((gpio_switch&16)==16)
    	 	      {
    	 	          design_params.kd=((design_params.kd-5)<0)?0:(design_params.kd-5);

    	 	      }
    	 	      else
    	 	      {
    	 	          design_params.kd=((design_params.kd-1))<0?0:(design_params.kd-1);

    	 	      }

    	 	      }

    	 	      /*The ki propotional constant is decremented by 1 or 5 or 10*/
    	 	      else if((gpio_switch&8)==8)
    	 	      {  if((gpio_switch&32)==32)
    	 	        {
    	 	    	  design_params.ki=((design_params.ki-10)<0)?0:(design_params.ki-10);

    	 	        }
    	 	       else if((gpio_switch&16)==16)
    	 	       {
    	 	    	  design_params.ki=((design_params.ki-5)<0)?0:(design_params.ki-5);

    	 	       }
    	 	       else
    	 	       {
    	 	    	   design_params.ki=((design_params.ki-1)<0)?0:(design_params.ki-1);
    	 	       }

    	 	      }

    	 	      /*The kp propotional constant is decremented by 1 or 5 or 10*/
    	 	      else if((gpio_switch&4)==4)
    	 	      { if((gpio_switch&32)==32)
    	 	        {
    	 	    	  design_params.kp=((design_params.kp-10)<0)?0:(design_params.kp-10);
    	 	        }
    	 	       else if((gpio_switch&16)==16)
    	 	       {
    	 	    	  design_params.kp=((design_params.kp-5)<0)?0:(design_params.kp-5);
    	 	       }
    	 	       else
    	 	       {
    	 	           design_params.kp=design_params.kp-1;
    	 	           if(design_params.kp<0)
    	 	           {
    	 	        	   design_params.kp=0;
    	 	           }

    	 	       }

    	 	      }
    	 	    }
    	 	}
    /* Will read the count from the pmodENC*/
     else
     {
    	 pmodENC_read_count(&pmodENC_inst, &RotaryCnt);
    	 design_params.duty_cycle=(RotaryCnt*100)/255;
    	 xQueueSend(xDisplayQueue,&design_params,0);
    	 xQueueSend(xPIDQueue, &design_params,0);
    	 if(pmodENC_is_switch_on(&pmodENC_inst)&&RotaryCnt==0)
    	 {
    	 			   XGpio_DiscreteWrite(&xDIRGPIOInstance, DIR_GPIO_2_CHANNEL_0, 1);
    	 }
    	 else if(RotaryCnt==0)
    	 {
    	 			   XGpio_DiscreteWrite(&xDIRGPIOInstance, DIR_GPIO_2_CHANNEL_0, 0);
         }
  }

}
}

/* Displays the changes on the pmodOLED*/
void display_task(void *p)
{   design_parameters actual_design_params,temp_design_params;
	while(1)
	{
		xQueueReceive(xDisplayQueue,&actual_design_params,100);

		/* Changes the display value for duty cycle when the value changes*/
		if(temp_design_params.duty_cycle!=actual_design_params.duty_cycle)
		{
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 2);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"    ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 2);
			PMDIO_putnum(&pmodOLEDrgb_inst,actual_design_params.duty_cycle,10);
			temp_design_params.duty_cycle=actual_design_params.duty_cycle;
		}

		/* Changes the display value for kp when the value changes*/
		if(temp_design_params.kp!=actual_design_params.kp)
		{
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 5);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"    ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 5);
			PMDIO_putnum(&pmodOLEDrgb_inst,actual_design_params.kp,10);
			temp_design_params.kp=actual_design_params.kp;
		}

		/* Changes the display value for ki when the value changes*/
		if(temp_design_params.ki!=actual_design_params.ki)
		{
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4,5);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"    ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 5);
			PMDIO_putnum(&pmodOLEDrgb_inst,actual_design_params.ki,10);
			temp_design_params.ki=actual_design_params.ki;
		}

		/* Changes the display value for kd when the value changes*/
		if(temp_design_params.kd!=actual_design_params.kd)
		{
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 8, 5);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"    ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 8, 5);
			PMDIO_putnum(&pmodOLEDrgb_inst,actual_design_params.kd,10);
			temp_design_params.kd=actual_design_params.kd;
		}


	}

}

/*The PID control logic with the changing the speed of the motor*/
void PID_task(void *p)
{
  design_parameters actual_design_params,temp_design_params;
  while(1)
  {
   xQueueReceive(xPIDQueue,&actual_design_params,0);      // Recieve the data from the Input_task.
   actual_design_params.set_RPM = RotaryCnt*53;
   actual_design_params.RC_Process_Value = process_value/53;
   actual_design_params.error = RotaryCnt - actual_design_params.RC_Process_Value;				//Proportional error
   actual_design_params.integrate_error= actual_design_params.integrate_error + actual_design_params.error;			// Integral Error
   	if(actual_design_params.integrate_error > 5000)
   	{
   		actual_design_params.integrate_error = 5000;
   	}
   	else if (actual_design_params.integrate_error < -5000)
   	{
   		actual_design_params.integrate_error = -5000;
   	}
   	else
   	{
   	 actual_design_params.integrate_error = actual_design_params.integrate_error;  // integral error
   	}
   	actual_design_params.diff_error = temp_design_params.error - actual_design_params.error;				// Differential error
   	actual_design_params.final_rotary_count = RotaryCnt + (actual_design_params.kp/10)*(actual_design_params.error) + (actual_design_params.kd)*(actual_design_params.diff_error) + (actual_design_params.ki/100)*(actual_design_params.integrate_error);
   	temp_design_params.error=actual_design_params.error;
    if(actual_design_params.final_rotary_count < 0)
   	{
   		actual_design_params.final_rotary_count = 0;
   	}
   	else if(actual_design_params.final_rotary_count > 255)
   	{
   	 actual_design_params.final_rotary_count = 255;
   	}

   	actual_design_params.duty_cycle = (actual_design_params.final_rotary_count*100)/255;
    uint32_t status;
    status = PWM_SetParams(&InstancePtr, 4*(10^3),actual_design_params.duty_cycle);
    //xil_printf("actual_design_params.duty_cycle is %d",actual_design_params.duty_cycle);
    if (status != 0)
    {
    	xil_printf("The PWM_SetParams is not working.");
    	break;
     }
  }
}

/* Setting the direction of the GPIO with interrupts. If you are using only one channel, send channel_x_ID(x=1,2) =0 for the channel which is not used*/
int Gpio_Direction_Interrupt_Initialization(XGpio gpioInstance,bool isDual,int interruptID,int channel_1_MASK, int channel_2_MASK,XInterruptHandler interrupt_handler)
{
    portBASE_TYPE xStatus;


    xStatus = xPortInstallInterruptHandler(interruptID ,interrupt_handler, NULL ); // this will install the interupt handler

    if( xStatus == pdPASS )
    {
     if(isDual)
	 {
	  XGpio_SetDataDirection( &gpioInstance, 2, 1 );
	 }

     XGpio_SetDataDirection(&gpioInstance,1,1);

	/* Enable the button and switch interupt interrupts in the interrupt controller.
	 *NOTE* The vPortEnableInterrupt() API function must be used for this
	 purpose. */
	vPortEnableInterrupt(interruptID);

    /* Enable GPIO channel interrupts. */
    XGpio_InterruptEnable( &gpioInstance,channel_1_MASK);

    /*Checks if two channels are enabled in the gpio, if it is, then the interrupt for that gpio is enabled*/
    if(isDual)
    {
	 XGpio_InterruptEnable( &gpioInstance,channel_2_MASK);
    }
	XGpio_InterruptGlobalEnable( &gpioInstance );
   }
   configASSERT( ( xStatus == pdPASS ) );

  return  xStatus;
}





