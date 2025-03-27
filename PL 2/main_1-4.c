/**
  ******************************************************************************
  * @file    main.c
  * @author  3S0 FreeRTOs
  * @version V1.0
  * @date    24/10/2017
  * @brief   FreeRTOS Example project.
  ******************************************************************************
*/


/*
 *
 * Mutex
 * 2017-2018
 *
 */

/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "lcd.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Mutex includes. */
#include "semphr.h"

/* Task priorities. */
//#define mainFLASH_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
//#define mainUSART_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)

#define mainLCD_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)

/* The rate at which the flash task toggles the LED. */
#define mainFLASH_DELAY			( ( TickType_t ) 1000 / portTICK_RATE_MS )

/* The rate at LCD is refreshed. */
#define mainLCD_DELAY			( ( TickType_t ) 250 / portTICK_RATE_MS )

///* The rate at which the message is sent to the USART*/
//#define mainUSART_DELAY			( ( TickType_t ) 1000 / portTICK_RATE_MS )

/* Configure RCC clocks */
static void prvSetupRCC( void );

/* Configure GPIO. */
static void prvSetupGPIO( void );

/* Configure NVIC. */
static void prvSetupNVIC( void );

/* Tasks */
static void prvPA1poolingTask( void *pvParameters );
static void prvLedTask( void *pvParameters );
static void prvLcdTask( void *pvParameters );


/********** Useful functions **********/
/* USART2 configuration. */
static void prvSetupUSART2( void );

/* USART2 send message. */
static void prvSendMessageUSART2(char *message);

/***************************************/


/* Task 1 handle variable. */
TaskHandle_t HandleTask1;

/* Task 2 handle variable. */
TaskHandle_t HandleTask2;

/* Task 2 handle variable. */
TaskHandle_t HandleTask3;

/* Semaphore Handler */
volatile SemaphoreHandle_t xSemaphore;

int main( void )
{
	/*Setup the hardware, RCC, GPIO, etc...*/
    prvSetupRCC();
    prvSetupGPIO();
    prvSetupUSART2();
    prvSetupNVIC();

//    /* Create the Mutex */
//    xSemaphore = xSemaphoreCreateMutex();
//    if (xSemaphore == NULL) {
//    	/* Error creating the semaphore */
//    }

    /* Create the Binary Semaphore */
    xSemaphore = xSemaphoreCreateBinary();
    if (xSemaphore == NULL) {
    	/* Error creating the semaphore */
    }

	/* Create the tasks */
 	xTaskCreate( prvLcdTask, "Lcd", configMINIMAL_STACK_SIZE, NULL, mainLCD_TASK_PRIORITY, &HandleTask1 );
 	//xTaskCreate( prvPA1poolingTask, "PA1pooling", configMINIMAL_STACK_SIZE, NULL, mainLCD_TASK_PRIORITY+1, &HandleTask2 );
 	xTaskCreate( prvLedTask, "Led", configMINIMAL_STACK_SIZE, NULL, mainLCD_TASK_PRIORITY+1, &HandleTask3 );
 	//xTaskCreate( prvFlashTask1, "Flash1", configMINIMAL_STACK_SIZE, NULL, mainFLASH_TASK_PRIORITY, &HandleTask2 );
 	//xTaskCreate( prvUsartTask, "Usart", configMINIMAL_STACK_SIZE, NULL, mainUSART_TASK_PRIORITY, &HandleTask3 );

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}
///*-----------------------------------------------------------*/
//
//static void prvFlashTask1( void *pvParameters )
//{
//    TickType_t xLastExecutionTime;
//
//    xLastExecutionTime = xTaskGetTickCount();
//
//    for( ;; )
//	{
//    	vTaskDelayUntil( &xLastExecutionTime, mainFLASH_DELAY );
//
//    	xSemaphoreTake(xSemaphore, (TickType_t) portMAX_DELAY);
//		GPIO_WriteBit(GPIOB, GPIO_Pin_0, (1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0)));
//		prvSendMessageUSART2("The LED was toggled...\r\n");
//		xSemaphoreGive(xSemaphore);
//
//	}
//}
///*-----------------------------------------------------------*/
//
//static void prvUsartTask( void *pvParameters )
//{
//    TickType_t xLastExecutionTime;
//
//    xLastExecutionTime = xTaskGetTickCount();
//
//    for( ;; )
//	{
//        vTaskDelayUntil( &xLastExecutionTime, mainUSART_DELAY );
//        xSemaphoreTake(xSemaphore, (TickType_t) portMAX_DELAY);
//        prvSendMessageUSART2("This is just a periodic message...\r\n");
//        xSemaphoreGive(xSemaphore);
//	}
//}
///*-----------------------------------------------------------*/

static void prvLcdTask( void *pvParameters )
{
    TickType_t xLastExecutionTime;
    char run_id[]="-\\|/-\\|/";
    uint8_t running_var=0;

    lcd_init ( );

    xLastExecutionTime = xTaskGetTickCount();
	for( ;; )
	{
        vTaskDelayUntil( &xLastExecutionTime, mainLCD_DELAY );
        lcd_draw_char( 63-(5*4)/2, 79-(7*4)/2, run_id[running_var] , 0xFFFF, 4 );

        running_var++;
        if(running_var==8) running_var=0;
    }
}
/*-----------------------------------------------------------*/

//static void prvPA1poolingTask( void *pvParameters )
//{
//    TickType_t xLastExecutionTime;
//
//    xLastExecutionTime = xTaskGetTickCount();
//
//    for( ;; )
//	{
//    	vTaskDelayUntil( &xLastExecutionTime, mainFLASH_DELAY );
//
//        while( GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == SET );
//        xSemaphoreGive(xSemaphore);
//	}
//}
///*-----------------------------------------------------------*/

static void prvLedTask( void *pvParameters )
{
    TickType_t xLastExecutionTime;

    xLastExecutionTime = xTaskGetTickCount();

    for( ;; )
	{
    	vTaskDelayUntil( &xLastExecutionTime, mainFLASH_DELAY );

        xSemaphoreTake(xSemaphore, (TickType_t) portMAX_DELAY);
        GPIO_WriteBit(GPIOB, GPIO_Pin_1, (1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_1)));
	}
}
/*-----------------------------------------------------------*/

static void prvSetupRCC( void )
{
    /* RCC configuration - 72 MHz */
    ErrorStatus HSEStartUpStatus;

    RCC_DeInit();
    /*Enable the HSE*/
    RCC_HSEConfig(RCC_HSE_ON);
    /* Wait untill HSE is ready or time out */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if(HSEStartUpStatus == SUCCESS)
    {
        /* Enable The Prefetch Buffer */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        /* 72 MHZ - 2 wait states */
        FLASH_SetLatency(FLASH_Latency_2);

        /* No division HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        /* PCLK1 = HCLK/2 (36MHz) */
        RCC_PCLK1Config(RCC_HCLK_Div2);
        /* PCLK2 = HCLK (72MHz)*/
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* Use PLL with HSE=12MHz */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6);
        /* Enable the PLL */
        RCC_PLLCmd(ENABLE);
        /* Wait for PLL ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET );

        /* Select the PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        /* Wait until PLL is used as system clock */
        while( RCC_GetSYSCLKSource() != 0x08 );
    }
    else
    {
        while(1);
    }
}
/*-----------------------------------------------------------*/

static void prvSetupGPIO( void )
{
    /* GPIO configuration */
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOA GPIOB clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE );

    // GPIOB1 - Led 1
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // GPIOA1 - SW5
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}
/*-----------------------------------------------------------*/

static void prvSetupNVIC( void )
{
	// Configuração das Prioridades Group 1
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // EXTI1
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
    // USART2
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    // Confiuração da interrupção EXTI1
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Confiuração da interrupção USART2
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}


void prvSetupUSART2( void )
{
USART_InitTypeDef USART_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

    /* USART2 is configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - 1 Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled */

    /* Enable GPIOA clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE );

    /* USART Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    /* Configure the USART2 */
    USART_Init(USART2, &USART_InitStructure);
    /* Enable the USART2 */
    USART_Cmd(USART2, ENABLE);
 }

/*-----------------------------------------------------------*/

static void prvSendMessageUSART2(char *message)
{
uint16_t cont_aux=0;

    while(cont_aux != strlen(message))
    {
        USART_SendData(USART2, (uint8_t) message[cont_aux]);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
        {
        }
        cont_aux++;
    }
}
/*-----------------------------------------------------------*/
