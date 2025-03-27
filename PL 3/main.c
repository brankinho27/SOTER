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
 * Messages Queues
 * 2017-2018
 *
 */

/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <lcd.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lcd.h"

/* Queue include. */
#include "queue.h"

/* Task priorities. */
#define mainFLASH_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
#define mainUSART_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
#define mainLCD_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)

/* The rate at which the flash task toggles the LED. */
#define mainFLASH_DELAY			( ( TickType_t ) 1000 / portTICK_RATE_MS )
/* The rate at which the temperature is read. */
#define mainTEMP_DELAY			( ( TickType_t ) 100 / portTICK_RATE_MS )

/* Configure RCC clocks */
static void prvSetupRCC( void );

/* Configure GPIO. */
static void prvSetupGPIO( void );

/* Configure NVIC. */
static void prvSetupNVIC( void );

/* Configure the ADC */
static void prvSetupADC( void );

/* Simple LED toggle task. */
static void prvFlashTask1( void *pvParameters );

/* LCD activity task. */
static void prvLcdTask( void *pvParameters );

/* ADC temperature read task. */
static void prvTempTask( void *pvParameters );


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

/* Task 3 handle variable. */
TaskHandle_t HandleTask3;

/* Queues handle variable. */
volatile QueueHandle_t xQueue1;
volatile QueueHandle_t xQueue2;
volatile QueueHandle_t xQueue3;
volatile QueueHandle_t xQueue4;

/* Temperature Stats Struct */
typedef struct {
	uint32_t ticks;
	int32_t temperature;
} TempStats;

int main( void )
{
	/*Setup the hardware, RCC, GPIO, etc...*/
    prvSetupRCC();
    prvSetupGPIO();
    prvSetupADC();
    prvSetupUSART2();
    //prvSetupNVIC();

    /* Create the Queue 1*/
//    xQueue1 = xQueueCreate(10, sizeof(char));
//    if (xQueue1 == 0) {
//    	/* Queue was not created and must not be used. */
//    } else {
//    	/* Queue created successfully. */
//    }

    /* Create the Queue 2*/
//    xQueue2 = xQueueCreate(10, sizeof(TickType_t));
//    if (xQueue2 == 0) {
//    	/* Queue was not created and must not be used. */
//    } else {
//    	/* Queue created successfully. */
//    }

    /* Create the Queue 3*/
//    xQueue3 = xQueueCreate(10, sizeof(TickType_t));
//    if (xQueue3 == 0) {
//    	/* Queue was not created and must not be used. */
//    } else {
//    	/* Queue created successfully. */
//    }

    /* Create the Queue 4*/
    xQueue4 = xQueueCreate(10, sizeof(TempStats));
    if (xQueue4 == 0) {
    	/* Queue was not created and must not be used. */
    } else {
    	/* Queue created successfully. */
    }

	/* Create the tasks */
 	xTaskCreate( prvLcdTask, "Lcd", configMINIMAL_STACK_SIZE, NULL, mainLCD_TASK_PRIORITY, &HandleTask1 );
 	xTaskCreate( prvFlashTask1, "Flash1", configMINIMAL_STACK_SIZE, NULL, mainFLASH_TASK_PRIORITY, &HandleTask2 );
    xTaskCreate( prvTempTask, "Temp", configMINIMAL_STACK_SIZE+100, NULL, mainFLASH_TASK_PRIORITY, &HandleTask3 );

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}
/*-----------------------------------------------------------*/


static void prvFlashTask1( void *pvParameters )
{
    TickType_t xLastExecutionTime;

    xLastExecutionTime = xTaskGetTickCount();

    for( ;; )
	{
    	vTaskDelayUntil( &xLastExecutionTime, mainFLASH_DELAY );
		GPIO_WriteBit(GPIOB, GPIO_Pin_0, (1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0)));
	}
}

/*-----------------------------------------------------------*/

// EXERCICIO 1
//static void prvLcdTask( void *pvParameters )
//{
//	lcd_init ( );
//
//	for( ;; )
//	{
//		char cBuffer;
//		if (xQueue1 != 0) {
//			xQueueReceive(xQueue1, &cBuffer, (TickType_t) portMAX_DELAY);
//	        lcd_draw_char( 63-(5*10)/2, 79-(7*10)/2, cBuffer, 0xFFFF, 10 );
//	        vTaskDelay( ( TickType_t ) 2000 / portTICK_RATE_MS);
//		}
//	}
//}
/*-----------------------------------------------------------*/


// EXERCICIO 2
//static void prvLcdTask( void *pvParameters )
//{
//	lcd_init ( );
//
//	for( ;; )
//	{
//		TickType_t xFirstTick;
//		TickType_t xLastTick;
//		if (xQueue2 != 0) {
//			xQueueReceive(xQueue2, &xFirstTick, (TickType_t) portMAX_DELAY);
//			xLastTick = xFirstTick;
//			vTaskDelay( ( TickType_t ) 500 / portTICK_RATE_MS);
//			while (uxQueueMessagesWaitingFromISR(xQueue2) != 0) xQueueReceive(xQueue2, &xLastTick, (TickType_t) portMAX_DELAY);
//
//			if (xFirstTick != xLastTick || xLastTick > xFirstTick + 50) {
//				lcd_draw_char( 63-(5*10)/2, 79-(7*10)/2, '2', 0xFFFF, 10 );
//			} else lcd_draw_char( 63-(5*10)/2, 79-(7*10)/2, '1', 0xFFFF, 10 );
//		}
//	}
//}
/*-----------------------------------------------------------*/


// EXERCICIO 3
//static void prvLcdTask( void *pvParameters )
//{
//	lcd_init ( );
//
//	for( ;; )
//	{
//		int32_t xBuffer;
//		char lcd_string[32];
//		if (xQueue3 != 0) {
//			xQueueReceive(xQueue3, &xBuffer, (TickType_t) portMAX_DELAY);
//
//			sprintf(lcd_string, "%d    ", xBuffer);
//			lcd_draw_string( 30, 50, "TEMP", 0xFFFF, 3 );
//			lcd_draw_string( 38, 80, lcd_string, 0xFFFF, 3 );
//		}
//	}
//}
/*-----------------------------------------------------------*/


// EXERCICIO 4
static void prvLcdTask( void *pvParameters )
{
	lcd_init ( );

	for( ;; )
	{
		TempStats xBuffer;
		char lcd_string[32];
		char usart_string[32];
		if (xQueue4 != 0) {
			xQueueReceive(xQueue4, &xBuffer, (TickType_t) portMAX_DELAY);

			sprintf(lcd_string, "%d    ", xBuffer.temperature);
			lcd_draw_string( 30, 50, "TEMP", 0xFFFF, 3 );
			lcd_draw_string( 38, 80, lcd_string, 0xFFFF, 3 );

			sprintf(usart_string, "%d %d\r\n", xBuffer.ticks, xBuffer.temperature);
			prvSendMessageUSART2(usart_string);
		}
	}
}
/*-----------------------------------------------------------*/


// EXERCICIO 3~4
static void prvTempTask( void *pvParameters )
{
    TickType_t xLastExecutionTime;
    int ADC1ConvertedValue=0;
    int32_t temp;

    xLastExecutionTime = xTaskGetTickCount();

    TempStats TempStats_1 = {0, 0};

    for( ;; )
	{
    	vTaskDelayUntil ( &xLastExecutionTime, mainTEMP_DELAY);
        /* Read Sensor */
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        while( ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) != SET );

        ADC1ConvertedValue = ADC_GetConversionValue(ADC1);
        ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

        /*work with x 1000 to avoid float*/
        temp = (int32_t) ADC1ConvertedValue;
        temp = ( temp * 3300 ) / 4096; // finds mv
        temp = ((( 1400 - temp ) * 1000 ) / 430 ) + 250;

        /*The temp variable has the temperature value times 10 */

        //xQueueSendToBack(xQueue3, &temp, (TickType_t) 10);

        TempStats_1.ticks = xTaskGetTickCount();
        TempStats_1.temperature = temp;
        xQueueSendToBack(xQueue4, &TempStats_1, (TickType_t) 10);
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

    // Configuração da interrupção EXTI1
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Configuração da interrupção USART2
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}
/*-----------------------------------------------------------*/


static void prvSetupGPIO( void )
{
    /* GPIO configuration */
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOA GPIOB clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE );

    // GPIOB0 - Led 0
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
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


static void prvSetupADC( void )
{
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16,1, ADC_SampleTime_239Cycles5);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

}
/*-----------------------------------------------------------*/
