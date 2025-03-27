/**
  ******************************************************************************
  * @file    main.c
  * @author  3S0 FreeRTOS nnd@isep.ipp.pt
  * @version V1.1
  * @date    28/11/2018
  * @brief   SISTR/SOTER FreeRTOS Example project
  ******************************************************************************
*/


/*
 *
 * LED blink
 *
 */

/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"


 /* Configure RCC clock at 72 MHz */
static void prvSetupRCC( void );

 /* Configure GPIO. */
static void prvSetupGPIO( void );

/* Simple LED toggle task. */
static void prvFlashTask1( void *pvParameters );
static void prvFlashTask2( void *pvParameters );
static void prvFlashTask3( void *pvParameters );


/********** Useful functions **********/
/* USART2 configuration. */
static void prvSetupUSART2( void );

/* USART2 send message. */
static void prvSendMessageUSART2(char *message);

/***************************************/


/* Task  handle variables. */
TaskHandle_t HandleTask1;
TaskHandle_t HandleTask2;
TaskHandle_t HandleTask3;

/* Tasks status variables. */
TaskStatus_t xTaskDetails1;
TaskStatus_t xTaskDetails2;
TaskStatus_t xTaskDetails3;

uint16_t RGB_Color_Convert(char r, char g, char b) {
	// Conversão RGB 24 bits para RGB 16 bits (R: 5 bits / G: 6 bits / B: 5 bits)
	r = (r*31)/255;
    g = (g*63)/255;
    b = (b*31)/255;
    return ((b) | (g << 5) | (r << 11));
}

int main( void )
{
	/*Setup the hardware, RCC, GPIO, etc...*/
    prvSetupRCC();
    prvSetupGPIO();
    prvSetupUSART2();
    lcd_init();

	/* Create the tasks */
 	xTaskCreate( prvFlashTask1, "Flash1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &HandleTask1 );
 	xTaskCreate( prvFlashTask2, "Flash2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &HandleTask2 );
 	xTaskCreate( prvFlashTask3, "Flash3", configMINIMAL_STACK_SIZE+100, NULL, 1, &HandleTask3 );

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}
/*-----------------------------------------------------------*/



static void prvFlashTask1( void *pvParameters )
{
    /* Declare the variable xLastExecutionTime */
    TickType_t xLastExecutionTime;
    /* Initialize xLastExecutionTime */
    xLastExecutionTime = xTaskGetTickCount();
    for( ;; )
	{
    	/* Block 1 second. */
		vTaskDelayUntil(&xLastExecutionTime, (TickType_t)1000/portTICK_RATE_MS );

        /* Toggle the LED B0*/
		GPIO_WriteBit(GPIOB, GPIO_Pin_0, 1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0 ) );

	}

}

static void prvFlashTask2( void *pvParameters )
{
    /* Declare the variable xLastExecutionTime */
    TickType_t xLastExecutionTime;
    /* Initialize xLastExecutionTime */
    xLastExecutionTime = xTaskGetTickCount();

    char buffer_lcd[32];

    for( ;; )
	{
		/* Block 0.5 seconds. */
    	vTaskDelayUntil(&xLastExecutionTime, (TickType_t)500/portTICK_RATE_MS );

    	sprintf(buffer_lcd, "TICK COUNT: %d", xTaskGetTickCount());
    	lcd_draw_string(10, 10 , buffer_lcd, RGB_Color_Convert(255, 0, 0), 1);

    	sprintf(buffer_lcd, "NO. TASKS: %d", uxTaskGetNumberOfTasks());
    	lcd_draw_string(10, 20 , buffer_lcd, RGB_Color_Convert(255, 0, 0), 1);

    	vTaskGetInfo(HandleTask1, &xTaskDetails1, pdTRUE, eInvalid);
    	sprintf(buffer_lcd, "%s: %d", xTaskDetails1.pcTaskName, xTaskDetails1.uxBasePriority);
    	lcd_draw_string(10, 40 , buffer_lcd, RGB_Color_Convert(255, 0, 0), 1);

    	vTaskGetInfo(HandleTask2, &xTaskDetails2, pdTRUE, eInvalid);
    	sprintf(buffer_lcd, "%s: %d", xTaskDetails2.pcTaskName, xTaskDetails2.uxBasePriority);
    	lcd_draw_string(10, 50 , buffer_lcd, RGB_Color_Convert(255, 0, 0), 1);

    	vTaskGetInfo(HandleTask3, &xTaskDetails3, pdTRUE, eInvalid);
    	sprintf(buffer_lcd, "%s: %d", xTaskDetails3.pcTaskName, xTaskDetails3.uxBasePriority);
    	lcd_draw_string(10, 60 , buffer_lcd, RGB_Color_Convert(255, 0, 0), 1);
	}
}


static void prvFlashTask3( void *pvParameters )
{
    /* Declare the variable xLastExecutionTime */
    TickType_t xLastExecutionTime;
    /* Initialize xLastExecutionTime */
    xLastExecutionTime = xTaskGetTickCount();

    char pcWriteBuffer[200];

    for( ;; )
	{
    	/* Block 1 second. */
		vTaskDelayUntil(&xLastExecutionTime, (TickType_t)1000/portTICK_RATE_MS );

		/*Write USART*/
		vTaskList(pcWriteBuffer);
		prvSendMessageUSART2("\nName		State	Prio	Stack	Num\n");
		prvSendMessageUSART2(pcWriteBuffer);
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

        /* No division HCLK = SYSCLK = 72 MHz*/
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        /* PCLK1 = HCLK/2 (36MHz) */
        RCC_PCLK1Config(RCC_HCLK_Div2);
        /* PCLK2 = HCLK (72MHz)*/
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* Use PLL with HSE = 12 MHz (12 MHz * 6 = 72 MHz) */
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
    	/* HSE error? No further action */
        while(1);
    }
}
/*-----------------------------------------------------------*/



static void prvSetupGPIO( void )
{
    /* GPIO configuration */
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOB clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE );

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

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


/* This is a blocking send USART function */
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


/*
 * QUESTÃO 4:
 * Tendo em conta que a tarefa 2 não sofreu qualquer alteração, nota-se um
 * delay na tarefa 1 causada pela função for. A cada ciclo, o tempo consumido é superior
 * perdendo cada vez mais a sincronização anterior.
 *
 * QUESTÃO 5:
 * Neste caso, a sincronização mantém-se visto que o delay utilizado nas tarefas tem em conta
 * o tempo em que a tarefa foi iniciada, ou seja, o tempo é absoluto.
 *
 * QUESTÃO 6:
 * As mensagens foram enviadas com sucesso para a USART.
 *
 * QUESTÃO 7:
 * Tendo em conta que a prioridade da primeira tarefa é menor do que a prioridade da segunda
 * tarefa, é de notar que a mensagem enviada para a USART em primeiro lugar é a que é enviada
 * pela segunda tarefa seguida pela mensagem enviada pela primeira tarefa.
 */

