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
static void prvFlashTask4( void *pvParameters );


/********** Useful functions **********/
/* USART2 configuration. */
static void prvSetupUSART2( void );

/* USART2 send message. */
static void prvSendMessageUSART2(char *message);

/***************************************/


/* Task 1 handle variable. */
TaskHandle_t HandleTask1;
TaskHandle_t HandleTask2;
TaskHandle_t HandleTask3;
TaskHandle_t HandleTask4;


int main( void )
{
	/*Setup the hardware, RCC, GPIO, etc...*/
    prvSetupRCC();
    prvSetupGPIO();
    prvSetupUSART2();

	/* Create the tasks */
 	xTaskCreate( prvFlashTask1, "Flash1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &HandleTask1 );
 	xTaskCreate( prvFlashTask2, "Flash2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &HandleTask2 );
 	//xTaskCreate( prvFlashTask3, "Flash3", configMINIMAL_STACK_SIZE, NULL, 1, &HandleTask3 );
 	xTaskCreate( prvFlashTask4, "Flash4", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, &HandleTask4 );

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
		vTaskDelayUntil(&xLastExecutionTime, (TickType_t)10/portTICK_RATE_MS );

        /* Toggle the LED B0*/
		GPIO_WriteBit(GPIOB, GPIO_Pin_0, 1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0 ) );

		/*Write USART*/
		prvSendMessageUSART2( "Mensagen enviada ao fim de 10 ms a partir de uma tarefa de baixa prioridade.\r\n" );
	}

}

static void prvFlashTask2( void *pvParameters )
{
    /* Declare the variable xLastExecutionTime */
    TickType_t xLastExecutionTime;
    /* Initialize xLastExecutionTime */
    xLastExecutionTime = xTaskGetTickCount();
    for( ;; )
	{
		/* Block 2 seconds. */
    	vTaskDelayUntil(&xLastExecutionTime, (TickType_t)10/portTICK_RATE_MS );

        /* Toggle the LED B1*/
		GPIO_WriteBit(GPIOB, GPIO_Pin_1, 1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_1 ) );

		/*Write USART*/
		prvSendMessageUSART2("O segundo LED mudou de estado agora mesmo a partir de uma tarefa de alta prioridade.\r\n");
	}
}

static void prvFlashTask3( void *pvParameters )
{
    for( ;; )
	{
    	//Suspend first task.
		vTaskSuspend(HandleTask1);

    	/* Block 10 second. */
		vTaskDelay( ( TickType_t ) 10000 / portTICK_PERIOD_MS  );

		//Resume first task.
		vTaskResume(HandleTask1);

    	/* Block 10 second. */
		vTaskDelay( ( TickType_t ) 10000 / portTICK_PERIOD_MS  );
	}
}

static void prvFlashTask4( void *pvParameters )
{
    for( ;; )
	{
    	while(1);
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
 *
 * QUESTÃO 8:
 * Com a preempção a 1, há uma interrupção quando uma tarefa de maior prioridade é "chamada" enquanto
 * uma de menor prioridade está a decorrer, ou seja, a mensagem que se pode observar na USART contém
 * partes de ambas as tarefas visto que a segunda interrompe a primeira enquanto esta ainda está a
 * escrever. Quando se define a 0 a preempção, essa interrupção deixa de existir, isto é, continua
 * a existir uma tarefa com maior prioridade mas que não pode interromper uma tarefa que já esteja
 * em funcionamento.
 *
 *QUESTÃO 9:
 *Sendo que a preempção está a 1, há a possibilidade de uma tarefa de maior prioridade interromper
 *uma de menor prioridade. Como a tarefa número 4 tem prioridade superior às outras duas e como
 *tem um ciclo infinito, o programa fica "parado" na tarefa 4.
 *Relativamente à segunda questão, neste contexto, o método mais adequado seria o método por interrupção
 *visto que as tarefas 1 e 2 não entram em funcionamento enquanto a tarefa 4 não terminar. Se for utilizado
 *o método de pooling, são realizadas verificações que não são necessárias e que consomem CPU visto que
 *o programa está "parado".
 *
 */

