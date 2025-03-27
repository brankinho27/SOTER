/*
  ******************************************************************************
  * @file    main.c
  * @authors Rodrigo Branquinho & José Soares
  * @version V1.3
  * @date    09/02/2022
  * @brief   Jogo Maze Runner @ FreeRTOS
  ******************************************************************************
*/

/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <lcd.h>

/* .h includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lcd.h"
#include "semphr.h"
#include "queue.h"

/* Tasks lower priority. */
#define mainGAME_CORE_PRIORITY	(tskIDLE_PRIORITY + 1)

/* Game Mazes (3 níveis). */
static char maze_1[21][21] = {
		{'#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#'},
		{'#','O',' ',' ',' ',' ','#',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ','#',' ','#'},
		{'#','#','#','#','#',' ','#',' ','#','#','#','#','#','#','#','#','#',' ','#',' ','#'},
		{'#',' ',' ',' ','#',' ','#',' ',' ',' ','#',' ',' ',' ',' ',' ','#',' ','#',' ','#'},
		{'#',' ','#',' ','#',' ','#','#','#',' ','#',' ','#','#','#',' ','#',' ','#',' ','#'},
		{'#',' ','#',' ','#',' ',' ',' ',' ',' ','#',' ',' ',' ','#',' ','#',' ',' ',' ','#'},
		{'#',' ','#',' ','#','#','#','#','#','#','#','#','#','#','#',' ','#','#','#',' ','#'},
		{'#',' ','#',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ','#',' ','#',' ',' ',' ','#'},
		{'#','#','#',' ','#','#','#','#','#','#','#','#','#',' ','#',' ','#',' ','#','#','#'},
		{'#',' ',' ',' ','#',' ',' ',' ',' ',' ','#',' ',' ',' ',' ',' ','#',' ','#',' ','#'},
		{'#',' ','#',' ','#',' ','#','#','#',' ','#','#','#','#','#','#','#',' ','#',' ','#'},
		{'#',' ','#',' ','#',' ','#',' ','#',' ',' ',' ',' ',' ',' ',' ',' ',' ','#',' ','#'},
		{'#',' ','#',' ','#',' ','#',' ','#','#','#','#','#','#','#','#','#','#','#',' ','#'},
		{'#',' ','#',' ','#',' ','#',' ',' ',' ',' ',' ','#',' ',' ',' ','#',' ',' ',' ','#'},
		{'#',' ','#','#','#',' ','#',' ','#','#','#',' ','#','#','#',' ','#',' ','#',' ','#'},
		{'#',' ','#',' ',' ',' ','#',' ','#',' ','#',' ',' ',' ','#',' ',' ',' ','#',' ','#'},
		{'#',' ','#',' ','#','#','#',' ','#',' ','#','#','#',' ','#','#','#','#','#',' ','#'},
		{'#',' ','#',' ','#',' ',' ',' ','#',' ',' ',' ','#',' ','#',' ',' ',' ','#',' ','#'},
		{'#',' ','#',' ','#',' ','#','#','#',' ','#','#','#',' ','#',' ','#',' ','#',' ','#'},
		{'#',' ',' ',' ',' ',' ','#',' ',' ',' ',' ',' ',' ',' ',' ',' ','#',' ',' ',' ','X'},
		{'#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#'}};

static char maze_2[21][21] = {
		{'#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#'},
		{'#','O','#',' ',' ',' ',' ',' ',' ',' ',' ',' ','#',' ',' ',' ',' ',' ',' ',' ','#'},
		{'#',' ','#','#','#',' ','#','#','#','#','#',' ','#',' ','#','#','#','#','#',' ','#'},
		{'#',' ',' ',' ','#',' ','#',' ',' ',' ','#',' ','#',' ','#',' ',' ',' ','#',' ','#'},
		{'#','#','#',' ','#',' ','#',' ','#',' ','#',' ','#',' ','#',' ','#','#','#',' ','#'},
		{'#',' ','#',' ','#',' ','#',' ','#',' ','#',' ',' ',' ','#',' ',' ',' ',' ',' ','#'},
		{'#',' ','#',' ','#',' ','#',' ','#','#','#','#','#','#','#',' ','#','#','#','#','#'},
		{'#',' ',' ',' ','#',' ','#',' ','#',' ',' ',' ',' ',' ',' ',' ','#',' ',' ',' ','#'},
		{'#',' ','#','#','#',' ','#',' ','#',' ','#','#','#','#','#','#','#',' ','#',' ','#'},
		{'#',' ',' ',' ','#',' ','#',' ','#',' ',' ',' ','#',' ','#',' ',' ',' ','#',' ','#'},
		{'#','#','#',' ','#',' ','#',' ','#','#','#',' ','#',' ','#',' ','#',' ','#',' ','#'},
		{'#',' ','#',' ',' ',' ','#',' ',' ',' ',' ',' ','#',' ',' ',' ','#',' ','#',' ','#'},
		{'#',' ','#','#','#','#','#',' ','#','#','#','#','#',' ','#','#','#',' ','#','#','#'},
		{'#',' ',' ',' ',' ',' ','#',' ','#',' ',' ',' ','#',' ','#',' ','#',' ',' ',' ','#'},
		{'#',' ','#','#','#',' ','#',' ','#',' ','#',' ','#',' ','#',' ','#','#','#',' ','#'},
		{'#',' ','#',' ',' ',' ','#',' ',' ',' ','#',' ','#',' ',' ',' ','#',' ',' ',' ','#'},
		{'#',' ','#',' ','#',' ','#','#','#','#','#',' ','#',' ','#','#','#',' ','#',' ','#'},
		{'#',' ','#',' ','#',' ',' ',' ',' ',' ','#',' ','#',' ','#',' ',' ',' ','#',' ','#'},
		{'#',' ','#',' ','#','#','#','#','#','#','#',' ','#','#','#',' ','#','#','#',' ','#'},
		{'#',' ','#',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ','#',' ',' ',' ','X'},
		{'#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#'}};

static char maze_3[21][21] = {
		{'#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#'},
		{'#','O',' ',' ','#',' ','#',' ',' ',' ',' ',' ','#',' ',' ',' ',' ',' ',' ',' ','#'},
		{'#','#','#',' ','#',' ','#',' ','#','#','#',' ','#',' ','#','#','#','#','#',' ','#'},
		{'#',' ',' ',' ','#',' ',' ',' ','#',' ',' ',' ','#',' ',' ',' ','#',' ','#',' ','#'},
		{'#',' ','#','#','#',' ','#','#','#',' ','#','#','#','#','#',' ','#',' ','#',' ','#'},
		{'#',' ',' ',' ','#',' ','#',' ',' ',' ','#',' ',' ',' ',' ',' ',' ',' ','#',' ','#'},
		{'#','#','#',' ','#','#','#',' ','#','#','#',' ','#','#','#','#','#','#','#',' ','#'},
		{'#',' ','#',' ',' ',' ','#',' ','#',' ',' ',' ','#',' ',' ',' ',' ',' ','#',' ','#'},
		{'#',' ','#','#','#',' ','#',' ','#',' ','#','#','#',' ','#','#','#',' ','#',' ','#'},
		{'#',' ',' ',' ','#',' ','#',' ',' ',' ','#',' ',' ',' ','#',' ','#',' ','#',' ','#'},
		{'#',' ','#',' ','#',' ','#','#','#','#','#',' ','#','#','#',' ','#',' ','#',' ','#'},
		{'#',' ','#',' ','#',' ',' ',' ','#',' ',' ',' ','#',' ',' ',' ','#',' ','#',' ','#'},
		{'#',' ','#','#','#','#','#',' ','#',' ','#','#','#',' ','#',' ','#',' ','#',' ','#'},
		{'#',' ','#',' ',' ',' ',' ',' ','#',' ',' ',' ','#',' ','#',' ','#',' ','#',' ','#'},
		{'#',' ','#',' ','#','#','#','#','#',' ','#',' ','#',' ','#','#','#',' ','#',' ','#'},
		{'#',' ','#',' ',' ',' ',' ',' ','#',' ','#',' ','#',' ','#',' ',' ',' ','#',' ','#'},
		{'#',' ','#','#','#','#','#',' ','#','#','#',' ','#',' ','#',' ','#','#','#',' ','#'},
		{'#',' ',' ',' ',' ',' ','#',' ',' ',' ',' ',' ','#',' ','#',' ',' ',' ',' ',' ','#'},
		{'#',' ','#',' ','#','#','#','#','#','#','#','#','#',' ','#','#','#','#','#',' ','#'},
		{'#',' ','#',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ','X'},
		{'#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#'}};

/* Funções Setup. */
static void prvSetupRCC(void);
static void prvSetupGPIO(void);
static void prvSetupNVIC(void);
static void prvSetupUSART2(void);
static void prvSetupSPI(void);
static void prvSetupCREGSPI(void);

/* Funções Auxiliares. */
static uint8_t prvSendReceiveSPI(uint8_t dataSend);
static uint16_t prvColorConvertRGB(char r, char g, char b);
static void prvSendMessageUSART2(char *message);

/* Funções Jogo. */
static void prvDrawMaze(void);
static void prvStartGame(void);
static void prvStartLevel(void);
static void prvEndGame(void);

/* Tasks Jogo. */
static void prvCheckSwitch(void *pvParameters);
static void prvSwitchActions(void *pvParameters);
static void prvGetCoordsACC(void *pvParameters);
static void prvUpdateInfoACC(void *pvParameters);
static void prvUpdateLevelTime(void *pvParameters);
static void prvMovePlayer(void *pvParameters);
static void prvFunnyLEDs(void *pvParameters);

/* Task handle variables. */
TaskHandle_t HandleTaskCheckSwitch;
TaskHandle_t HandleTaskSwitchActions;
TaskHandle_t HandleTaskGetACC;
TaskHandle_t HandleTaskUpdateACC;
TaskHandle_t HandleTaskUpdateTime;
TaskHandle_t HandleTaskMovePlayer;
TaskHandle_t HandleTaskLEDs;

/* Queue handle variable. */
volatile QueueHandle_t xQueueSwitchDebounce;
volatile QueueHandle_t xQueueLevelTimes;

/* Other variables. */

// ACC SPI
int16_t sXH = 0, sXL = 0, sYH = 0, sYL = 0, sZH = 0, sZL = 0;
int16_t sACCx = 0, sACCy = 0, sACCz = 0;

// Flags
typedef struct {
	uint8_t gameFirstStart:1;
	uint8_t gamePaused:1;
	uint8_t gameEnded:1;
	uint8_t switchPressed:1;
} Flags_t;
volatile Flags_t xFlag = {0,0,0,0};

// Contagem de segundos
volatile uint16_t usT1s = 0;

// Player Struct
typedef struct {
	uint8_t x;
	uint8_t y;
	uint8_t previous_x;
	uint8_t previous_y;
	uint8_t level;
} Player_t;
Player_t xP;
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/


/* Main e Tasks Jogo. */
int main(void) {
	/*Setup the hardware, RCC, GPIO, etc... */
    prvSetupRCC();
    lcd_init();
    prvSetupGPIO();
    prvSetupSPI();
    prvSetupUSART2();
    prvSetupNVIC();
    prvSetupCREGSPI();

	/* Configurações Iniciais das Flags. */
	xFlag.gameFirstStart = 1;
	xFlag.gamePaused = 1;
	xFlag.gameEnded = 1;

	/* Menu Inicial */
	lcd_draw_string(26, 30, "MAZE", prvColorConvertRGB(255, 171, 63), 3);
	lcd_draw_string(10, 60, "RUNNER", prvColorConvertRGB(255, 171, 63), 3);
	lcd_draw_string(10, 110, "START:SW5", prvColorConvertRGB(255, 30, 30), 2);

    /* Create the queues. */
    xQueueSwitchDebounce = xQueueCreate(10, sizeof(TickType_t));
    xQueueLevelTimes = xQueueCreate(3, sizeof(uint16_t));

	/* Create the tasks. */
    xTaskCreate(prvCheckSwitch, "CheckSwitch", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &HandleTaskCheckSwitch);
    xTaskCreate(prvSwitchActions, "SwitchActions", configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1, &HandleTaskSwitchActions);
    xTaskCreate(prvGetCoordsACC, "GetCoordsACC", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &HandleTaskGetACC);
    xTaskCreate(prvUpdateInfoACC, "UpdateInfoACC", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &HandleTaskUpdateACC);
    xTaskCreate(prvUpdateLevelTime, "UpdateLevelTime", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &HandleTaskUpdateTime);
    xTaskCreate(prvMovePlayer, "MovePlayer", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &HandleTaskMovePlayer);
    xTaskCreate(prvFunnyLEDs, "FunnyLEDs", configMINIMAL_STACK_SIZE, NULL, mainGAME_CORE_PRIORITY + 1, &HandleTaskLEDs);

 	/* Suspended tasks no Menu Inicial. */
    vTaskSuspend(HandleTaskGetACC);
    vTaskSuspend(HandleTaskUpdateACC);
 	vTaskSuspend(HandleTaskUpdateTime);
 	vTaskSuspend(HandleTaskMovePlayer);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}
/*-----------------------------------------------------------*/

static void prvCheckSwitch(void *pvParameters) {
	TickType_t xLastExecutionTime;
	xLastExecutionTime = xTaskGetTickCount();
	for( ;; ) {
		vTaskDelayUntil( &xLastExecutionTime, (TickType_t) 50 / portTICK_RATE_MS);

		TickType_t xTick;

		if (xQueueSwitchDebounce != 0) {
			xQueueReceive(xQueueSwitchDebounce, &xTick, (TickType_t) portMAX_DELAY);
			vTaskDelay( ( TickType_t ) 50 / portTICK_RATE_MS);
			while (uxQueueMessagesWaitingFromISR(xQueueSwitchDebounce) != 0) xQueueReceive(xQueueSwitchDebounce, &xTick, (TickType_t) portMAX_DELAY);

			xFlag.switchPressed = 1;
		}
	}
}
/*-----------------------------------------------------------*/

static void prvSwitchActions(void *pvParameters) {
	TickType_t xLastExecutionTime;
	xLastExecutionTime = xTaskGetTickCount();
	for( ;; ) {
		vTaskDelayUntil( &xLastExecutionTime, (TickType_t) 100 / portTICK_RATE_MS);

    	// SW5: Inicar, pausar ou retomar o Jogo
		if (xFlag.switchPressed == 1) {
			xFlag.switchPressed = 0;

    		// Pausar ou Retomar
    		if (xFlag.gameEnded == 0) {
    			// Pausar o Jogo => Parar o tempo de nível decorrente e não mexer o player no labirinto
        		if (xFlag.gamePaused == 0) {
        			xFlag.gamePaused = 1;
        			// Escreve "PAUSE" em cima do labirinto
        			lcd_draw_fillrect(18, 52, 93, 28, 0);
        			lcd_draw_string(21, 56, "PAUSE", prvColorConvertRGB(255, 30, 30), 3);

        		// Retomar o Jogo => Atualizar o tempo de nível decorrente e mover o player no labirinto
        		} else {
        			xFlag.gamePaused = 0;
        			// Redesenhar o player e o labirinto
        			for (uint8_t x=17;x<=112;x++) {
        				for (uint8_t y=51;y<=81;y++) {
            				lcd_draw_pixel(x, y, 0);
        				}
        			}
        			prvDrawMaze();
        			lcd_draw_filled_circle(xP.x, xP.y, 2, prvColorConvertRGB(255, 171, 63));
        		}
    		}

			// Iniciar o Jogo (ou re-inicar após vitória)
			if (xFlag.gameEnded == 1) {
				prvStartGame();
			}
    	}
	}
}
/*-----------------------------------------------------------*/

static void prvGetCoordsACC(void *pvParameters) {
	TickType_t xLastExecutionTime;
	xLastExecutionTime = xTaskGetTickCount();
	for( ;; ) {
		vTaskDelayUntil( &xLastExecutionTime, (TickType_t) 50 / portTICK_RATE_MS);
		for (uint8_t i=0; i<6; i++) {
			GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET); // ACC SPI Enable
			if (i==0) sXL = prvSendReceiveSPI(0b10101000);
			if (i==1) sXH = prvSendReceiveSPI(0b10101001);
			if (i==2) sYL = prvSendReceiveSPI(0b10101010);
			if (i==3) sYH = prvSendReceiveSPI(0b10101011);
			if (i==4) sZL = prvSendReceiveSPI(0b10101100);
			if (i==5) sZH = prvSendReceiveSPI(0b10101101);
			GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET); // ACC SPI Disable
		}
		sACCx = (sXH << 8) | sXL;
		sACCy = (sYH << 8) | sYL;
		sACCz = (sZH << 8) | sZL;
	}
}
/*-----------------------------------------------------------*/

static void prvUpdateInfoACC(void *pvParameters) {
	TickType_t xLastExecutionTime;
	xLastExecutionTime = xTaskGetTickCount();
    for( ;; ) {
    	vTaskDelayUntil( &xLastExecutionTime, (TickType_t) 500 / portTICK_RATE_MS);

    	if (xFlag.gameEnded == 0) {
    		// Mostrar os valores do acelerómetro (x, y, z) no LCD e na USART
    		char cBuffer[32];

    		// USART
    	    sprintf(cBuffer, "X:%d Y:%d Z:%d\r\n", sACCx, sACCy, sACCz);
    	    prvSendMessageUSART2(cBuffer);

    		// LCD
    	    char cEraseNumber[5] = "";
    	    char cCoordACC[3] = {'X','Y','Z'};
    	    int16_t sACC[3] = {sACCx, sACCy, sACCz};
    	    for (uint8_t i=0;i<3;i++) {
    	    	if (sACC[i] >= 0) strcpy(cEraseNumber, " ");
    	    	if (abs(sACC[i]) < 1000) strcpy(cEraseNumber, "  ");
    	    	if (abs(sACC[i]) < 100) strcpy(cEraseNumber, "   ");
    	    	if (abs(sACC[i]) < 10) strcpy(cEraseNumber, "    ");
    	    	sprintf(cBuffer, "%c:%d%s", cCoordACC[i], sACC[i], cEraseNumber);
    	    	lcd_draw_string(43*i, 152, cBuffer, prvColorConvertRGB(255, 30, 30), 1);
    	    }
    	}
	}
}
/*-----------------------------------------------------------*/

static void prvUpdateLevelTime(void *pvParameters) {
	TickType_t xLastExecutionTime;
	xLastExecutionTime = xTaskGetTickCount();
	for( ;; ) {
		vTaskDelayUntil( &xLastExecutionTime, (TickType_t) 1000 / portTICK_RATE_MS);

		if (xFlag.gameEnded == 0 && xFlag.gamePaused == 0) {

			// Incrementar o tempo
			usT1s++;

        	// Atualizar o tempo do nível decorrente (LCD)
        	uint8_t ucTimeMin = usT1s / 60;
        	uint8_t ucTimeSeg = usT1s % 60;
        	char cTimeStr[16];
        	if (ucTimeMin < 10) {
        		if (ucTimeSeg >= 10) sprintf(cTimeStr, "TIME 0%d:%d",ucTimeMin, ucTimeSeg);
        		if (ucTimeSeg < 10) sprintf(cTimeStr, "TIME 0%d:0%d", ucTimeMin, ucTimeSeg);
        	} else {
        		if (ucTimeSeg >= 10) sprintf(cTimeStr, "TIME %d:%d", ucTimeMin, ucTimeSeg);
        		if (ucTimeSeg < 10) sprintf(cTimeStr, "TIME %d:0%d", ucTimeMin, ucTimeSeg);
        	}
        	lcd_draw_string(68, 138, cTimeStr, prvColorConvertRGB(255, 30, 30), 1);
    	}
	}
}
/*-----------------------------------------------------------*/

static void prvMovePlayer(void *pvParameters) {
	TickType_t xLastExecutionTime;
	xLastExecutionTime = xTaskGetTickCount();
	for( ;; ) {
		vTaskDelayUntil( &xLastExecutionTime, (TickType_t) 50 / portTICK_RATE_MS);

    	if (xFlag.gameEnded == 0 && xFlag.gamePaused == 0) {
    		// Guardar a posição anterior
    		xP.previous_x = xP.x;
    		xP.previous_y = xP.y;

    		// Nova posição
    		if(sACCx >= 170) xP.x+=1;
    		if(sACCx <= -170) xP.x-=1;
    		if(sACCy >= 170) xP.y+=1;
    		if(sACCy <= -170) xP.y-=1;

    		// Verificar as paredes do labirinto (caso seja uma parede => manter a posição anterior)
    		if (xP.level == 1) {
        		if (maze_1[(xP.x+1)/6][(xP.y+2)/6] == '#' || maze_1[(xP.x+2)/6][(xP.y+1)/6] == '#' ||
        				maze_1[(xP.x+1)/6][(xP.y-2)/6] == '#' || maze_1[(xP.x+2)/6][(xP.y-1)/6] == '#' ||
    					maze_1[(xP.x-1)/6][(xP.y+2)/6] == '#' || maze_1[(xP.x-2)/6][(xP.y+1)/6] == '#' ||
    					maze_1[(xP.x-1)/6][(xP.y-2)/6] == '#' || maze_1[(xP.x-2)/6][(xP.y-1)/6] == '#') {

     			   xP.x = xP.previous_x;
     			   xP.y = xP.previous_y;
        		}
    		}
    		if (xP.level == 2) {
        		if (maze_2[(xP.x+1)/6][(xP.y+2)/6] == '#' || maze_2[(xP.x+2)/6][(xP.y+1)/6] == '#' ||
        				maze_2[(xP.x+1)/6][(xP.y-2)/6] == '#' || maze_2[(xP.x+2)/6][(xP.y-1)/6] == '#' ||
    					maze_2[(xP.x-1)/6][(xP.y+2)/6] == '#' || maze_2[(xP.x-2)/6][(xP.y+1)/6] == '#' ||
    					maze_2[(xP.x-1)/6][(xP.y-2)/6] == '#' || maze_2[(xP.x-2)/6][(xP.y-1)/6] == '#') {

     			   xP.x = xP.previous_x;
     			   xP.y = xP.previous_y;
        		}
    		}
    		if (xP.level == 3) {
        		if (maze_3[(xP.x+1)/6][(xP.y+2)/6] == '#' || maze_3[(xP.x+2)/6][(xP.y+1)/6] == '#' ||
        				maze_3[(xP.x+1)/6][(xP.y-2)/6] == '#' || maze_3[(xP.x+2)/6][(xP.y-1)/6] == '#' ||
    					maze_3[(xP.x-1)/6][(xP.y+2)/6] == '#' || maze_3[(xP.x-2)/6][(xP.y+1)/6] == '#' ||
    					maze_3[(xP.x-1)/6][(xP.y-2)/6] == '#' || maze_3[(xP.x-2)/6][(xP.y-1)/6] == '#') {

     			   xP.x = xP.previous_x;
     			   xP.y = xP.previous_y;
        		}
    		}

    		// Desenhar o player apenas se a posição nova for diferente da anterior
    		if (xP.x != xP.previous_x || xP.y != xP.previous_y) {
        		// Desenhar o player na nova posição (bola amarela)
        		lcd_draw_filled_circle(xP.previous_x, xP.previous_y, 2, 0);
        		lcd_draw_filled_circle(xP.x, xP.y, 2, prvColorConvertRGB(255, 171, 63));

        		// Verificar se é o final do labirinto e começar o nível seguinte (ou terminar o jogo)
        		if ((xP.x == 116 || xP.x == 117) && xP.y == 122) {
        			xP.level++;
        			prvStartLevel();
        		}
    		}
    	}
	}
}
/*-----------------------------------------------------------*/

static void prvFunnyLEDs(void *pvParameters) {
	TickType_t xLastExecutionTime;
	xLastExecutionTime = xTaskGetTickCount();
    for( ;; ) {
    	vTaskDelayUntil( &xLastExecutionTime, (TickType_t) 500 / portTICK_RATE_MS);

    	// LEDs aleatórios no Ecrã Inicial
    	if (xFlag.gameFirstStart == 1) {
    		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0) {
    			GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET);
    			GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_SET);
    		} else GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET);
    		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0) {
    			GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_SET);
    			GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET);
    		} else GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_RESET);
    		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2) == 0) {
    			GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_SET);
    			GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_SET);
    		} else GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_RESET);
    	}

    	// LEDs da Vitória!
        if (xFlag.gameEnded == 1 && xFlag.gameFirstStart == 0) {
        	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0) {
        		GPIO_WriteBit(GPIOB, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2, Bit_SET);
        	} else GPIO_WriteBit(GPIOB, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2, Bit_RESET);
    	}
	}
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/


/* Funções Jogo. */
static void prvDrawMaze() {
	// Display do Labirinto
	for(int n = 0; n<21; n++) {
		for(int k = 0; k<21; k++) {
			// Impressão no LCD (com scale x6.0)
			if (xP.level == 1) {
				if (maze_1[n][k] == '#') {
					for(int j = 0; j<6; j++) {
						for(int p = 0; p<6; p++) {
							lcd_draw_pixel((n*6)+p, (k*6)+j,prvColorConvertRGB(255, 117, 63));
						}
	            	}
	            }
			}
			if (xP.level == 2) {
				if (maze_2[n][k] == '#') {
					for(int j = 0; j<6; j++) {
						for(int p = 0; p<6; p++) {
							lcd_draw_pixel((n*6)+p, (k*6)+j,prvColorConvertRGB(255, 117, 63));
						}
	            	}
	            }
			}
			if (xP.level == 3) {
				if (maze_3[n][k] == '#') {
					for(int j = 0; j<6; j++) {
						for(int p = 0; p<6; p++) {
							lcd_draw_pixel((n*6)+p, (k*6)+j,prvColorConvertRGB(255, 117, 63));
						}
	            	}
	            }
			}
        }
	}

	// Desenho de uma meta no final do labirinto
	for (int j=0;j<6;j++) {
		for (int p=0;p<6;p++) {
			if (j == 3 || j == 5) {
				if (p == 1 || p == 3 || p == 5) {
					lcd_draw_pixel(114+p, 120+j, 0);
				} else lcd_draw_pixel(114+p, 120+j, prvColorConvertRGB(200, 200, 200));
			} else {
				if (j == 4) {
					if (p == 0 || p == 2 || p == 4) {
						lcd_draw_pixel(114+p, 120+j, 0);
					} else lcd_draw_pixel(114+p, 120+j, prvColorConvertRGB(200, 200, 200));
				} else lcd_draw_pixel(114+p, 120+j, 0);
			}
		}
	}
}
/*-----------------------------------------------------------*/

static void prvStartGame(void) {
	xFlag.gameFirstStart = 0;
	xFlag.gameEnded = 0;

	// Reset LEDs e limpar LCD
	GPIO_WriteBit(GPIOB, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2, Bit_RESET);
	lcd_draw_fillrect(0, 0, 128, 160, 0);

	// Countdown e LEDs a piscar (com a task FunnyLEDs)
	lcd_draw_string(56, 60, "3", prvColorConvertRGB(255, 171, 63), 3);
	vTaskDelay( (TickType_t ) 1000 / portTICK_RATE_MS);

	lcd_draw_fillrect(0, 0, 128, 90, 0);
	lcd_draw_string(56, 60, "2", prvColorConvertRGB(255, 171, 63), 3);
	vTaskDelay( (TickType_t ) 1000 / portTICK_RATE_MS);

	lcd_draw_fillrect(0, 0, 128, 90, 0);
	lcd_draw_string(56, 60, "1", prvColorConvertRGB(255, 171, 63), 3);
	vTaskDelay( (TickType_t ) 1000 / portTICK_RATE_MS);

	lcd_draw_fillrect(0, 0, 128, 90, 0);
	lcd_draw_string(45, 60, "GO!", prvColorConvertRGB(255, 30, 30), 3);
	vTaskDelay( (TickType_t ) 1000 / portTICK_RATE_MS);

	// Linha separadora, resume ACC Tasks e começar o nivel
	lcd_draw_fillrect(0, 148, 128, 1, prvColorConvertRGB(255, 30, 30));
    vTaskResume(HandleTaskGetACC);
    vTaskResume(HandleTaskUpdateACC);
	xP.level=1;
	prvStartLevel();
}
/*-----------------------------------------------------------*/

static void prvStartLevel(void) {
	xFlag.gamePaused = 1;
	// Nível 4 => Fim do Jogo (guarda o tempo e não precisa de executar o resto)
	if (xP.level == 4) {
		xQueueSendToBack(xQueueLevelTimes, &usT1s, (TickType_t) 10);
		prvEndGame();

	} else {

		// Limpar LCD e desenhar novo labirinto
		lcd_draw_fillrect(0, 0, 128, 146, 0);
		prvDrawMaze();

		// Nivel 1, 2 e 3 - Guardar Tempo e mostrar info nos LEDs
		if (xP.level == 1) GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET);
		if (xP.level == 2) {
			xQueueSendToBack(xQueueLevelTimes, &usT1s, (TickType_t) 10);
			GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_SET);
		}
		if (xP.level == 3) {
			xQueueSendToBack(xQueueLevelTimes, &usT1s, (TickType_t) 10);
			GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_SET);
		}
		// Mostrar info no LCD
		char buffer[32];
		sprintf(buffer, "LEVEL %d", xP.level);
		lcd_draw_string(68, 138, "TIME 00:00", prvColorConvertRGB(255, 30, 30), 1);
		lcd_draw_string(1, 138, buffer, prvColorConvertRGB(255, 30, 30), 1);

		// Retomar a posição inicial, reset no tempo e arrancar tasks (caso suspensas)
		xP.x = 9; xP.y = 9;
		lcd_draw_filled_circle(xP.x, xP.y, 2, prvColorConvertRGB(255, 171, 63));
		usT1s = 0;

		xFlag.gamePaused = 0;
	 	vTaskResume(HandleTaskUpdateTime);
	 	vTaskResume(HandleTaskMovePlayer);
	}
}
/*-----------------------------------------------------------*/

static void prvEndGame(void) {
	xFlag.gameEnded = 1;
	vTaskSuspend(HandleTaskUpdateACC);

	// Limpar LCD
	lcd_draw_fillrect(0, 0, 128, 170, 0);

	// Menu Final
	lcd_draw_string(7, 11, "GOOD GAME!", prvColorConvertRGB(255, 171, 63), 2);
	lcd_draw_string(34, 40, "SCORE", prvColorConvertRGB(255, 117, 63), 2);
	lcd_draw_string(10, 135, "AGAIN:SW5", prvColorConvertRGB(255, 30, 30), 2);

	// Mostrar os tempos de todos os niveis e o total acumulado
	uint16_t usTotalTime;
	uint16_t usLevelTime;
	uint8_t ucTimeMin;
	uint8_t ucTimeSeg;
	char cTimeStr[32];
	for (uint8_t i=0;i<3;i++) {
		xQueueReceive(xQueueLevelTimes, &usLevelTime, (TickType_t) portMAX_DELAY);
		usTotalTime += usLevelTime;
		ucTimeMin = usLevelTime / 60;
		ucTimeSeg = usLevelTime % 60;
		if (ucTimeMin < 10) {
			if (ucTimeSeg >= 10) sprintf(cTimeStr, "LEVEL %d  0%d:%d", i+1, ucTimeMin, ucTimeSeg);
			if (ucTimeSeg < 10) sprintf(cTimeStr, "LEVEL %d  0%d:0%d", i+1, ucTimeMin, ucTimeSeg);
		} else {
			if (ucTimeSeg >= 10) sprintf(cTimeStr, "LEVEL %d  %d:%d", i+1, ucTimeMin, ucTimeSeg);
			if (ucTimeSeg < 10) sprintf(cTimeStr, "LEVEL %d  %d:0%d", i+1, ucTimeMin, ucTimeSeg);
		}
		lcd_draw_string(21, 65+i*15, cTimeStr, prvColorConvertRGB(255, 117, 63), 1);
	}
	ucTimeMin = usTotalTime / 60;
	ucTimeSeg = usTotalTime % 60;
	if (ucTimeMin < 10) {
		if (ucTimeSeg >= 10) sprintf(cTimeStr, "TOTAL  0%d:%d", ucTimeMin, ucTimeSeg);
		if (ucTimeSeg < 10) sprintf(cTimeStr, "TOTAL  0%d:0%d", ucTimeMin, ucTimeSeg);
	} else {
		if (ucTimeSeg >= 10) sprintf(cTimeStr, "TOTAL  %d:%d", ucTimeMin, ucTimeSeg);
		if (ucTimeSeg < 10) sprintf(cTimeStr, "TOTAL  %d:0%d", ucTimeMin, ucTimeSeg);
	}
	lcd_draw_string(33, 110, cTimeStr, prvColorConvertRGB(255, 117, 63), 1);
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/


/* Funções Setup. */
static void prvSetupRCC(void) {
	// RCC - HSE PLL 72 MHz
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	ErrorStatus HSEStartUpStatus;
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if (HSEStartUpStatus == SUCCESS) {
		FLASH_SetLatency(FLASH_Latency_2);
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		RCC_PCLK1Config(RCC_HCLK_Div2); // PCLK1 Max 36MHz
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6);
		RCC_PLLCmd(ENABLE);
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	} else while(1);
	while (RCC_GetSYSCLKSource() != 0x08);
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
}
/*-----------------------------------------------------------*/

static void prvSetupGPIO(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	// SWITCHES: PA1 - SW5
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	// LEDS: PB0 PB1 PB2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// USART2: PA2 - TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// SPI: PB5 - RDY / PB13 - SCL (SPI2 SCK) / PB15 - SDA (SPI2 MOSI)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// SPI: PB14 - SDO (SPI2 MISO)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// SPI: PD2 - CS/SS
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}
/*-----------------------------------------------------------*/

static void prvSetupNVIC(void) {
    // Configuração das Prioridades Group 1
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // EXTI1
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    // Configuração da interrupção EXTI1
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}
/*-----------------------------------------------------------*/

static void prvSetupUSART2(void) {
	// USART 2 - 115200 bps
	USART_InitTypeDef USART_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
    USART_Cmd(USART2, ENABLE);
}
/*-----------------------------------------------------------*/

static void prvSetupSPI(void) {
	// SPI2 - Full Duplex, Placa Master, Read High
	SPI_InitTypeDef SPI_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);

	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET); // ACC SPI Disable
}
/*-----------------------------------------------------------*/

static void prvSetupCREGSPI(void) {
	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET); // ACC SPI Enable
	prvSendReceiveSPI(0x20); // CTRL_REG1
	prvSendReceiveSPI(0b11000111); 	// Sensor ON, Rate 40Hz, Self Test OFF, Eixos X Y Z ON
	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET); // ACC SPI Disable
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/


/* Funções Auxiliares. */
static uint8_t prvSendReceiveSPI(uint8_t dataSend) {
	uint8_t dataReceive = 0;
	for(uint8_t i=0;i<2;i++) {
		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData(SPI2, dataSend);
		SPI_I2S_ClearFlag(SPI2, SPI_I2S_FLAG_TXE);

		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
		dataReceive = SPI_I2S_ReceiveData(SPI2);
		SPI_I2S_ClearFlag(SPI2, SPI_I2S_FLAG_RXNE);
	}
	return dataReceive;
}
/*-----------------------------------------------------------*/

static uint16_t prvColorConvertRGB(char r, char g, char b) {
	// Conversão RGB 24 bits para RGB 16 bits (R: 5 bits / G: 6 bits / B: 5 bits)
	r = (r*31)/255;
    g = (g*63)/255;
    b = (b*31)/255;
    return ((b) | (g << 5) | (r << 11));
}
/*-----------------------------------------------------------*/

static void prvSendMessageUSART2(char *message) {
	uint16_t aux=0;
    while(aux != strlen(message)) {
        USART_SendData(USART2, (uint8_t) message[aux]);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        aux++;
    }
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/
/* EoF
/*-----------------------------------------------------------*/
