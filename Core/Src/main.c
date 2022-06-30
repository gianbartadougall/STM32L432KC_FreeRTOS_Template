/**
 * @file main.c
 * @author Gian Barta-Dougall
 * @brief Program File
 * @version 0.1
 * @date 2022-04-15
 *
 * @copyright Copyright (c) 2022
 *
 */

/* Public includes */
#include "FreeRTOS.h"
#include "task.h"

/* Private includes */
#include "main.h"

/* Private #defines */
#define LED_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 32)
#define LED_TASK_PRIORITY (tskIDLE_PRIORITY + 2)

/* Function prototypes */
void led_task(void);
void error_handler(void);
void led_init(void);
void SystemClock_Config(void);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    // Reset all peripherals, initialise the flash interface and the systick
    HAL_Init();

	led_init();

    // Start tasks for program
    if (xTaskCreate((void *)&led_task, (const char *)"LED", LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, NULL) != pdPASS) {
		error_handler();
	}

	HAL_GPIO_WritePin(LD3_PORT, LD3_PIN, 1);
	HAL_Delay(1000);
    // Start the scheduler
    vTaskStartScheduler();
	
    return 0;
}

/**
 * @brief This function has not been implemented but is called when there is a stack overflow
 * 
 * @param xTask 
 * @param pcTaskName 
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
	error_handler();
}


void led_task(void) {

	HAL_GPIO_WritePin(LD3_PORT, LD3_PIN, 0);

	for (;;);

    // Disable interrupts
    // portDISABLE_INTERRUPTS();

    // Enable interrupts
    // portENABLE_INTERRUPTS();
	const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
    for (;;) {

        HAL_GPIO_TogglePin(LD3_PORT, LD3_PIN);

        vTaskDelay(xDelay);
    }
}

/**
 * @brief Initialises the onboard LED (LD3)
 *
 */
void led_init(void) {
    GPIO_InitTypeDef led;

    // Configure the LED
    led.Pin = LD3_PIN;
    led.Mode = GPIO_MODE_OUTPUT_PP;
    led.Pull = GPIO_PULLUP;
    led.Speed = GPIO_SPEED_FREQ_HIGH;

    // Enable clock for LED
    LD3_CLK_ENABLE();

    // Initialise the LED
    HAL_GPIO_Init(LD3_PORT, &led);
}

/**
 * @brief Handles initialisation errors
 *
 */
void error_handler(void) {

    // Initialisation error shown by blinking LED (LD3) in pattern
    while (1) {

        for (int i = 0; i < 5; i++) {
            HAL_GPIO_TogglePin(LD3_PORT, LD3_PIN);
            HAL_Delay(100);
        }

        HAL_Delay(500);
    }
}