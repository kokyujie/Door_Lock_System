/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "liquidcrystal_i2c.h"
#include "string.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern I2C_HandleTypeDef hi2c1;  // lcd
extern TIM_HandleTypeDef htim3;  // timer

char input[PASSWORD_LENGTH + 1] = {'0','0','0','0','0','0','\0'};  // Input for Access
char password[PASSWORD_LENGTH + 1] = {'1','1','1','1','1','1','\0'};  // Default password
char clear[PASSWORD_LENGTH + 1] = {'0','0','0','0','0','0','\0'};   // Clear input
char temp[PASSWORD_LENGTH + 1] = {'0','0','0','0','0','0','\0'};  // Temporary password

int clearState;  // Make sure reset is pressed to clear screen
int enterState;  // Make sure enter is pressed to enter next stage
uint8_t buttonpressedFlag = 0;  // Check for input
volatile uint32_t doorOpenCounter = 0;
volatile uint8_t doorOpenFlag = 0;  // Check for door status

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t lastDebounceTime;
    GPIO_PinState lastState;
    GPIO_PinState stableState;
    uint32_t pressStartTime;
} Button_t;

Button_t buttons[NUM_INPUT] = {
    {BUTTON_PORT, GPIO_PIN_0, 0, GPIO_PIN_RESET, GPIO_PIN_RESET},
    {BUTTON_PORT, GPIO_PIN_1, 0, GPIO_PIN_RESET, GPIO_PIN_RESET},
	{BUTTON_PORT, GPIO_PIN_2, 0, GPIO_PIN_RESET, GPIO_PIN_RESET},
	{BUTTON_PORT, GPIO_PIN_3, 0, GPIO_PIN_RESET, GPIO_PIN_RESET},
	{BUTTON_PORT, GPIO_PIN_4, 0, GPIO_PIN_RESET, GPIO_PIN_RESET},
	{BUTTON_PORT, GPIO_PIN_5, 0, GPIO_PIN_RESET, GPIO_PIN_RESET},
	{BUTTON_PORT, GPIO_PIN_6, 0, GPIO_PIN_RESET, GPIO_PIN_RESET},
	{BUTTON_PORT, GPIO_PIN_7, 0, GPIO_PIN_RESET, GPIO_PIN_RESET},
	{BUTTON_PORT, GPIO_PIN_8, 0, GPIO_PIN_RESET, GPIO_PIN_RESET},
	{BUTTON_PORT, GPIO_PIN_9, 0, GPIO_PIN_RESET, GPIO_PIN_RESET},
    {BUTTON_PORT, RESET_BUTTON_PIN, 0, GPIO_PIN_RESET, GPIO_PIN_RESET, 0},
	{BUTTON_PORT, ENTER_BUTTON_PIN, 0, GPIO_PIN_RESET, GPIO_PIN_RESET},
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_TIM3_Init(void);
int enter(int inputindex);
void unlockDoor(void);
void alertWrongPassword(void);
void alertDoorOpen(void);
void checkButtons(void);
void clearScreen(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HD44780_Init(2);
  HD44780_Clear();
  HD44780_PrintStr("Enter Password");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t currenttime = HAL_GetTick();
	  GPIO_PinState buttonState = HAL_GPIO_ReadPin(buttons[10].port, buttons[10].pin);

	  if(buttonState == GPIO_PIN_SET && buttons[10].pressStartTime == 0){
		  buttons[10].pressStartTime = currenttime;
	  }

	  if (buttonState == GPIO_PIN_SET && ((currenttime - buttons[10].pressStartTime) >= 2000)) {    // Check whether reset button is long pressed to set a new password
		  HD44780_Clear();
		  HD44780_PrintStr("Enter New ");
		  HD44780_SetCursor(0,1);
		  HD44780_PrintStr("Password ");
		  for (int j = 0; j <= PASSWORD_LENGTH;){
			  int num1 = enter(j);
			  if(clearState == 1){  // When reset button is press, all the temporary password is cleared
				  strcpy(temp, clear);
				  clearState = 0;
				  break;
			  }
			  else if((enterState == 1) && (j == PASSWORD_LENGTH)){  // When 6 digit password is entered and enter is pressed, temporary password is stored
				  strcpy(password, temp);
				  enterState = 0;
				  HD44780_Clear();
				  HD44780_PrintStr("Enter Password");
				  break;
			  }
			  else if(j < PASSWORD_LENGTH){  //  pressed number is store in temporary password variable
				  temp[j] = '0' + num1;
				  j++;
			  }
		  }
		  buttons[10].pressStartTime = 0;
	  }

	  if (buttonState == GPIO_PIN_RESET){
		  for (int h = 0; h < 10; h++){
			  if (HAL_GPIO_ReadPin(buttons[h].port, buttons[h].pin) == GPIO_PIN_SET){  // Check whether button is pressed
				  for (int k = 0; k <= PASSWORD_LENGTH;) {  // Check input digit one by one
					  int num2 = enter(k);
					  if(clearState == 1){  // When reset button is press, all the input is cleared
						  strcpy(input, clear);
						  k = 0;
						  clearState = 0;
						  break;
					  }
					  else if((enterState == 1) && (k == PASSWORD_LENGTH))	{   // When 6 digit password is entered and enter is pressed, input is compared with password
						  if (strcmp(input,password) == 0) {  // if input correct password, door is unlocked
							  unlockDoor();
						  }
						  else {
							  alertWrongPassword();  // give alert for wrong password
						  }
						  enterState = 0;
						  break;
					  }
					  else if(k < PASSWORD_LENGTH){  // pressed number is store in input variable
						  input[k] = '0' + num2;
						  k++;
					  }
				  }
			  }
		  }
		  buttons[10].pressStartTime = 0;
	  }
  }
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
int enter(int inputindex){
	buttonpressedFlag = 0;
	while(buttonpressedFlag == 0){
		uint32_t currentTime = HAL_GetTick();
		for (int i = 0; i < NUM_INPUT; i++) {
			GPIO_PinState currentState = HAL_GPIO_ReadPin(buttons[i].port, buttons[i].pin);   //Check for button pressed using debounce code

			// Check if state has changed from last stable state
			if (currentState != buttons[i].lastState) {
				buttons[i].lastDebounceTime = currentTime;  // Reset debounce timer
			}

			if ((currentTime - buttons[i].lastDebounceTime) >= 50) {   //Make sure last debounce time over 50ms to ensure stable state
				if (currentState != buttons[i].stableState) {
					buttons[i].stableState = currentState;
					if (currentState == GPIO_PIN_SET) {
						if ((i < 10) && (inputindex == 0)){  // before first input, the screen is cleared
							HD44780_Clear();
						}
						if ((i < 10) && (inputindex < PASSWORD_LENGTH)){   // * is print for every input for safety
							HD44780_PrintStr("*");
							buttonpressedFlag = 1;
							return i;
						}
						else if ((i == 10) && (inputindex > 0)){   // Reset button is press to clear all the input
							HD44780_Clear();
							HD44780_PrintStr("Enter Password");
							buttonpressedFlag = 1;
							clearState = 1;
						}
						else if ((i == 11) && (inputindex == PASSWORD_LENGTH)){  // Ensure inputIndex does not exceed buffer size
							buttonpressedFlag = 1;
							enterState = 1;
						}
					}
				}
			}
			buttons[i].lastState = currentState;  // Update last known state
		}
	}
}


void unlockDoor(void) {
    HAL_GPIO_WritePin(GPIOB, RELAY_PIN, GPIO_PIN_SET);  // Activate relay to open the door lock
    HD44780_Clear();
    HD44780_PrintStr("Door Unlocked");
    alertDoorOpen();
    HAL_GPIO_WritePin(GPIOB, RELAY_PIN, GPIO_PIN_RESET);  // Deactivate relay to lock the door
    HD44780_Clear();
    HD44780_PrintStr("Enter Password");
}

void alertWrongPassword(void) {
    HAL_GPIO_WritePin(OUTPUT_PORT, BUZZER_PIN, GPIO_PIN_SET);  // Activate buzzer
    HD44780_Clear();
    HD44780_PrintStr("Wrong Password");
    HAL_GPIO_WritePin(OUTPUT_PORT, BUZZER_PIN, GPIO_PIN_RESET);  // Deactivate buzzer
    HAL_Delay(1000);
    HD44780_Clear();
    HD44780_PrintStr("Enter Password");
}

void alertDoorOpen(void){
	HAL_Delay(10000);  // delay to ensure door can be open before it locks back
	doorOpenFlag = 0;
	while (HAL_GPIO_ReadPin(BUTTON_PORT, DOOR_SENSOR_PIN) == GPIO_PIN_RESET) {
		if (!doorOpenFlag) {
			doorOpenFlag = 1;
            doorOpenCounter = 0;  // Reset the timer counter when door opens
            HAL_TIM_Base_Start_IT(&htim3);  // Start timer interrupt
        }
    }
	if (HAL_GPIO_ReadPin(BUTTON_PORT, DOOR_SENSOR_PIN) == GPIO_PIN_SET){
		HAL_GPIO_WritePin(OUTPUT_PORT, BUZZER_PIN, GPIO_PIN_RESET);  // Turn off buzzer
		HAL_TIM_Base_Stop_IT(&htim3);  // Stop timer interrupt when door closes
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM3) {  // Check if TIM2 interrupt
		while (doorOpenFlag) {
			doorOpenCounter++;
			if (doorOpenCounter > 30000000) {
				HAL_GPIO_WritePin(OUTPUT_PORT, BUZZER_PIN, GPIO_PIN_SET); // Turn on buzzer
				break;
			}
			else if (HAL_GPIO_ReadPin(BUTTON_PORT, DOOR_SENSOR_PIN) == GPIO_PIN_SET){
				HAL_TIM_Base_Stop_IT(&htim3);  // Stop timer interrupt when door closes
				break;
			}
		}
	}
}

  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
