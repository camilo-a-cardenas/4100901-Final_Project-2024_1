/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "ssd1306.h"
#include "ssd1306_fonts.h"

#include "ring_buffer.h"
#include "keypad.h"


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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

#define USART2_BUFFER_SIZE 8
uint8_t usart2_buffer[USART2_BUFFER_SIZE];
ring_buffer_t usart2_rb;
uint8_t usart2_rx;
uint8_t IZQ;
uint8_t DER;





uint32_t left_toggles = 0;
uint32_t right_toggles = 0;
uint32_t left_last_press_tick = 0;
uint32_t right_last_press_tick = 0;



typedef enum {
    MENU_PRINCIPAL,
    MENU_DIRECCION,
    SELECCION_IZQUIERDA,
    SELECCION_DERECHA
} MenuState;

MenuState currentMenu = MENU_PRINCIPAL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* Private variables ---------------------------------------------------------*/
#define MAX_DIGITS 4  // Longitud de la clave
char current_password[MAX_DIGITS + 1] = "";  // Buffer para almacenar la clave actual
uint8_t digit_index = 0; // �?ndice para los dígitos ingresados
char correct_password[MAX_DIGITS + 1] = "1234";  // Clave correcta para comparación
uint8_t authenticated = 0;  // Estado de autenticación (0 = no autenticado, 1 = autenticado)

char clave_actual[5] = "1234"; // Clave por defecto
char clave_ingresada[5]; // Para almacenar la clave ingresada
int8_t v=0;

/* Función para actualizar la pantalla con la clave ingresada */
void update_password_display() {
    ssd1306_Fill(Black);  // Limpiar la pantalla
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("Clave: ", Font_11x18, White);

    // Mostrar los dígitos ingresados
    ssd1306_SetCursor(0, 20);
    ssd1306_WriteString(current_password, Font_11x18, White);

    ssd1306_UpdateScreen();  // Actualizar la pantalla
}

void process_keypad_input(uint8_t key) {
    // Si el usuario ya está autenticado, pasa a los comandos
    if (authenticated) {
          // Aquí se abre el menú de comandos
        uint8_t key_pressed = keypad_scan(key);


    }

    // Si el usuario no está autenticado, sigue pidiendo la clave
    if (key >= '0' && key <= '9' && !authenticated) {
        if (digit_index < MAX_DIGITS) {
            current_password[digit_index++] = key;  // Añadir el dígito al buffer
            current_password[digit_index] = '\0';  // Asegurar que el string termine en '\0'
            update_password_display();  // Actualizar la pantalla
        }

        if (digit_index == MAX_DIGITS) {
            verify_password();  // Verificar la clave
        }
    }
}





void comandos(uint8_t key2) {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("1)IZQ 2)STOP", Font_11x18, White);
    ssd1306_SetCursor(0, 20);
    ssd1306_WriteString("3) DER", Font_11x18, White);
    ssd1306_SetCursor(0, 40);
    ssd1306_WriteString("4)C.CLAVE", Font_11x18, White);
    ssd1306_UpdateScreen();
    HAL_UART_Transmit(&huart2,(uint8_t *)"MENU\r\n1)IZQ 2)STOP\r\n3) DER\r\n4)C.CLAVE\r\n",47 , 10);
    HAL_UART_Transmit(&huart3,(uint8_t *)"MENU\r\n1)IZQ 2)STOP\r\n3) DER\r\n4)C.CLAVE\r\n",47 , 10);



    // Debugging: Ver el valor escaneado
    printf("Tecla presionada: %c\r\n", key2);  // Imprime el valor escaneado para verificar

    // Verificar las entradas para los comandos
    if (key2 == '1') {
        signal_direction(1);  // Izquierda
    } else if (key2 == '2') {
        signal_stop();  // Stop
    } else if (key2 == '3') {
        signal_direction(3);  // Derecha
    } else if (key2 == '4') {
        cambiar_clave();  // Cambiar clave
    } else {
        // Si ninguna tecla es válida, mostrar un error o continuar

        ssd1306_WriteString("Comando no reconocido", Font_11x18, White);
		HAL_UART_Transmit(&huart2,(uint8_t *)"Comando no reconocido\r\n",24 , 10);
		HAL_UART_Transmit(&huart3, (uint8_t *)"Comando no reconocido\r\n", 24, 10);
    }
}






/* Función para verificar si la clave ingresada es correcta */

void verify_password() {
    if (strcmp(current_password, correct_password) == 0) {
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("Clave Correcta!", Font_11x18, White);
        HAL_UART_Transmit(&huart2, (uint8_t *)"Clave Correcta!\r\n",18 , 10);
        HAL_UART_Transmit(&huart3, (uint8_t *)"Clave Correcta!\r\n", 18, 10);
        ssd1306_UpdateScreen();
        authenticated = 1;  // El usuario ahora está autenticado
    } else {
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("Clave Incorrecta", Font_11x18, White);
        ssd1306_UpdateScreen();
        HAL_UART_Transmit(&huart2,(uint8_t *)"Clave Incorrecta\r\n",24 , 10);
        HAL_UART_Transmit(&huart3, (uint8_t *)"Clave Incorrecta\r\n", 24, 10);
    }

    // Reiniciar la clave ingresada
    digit_index = 0;
    current_password[0] = '\0';  // Limpiar el buffer
}













void cambiar_clave() {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("Nueva Clave:", Font_11x18, White);
    HAL_UART_Transmit(&huart2,(uint8_t *)"Nueva clave\r\n",15 , 10);
    HAL_UART_Transmit(&huart3,(uint8_t *)"Nueva clave\r\n",15 , 10);
    ssd1306_UpdateScreen();

    // Capturar la nueva clave
    for (int i = 0; i < 4; i++) {
        clave_actual[i] = keypad_scan(GPIO_PIN_All);  // Capturar cada número
        HAL_Delay(500);  // Simular el tiempo entre teclas
    }
    clave_actual[4] = '\0';  // Asegurar que termine con '\0'

    // Mostrar mensaje de éxito
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("Clave Cambiada", Font_11x18, White);
    HAL_UART_Transmit(&huart2,(uint8_t *)"Clave cambiada\r\n",19 , 10);
    HAL_UART_Transmit(&huart3,(uint8_t *)"Clave cambiada\r\n",19 , 10);
    ssd1306_UpdateScreen();
    HAL_Delay(2000);
}


void signal_direction(uint8_t direction) {
	static uint16_t last_pressed = 0xFFFF;
	static uint32_t last_tick = 0;
	ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    last_tick = HAL_GetTick();
    last_pressed = direction;

    if (direction == 1) {

    	        if (HAL_GetTick() < (left_last_press_tick + 300)) {
    	            left_toggles = 0xFFFFFF;  // Presión larga
    	        } else {
    	            left_toggles = 6;  // Presión corta
    	        }
    	left_last_press_tick = HAL_GetTick();
    	ssd1306_WriteString("<< Izquierda <<", Font_11x18, White);
        HAL_UART_Transmit(&huart2,(uint8_t *)"<< Izquierda <<\r\n",19 , 10);
        HAL_UART_Transmit(&huart3,(uint8_t *)"<< Izquierda <<\r\n",19 , 10);
    }  else if (direction == 1 || direction == 2 || direction == 3) {
        left_toggles = 0;
    }






    if (direction == 3) {
        ssd1306_WriteString(">> Derecha >>", Font_11x18, White);
        HAL_UART_Transmit(&huart2,(uint8_t *)">> Derecha >>\r\n",19 , 10);
        HAL_UART_Transmit(&huart3,(uint8_t *)">> Derecha >>\r\n",19 , 10);
    }
    ssd1306_UpdateScreen();
    HAL_Delay(2000);  // Mantener el mensaje por 2 segundos
}


void heartbeat(void)
{
	static uint32_t heartbeat_tick = 0;
	if (heartbeat_tick < HAL_GetTick()) {
		heartbeat_tick = HAL_GetTick() + 500;
		HAL_GPIO_TogglePin(D1_GPIO_Port, D1_Pin);
	}
}

void turn_signal_left(void)
{
	if(authenticated){
	static uint32_t turn_toggle_tick = 0;
	if (turn_toggle_tick < HAL_GetTick()) {
		if (left_toggles > 0) {
			turn_toggle_tick = HAL_GetTick() + 500;

			ssd1306_Fill(Black);
			HAL_Delay(500);
			ssd1306_SetCursor(0, 0);
			ssd1306_WriteString("<< Izquierda <<", Font_11x18, White);
			HAL_UART_Transmit(&huart2,(uint8_t *)"<< Izquierda <<\r\n",19 , 10);
			HAL_UART_Transmit(&huart3,(uint8_t *)"<< Izquierda <<\r\n",19 , 10);
			ssd1306_UpdateScreen();
			left_toggles--;
		} else {
			ssd1306_Fill(Black);
			HAL_Delay(500);
			ssd1306_SetCursor(0, 0);
			ssd1306_WriteString("<< Izquierda <<", Font_11x18, White);
			HAL_UART_Transmit(&huart2,(uint8_t *)"<< Izquierda <<\r\n",19 , 10);
			HAL_UART_Transmit(&huart3,(uint8_t *)"<< Izquierda <<\r\n",19 , 10);
			ssd1306_UpdateScreen();
		}

	}
	}
}



void signal_stop() {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("=== STOP ===", Font_11x18, White);
    HAL_UART_Transmit(&huart2,(uint8_t *)">> Derecha >>\r\n",19 , 10);
    HAL_UART_Transmit(&huart3,(uint8_t *)">> Derecha >>\r\n",19 , 10);
    ssd1306_UpdateScreen();
    HAL_Delay(2000);  // Mantener el mensaje por 2 segundos
}



int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 10);
  return len;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Data received in USART2 */
  if (huart->Instance == USART2) {
	  usart2_rx = USART2->RDR; // leyendo el byte recibido de USART2
	  ring_buffer_write(&usart2_rb, usart2_rx); // put the data received in buffer
	  //HAL_UART_Receive_IT(&huart2, &usart2_rx, 1); // enable interrupt to continue receiving
	  ATOMIC_SET_BIT(USART2->CR1, USART_CR1_RXNEIE); // usando un funcion mas liviana para reducir memoria
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{




    uint8_t key_pressed = keypad_scan(GPIO_Pin);

    if(!authenticated){
    // Llamar a la función para procesar la tecla presionada
    process_keypad_input(key_pressed);
    }else{


    	comandos(key_pressed);
    }

    // Aquí puedes continuar con la lógica del menú si es necesario


	if (GPIO_Pin == BUTTON_RIGHT_Pin) {
		HAL_UART_Transmit(&huart2, (uint8_t *)"S1\r\n", 4, 10);
		if (HAL_GetTick() < (left_last_press_tick + 300)) { // if last press was in the last 300ms
			left_toggles = 0xFFFFFF; // a long time toggling (infinite)
		} else {
			left_toggles = 6;
		}
		left_last_press_tick = HAL_GetTick();
	} else if (GPIO_Pin == BUTTON_LEFT_Pin) {
		left_toggles = 0;
	}

	}


void low_power_mode()
{
#define AWAKE_TIME (10 * 1000) // 10 segundos
	static uint32_t sleep_tick = AWAKE_TIME;

	if (sleep_tick > HAL_GetTick()) {
		return;
	}
	printf("Sleeping\r\n");
	sleep_tick = HAL_GetTick() + AWAKE_TIME;

	RCC->AHB1SMENR  = 0x0;
	RCC->AHB2SMENR  = 0x0;
	RCC->AHB3SMENR  = 0x0;

	RCC->APB1SMENR1 = 0x0;
	RCC->APB1SMENR2 = 0x0;
	RCC->APB2SMENR  = 0x0;

	/*Suspend Tick increment to prevent wakeup by Systick interrupt.
	Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base)*/
	HAL_SuspendTick();

	/* Enter Sleep Mode , wake up is done once User push-button is pressed */
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

	/* Resume Tick interrupt if disabled prior to SLEEP mode entry */
	HAL_ResumeTick();

	printf("Awake\r\n");
}
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  ssd1306_Init();
  ssd1306_SetCursor(25, 5);





  ring_buffer_init(&usart2_rb, usart2_buffer, USART2_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("Starting...\r\n");
  //HAL_UART_Receive_IT(&huart2, &usart2_rx, 1); // enable interrupt for USART2 Rx
  ATOMIC_SET_BIT(USART2->CR1, USART_CR1_RXNEIE); // usando un funcion mas liviana para reducir memoria


  while (1) {






	  while (1) {
	      HAL_Delay(500);

	      if (!authenticated) {
	          ssd1306_Fill(Black);
	          ssd1306_SetCursor(0, 0);
	          ssd1306_WriteString("Ingrese Clave:", Font_11x18, White);
	          ssd1306_UpdateScreen();
	      }
//	      else {
//	          comandos();  // Mostrar comandos si el usuario está autenticado
//	      }
//
         low_power_mode();  // Modo de bajo consumo
	      }

//	  if (ingresar_clave()) { // Verificar clave
//	              char tecla = keypad_scan(GPIO_PIN_All);
//
//	              if (tecla == '1' || tecla == '2' || tecla == '3') {
//	                  modo_direccional();
//	              } else if (tecla == 'A') {
//	                  cambiar_clave(); // Cambiar clave
//	              }
//	          }


	  if (ring_buffer_is_full(&usart2_rb) != 0) {
		  printf("Received:\r\n");
		  while (ring_buffer_is_empty(&usart2_rb) == 0) {
			  uint8_t data;
			  ring_buffer_read(&usart2_rb, &data);
			  HAL_UART_Transmit(&huart2, &data, 1, 10);
		  }
		  printf("\r\n");
	  }

	  low_power_mode();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_HEARTBEAT_Pin|LED_LEFT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ROW_1_GPIO_Port, ROW_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ROW_2_Pin|ROW_4_Pin|ROW_3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUTTON_LEFT_Pin BUTTON_RIGHT_Pin */
  GPIO_InitStruct.Pin = BUTTON_LEFT_Pin|BUTTON_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_HEARTBEAT_Pin LED_LEFT_Pin */
  GPIO_InitStruct.Pin = LED_HEARTBEAT_Pin|LED_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : COLUMN_1_Pin */
  GPIO_InitStruct.Pin = COLUMN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(COLUMN_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : COLUMN_4_Pin */
  GPIO_InitStruct.Pin = COLUMN_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(COLUMN_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COLUMN_2_Pin COLUMN_3_Pin */
  GPIO_InitStruct.Pin = COLUMN_2_Pin|COLUMN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ROW_1_Pin */
  GPIO_InitStruct.Pin = ROW_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ROW_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW_2_Pin ROW_4_Pin ROW_3_Pin */
  GPIO_InitStruct.Pin = ROW_2_Pin|ROW_4_Pin|ROW_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RIGHT_Pin */
  GPIO_InitStruct.Pin = LED_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RIGHT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
