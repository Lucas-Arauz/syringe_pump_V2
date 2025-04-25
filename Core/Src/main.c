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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "I2C_LCD_cfg.h"
#include "I2C_LCD.h"
#include "Util.h"
#include "TMC2208.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	float vol_jeringa;
	float vol_dispensar;
	float caudal;
	float diametro;
	float largo;
} ParametrosDispensado;

typedef struct
{
	uint32_t total_pasos;
	uint16_t arr;
	uint8_t micropasos;
} MovimientoMotor;

typedef enum
{
	MENU_VJERINGA,
	MENU_VDISPENSAR,
	MENU_CAUDAL,
	MENU_DIAM_LARGO,
	MENU_CONFIRMAR,
	MENU_EJECUTANDO
} EstadoMenu;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PASOS_POR_VUELTA 	200
#define PASO_HUSILLO_MM 	8.0f
#define FCLK_TIMER 			72000000UL
#define MAX_STR 			32

#define MyI2C_LCD 			I2C_LCD_1

#define DEBOUNCE_DELAY		150

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
ParametrosDispensado parametros = {0};
MovimientoMotor motor_cfg = {0};
EstadoMenu estado_actual = MENU_VJERINGA;

volatile uint32_t pasos_dados = 0;
volatile uint32_t total_pasos_a_dar = 0;

uint32_t last_interrupt_time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
float calcular_area_mm2(float d);
float calcular_desplazamiento_mm(ParametrosDispensado *p);
MovimientoMotor calcular_parametros_motor(ParametrosDispensado *p);
void configurar_timer_motor(uint16_t arr);
void mostrar_menu(void);
void avanzar_menu(void);
void incrementar_valor(void);
void decrementar_valor(void);
void boton_select_callback(void);

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
	MX_USART1_UART_Init();
	MX_TIM3_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	I2C_LCD_Init(MyI2C_LCD);
	I2C_LCD_Clear(MyI2C_LCD);
	mostrar_menu();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 7200-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

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
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, STEPPER_EN_Pin|STEPPER_DIR_Pin|STEPPER_STEP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : STEPPER_EN_Pin STEPPER_DIR_Pin STEPPER_STEP_Pin */
	GPIO_InitStruct.Pin = STEPPER_EN_Pin|STEPPER_DIR_Pin|STEPPER_STEP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN_DOWN_Pin BTN_SELECT_Pin BTN_UP_Pin */
	GPIO_InitStruct.Pin = BTN_DOWN_Pin|BTN_SELECT_Pin|BTN_UP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// === FUNCIONES DE CÁLCULO ===
float calcular_area_mm2(float d)
{
    return (M_PI * d * d) / 4.0f;
}

float calcular_desplazamiento_mm(ParametrosDispensado *p)
{
    if(p->diametro > 0.0f)
        return (p->vol_dispensar * 1000.0f) / calcular_area_mm2(p->diametro);
    else
        return (p->vol_dispensar / p->vol_jeringa) * p->largo;
}

MovimientoMotor calcular_parametros_motor(ParametrosDispensado *p)
{
    MovimientoMotor res = {0};
    const int micros[] = {256, 128, 64, 32, 16, 8, 4, 2, 1};
    float mm = calcular_desplazamiento_mm(p);
    float tiempo = (p->vol_dispensar / p->caudal) * 60.0f;

    for(uint8_t i = 0; i < sizeof(micros); i++)
    {
        int micro = micros[i];
        float pasos = (mm / PASO_HUSILLO_MM) * PASOS_POR_VUELTA * micro;
        float freq = pasos / tiempo;
        uint32_t arr = (FCLK_TIMER / freq) - 1;

        if (arr <= 0xFFFF)
        {
            res.total_pasos = (uint32_t)pasos;
            res.arr = (uint16_t)arr;
            res.micropasos = micro;
            break;
        }
    }

    return res;
}

void configurar_timer_motor(uint16_t arr)
{
    TIM2->ARR = arr;
    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
}

// === FUNCIONES DE INTERFAZ ===
void mostrar_menu(void)
{
    char buffer[MAX_STR];
    I2C_LCD_Clear(MyI2C_LCD);

    switch(estado_actual) {
        case MENU_VJERINGA:
            sprintf(buffer, "Vol Jer[ml]:%.1f", parametros.vol_jeringa);
        break;
        case MENU_VDISPENSAR:
            sprintf(buffer, "Vol Disp[ml]:%.1f", parametros.vol_dispensar);
        break;
        case MENU_CAUDAL:
            sprintf(buffer, "Caudal[ml/min]:%.1f", parametros.caudal);
        break;
        case MENU_DIAM_LARGO:
            if (parametros.diametro > 0)
                sprintf(buffer, "Diam[mm]:%.2f", parametros.diametro);
            else
                sprintf(buffer, "Largo[mm]:%.2f", parametros.largo);
        break;
        case MENU_CONFIRMAR:
            sprintf(buffer, "Iniciar");
        break;
        case MENU_EJECUTANDO:
            sprintf(buffer, "Ejecutando...");
        break;
        default:
        	return;
    }

    I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
    I2C_LCD_WriteString(MyI2C_LCD, buffer);
}

void avanzar_menu(void)
{
    if (estado_actual < MENU_CONFIRMAR)
        estado_actual++;
    else
        estado_actual = MENU_VJERINGA;

    mostrar_menu();
}

void incrementar_valor(void)
{
    switch (estado_actual) {
        case MENU_VJERINGA:
        	parametros.vol_jeringa += 0.1f;
        break;
        case MENU_VDISPENSAR:
        	parametros.vol_dispensar += 0.1f;
        break;
        case MENU_CAUDAL:
        	parametros.caudal += 0.1f;
        break;
        case MENU_DIAM_LARGO:
            if (parametros.diametro > 0)
                parametros.diametro += 0.1f;
            else
                parametros.largo += 0.1f;
        break;
        default:
        	return;
    }
    mostrar_menu();
}

void decrementar_valor(void)
{
    switch (estado_actual) {
        case MENU_VJERINGA:
        	if(parametros.vol_jeringa > 0.1f)
        		parametros.vol_jeringa -= 0.1f;
        break;
        case MENU_VDISPENSAR:
        	if(parametros.vol_dispensar > 0.1f)
        		parametros.vol_dispensar -= 0.1f;
        break;
        case MENU_CAUDAL:
        	if(parametros.caudal > 0.1f)
        		parametros.caudal -= 0.1f;
        break;
        case MENU_DIAM_LARGO:
            if(parametros.diametro > 0.1f)
                parametros.diametro -= 0.1f;
            else if(parametros.largo > 0.1f)
                parametros.largo -= 0.1f;
        break;
        default:
        	return;
    }
    mostrar_menu();
}

void boton_select_callback(void)
{
    if(estado_actual == MENU_CONFIRMAR)
    {
        estado_actual = MENU_EJECUTANDO;
        mostrar_menu();
        motor_cfg = calcular_parametros_motor(&parametros);
        configurar_timer_motor(motor_cfg.arr);
        total_pasos_a_dar = motor_cfg.total_pasos;
        pasos_dados = 0;
        TMC2208_SetMicrostepping(motor_cfg.micropasos);
    } else
    {
        avanzar_menu();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t interrup_time = HAL_GetTick();
	if(interrup_time - last_interrupt_time > DEBOUNCE_DELAY)
	{
		switch(GPIO_Pin)
		{
			case BTN_DOWN_Pin:
				 decrementar_valor();
			break;
			case BTN_SELECT_Pin:
				boton_select_callback();
			break;
			case BTN_UP_Pin:
				incrementar_valor();
			break;
		}
		last_interrupt_time = interrup_time;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{

	}
}

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
