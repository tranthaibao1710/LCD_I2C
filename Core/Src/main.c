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
#include <stdio.h>
#include <string.h>
#define Rs 0x01  // P0
#define Rw 0x02  // P1 (không dùng, có th? b?)
#define En 0x04  // P2
#define Bl 0x08  // P3
#define LCD_ADDR (0x27 << 1)  // n?u module c?a b?n ? d?a ch? 0x27

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
typedef struct {
    I2C_HandleTypeDef *hi2c;     // I2C handler for communication
    uint8_t address;            // I2C address of the LCD
} I2C_LCD_HandleTypeDef;

/**
 * @brief Initializes the LCD.
 * @param lcd: Pointer to the LCD handle
 */
void lcd_init(I2C_LCD_HandleTypeDef *lcd);

/**
 * @brief Sends a command to the LCD.
 * @param lcd: Pointer to the LCD handle
 * @param cmd: Command byte to send
 */
void lcd_send_cmd(I2C_LCD_HandleTypeDef *lcd, char cmd);

/**
 * @brief Sends data (character) to the LCD.
 * @param lcd: Pointer to the LCD handle
 * @param data: Data byte to send
 */
void lcd_send_data(I2C_LCD_HandleTypeDef *lcd, char data);

/**
 * @brief Sends a single character to the LCD.
 * @param lcd: Pointer to the LCD handle
 * @param ch: Character to send
 */
void lcd_putchar(I2C_LCD_HandleTypeDef *lcd, char ch);

/**
 * @brief Sends a string to the LCD.
 * @param lcd: Pointer to the LCD handle
 * @param str: Null-terminated string to send
 */
void lcd_puts(I2C_LCD_HandleTypeDef *lcd, char *str);

/**
 * @brief Moves the cursor to a specific position on the LCD.
 * @param lcd: Pointer to the LCD handle
 * @param col: Column number (0-15)
 * @param row: Row number (0 or 1)
 */
void lcd_gotoxy(I2C_LCD_HandleTypeDef *lcd, int col, int row);

/**
 * @brief Clears the LCD display.
 * @param lcd: Pointer to the LCD handle
 */
void lcd_clear(I2C_LCD_HandleTypeDef *lcd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ----- LCD I2C Functions -----
// ================= LCD I2C (HD44780 + PCF8574) ===================

void lcd_send_cmd(I2C_LCD_HandleTypeDef *lcd, char cmd)
{
    char upper_nibble, lower_nibble;
    uint8_t data_t[4];

    upper_nibble = (cmd & 0xF0);            // Extract upper nibble
    lower_nibble = ((cmd << 4) & 0xF0);     // Extract lower nibble
    
    data_t[0] = upper_nibble | 0x0C;  // en=1, rs=0
    data_t[1] = upper_nibble | 0x08;  // en=0, rs=0
    data_t[2] = lower_nibble | 0x0C;  // en=1, rs=0
    data_t[3] = lower_nibble | 0x08;  // en=0, rs=0

    HAL_I2C_Master_Transmit(lcd->hi2c, lcd->address, data_t, 4, 100);
}

/**
 * @brief  Sends data (character) to the LCD.
 * @param  lcd: Pointer to the LCD handle
 * @param  data: Data byte to send
 * @retval None
 */
void lcd_send_data(I2C_LCD_HandleTypeDef *lcd, char data)
{
    char upper_nibble, lower_nibble;
    uint8_t data_t[4];

    upper_nibble = (data & 0xF0);            // Extract upper nibble
    lower_nibble = ((data << 4) & 0xF0);     // Extract lower nibble
    
    data_t[0] = upper_nibble | 0x0D;  // en=1, rs=1
    data_t[1] = upper_nibble | 0x09;  // en=0, rs=1
    data_t[2] = lower_nibble | 0x0D;  // en=1, rs=1
    data_t[3] = lower_nibble | 0x09;  // en=0, rs=1

    HAL_I2C_Master_Transmit(lcd->hi2c, lcd->address, data_t, 4, 100);
}

/**
 * @brief  Clears the LCD display.
 * @param  lcd: Pointer to the LCD handle
 * @retval None
 */
void lcd_clear(I2C_LCD_HandleTypeDef *lcd)
{
    lcd_send_cmd(lcd, 0x80);  // Move cursor to the home position
    // Clear all characters
    // 16x4 = 64 characters
    // 20x4 = 80 characters
    // So 80 character clearing is enough for both 16x2, 16x4, 20x2 and 20x5 displays
    for (int i = 0; i < 80; i++)
    {
        lcd_send_data(lcd, ' ');  // Write a space on each position
    }
}

/**
 * @brief  Moves the cursor to a specific position on the LCD.
 * @param  lcd: Pointer to the LCD handle
 * @param  col: Column number (0-15)
 * @param  row: Row number (0 or 1)
 * @retval None
 */
void lcd_gotoxy(I2C_LCD_HandleTypeDef *lcd, int col, int row)
{
    uint8_t address;

    switch (row)
    {
        case 0: address = 0x80 + col; break;  // First row
        case 1: address = 0xC0 + col; break;  // Second row
        case 2: address = 0x94 + col; break;  // Third row
        case 3: address = 0xD4 + col; break;  // Fourth row
        default: return;  // Ignore invalid row numbers
    }

    lcd_send_cmd(lcd, address);  // Send command to move the cursor
}

/**
 * @brief  Initializes the LCD in 4-bit mode.
 * @param  lcd: Pointer to the LCD handle
 * @retval None
 */
void lcd_init(I2C_LCD_HandleTypeDef *lcd)
{
    HAL_Delay(50);  // Wait for LCD power-up
    lcd_send_cmd(lcd, 0x30);  // Wake up command
    HAL_Delay(5);
    lcd_send_cmd(lcd, 0x30);  // Wake up command
    HAL_Delay(1);
    lcd_send_cmd(lcd, 0x30);  // Wake up command
    HAL_Delay(10);
    lcd_send_cmd(lcd, 0x20);  // Set to 4-bit mode
    HAL_Delay(10);

    // LCD configuration commands
    lcd_send_cmd(lcd, 0x28);  // 4-bit mode, 2 lines, 5x8 font
    HAL_Delay(1);
    lcd_send_cmd(lcd, 0x08);  // Display off, cursor off, blink off
    HAL_Delay(1);
    lcd_send_cmd(lcd, 0x01);  // Clear display
    HAL_Delay(2);
    lcd_send_cmd(lcd, 0x06);  // Entry mode: cursor moves right
    HAL_Delay(1);
    lcd_send_cmd(lcd, 0x0C);  // Display on, cursor off, blink off
}

/**
 * @brief  Sends a string to the LCD.
 * @param  lcd: Pointer to the LCD handle
 * @param  str: Null-terminated string to display
 * @retval None
 */
void lcd_puts(I2C_LCD_HandleTypeDef *lcd, char *str)
{
    while (*str) lcd_send_data(lcd, *str++);  // Send each character in the string
}

/**
 * @brief  Sends a single character to the LCD.
 * @param  lcd: Pointer to the LCD handle
 * @param  ch: Character to send
 * @retval None
 */
void lcd_putchar(I2C_LCD_HandleTypeDef *lcd, char ch)
{
    lcd_send_data(lcd, ch);  // Send the character to the display
}

uint16_t Read_ADC_Channel(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;               // ? Channel b?n mu?n d?c
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(hadc, &sConfig);

    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 100);
    uint16_t val = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return val;
}
uint16_t HAL_ADC_ReadAverage(ADC_HandleTypeDef *hadc, uint32_t channel, uint8_t n)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < n; i++)
        sum += Read_ADC_Channel(hadc, channel);
    return (uint16_t)(sum / n);
}
uint16_t adc_avg;
float adc_voltage;
float tem;
uint8_t check = 0 ; 
uint16_t cnt = 0;
char lcd_buffer[20];
I2C_LCD_HandleTypeDef lcd;
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
     lcd.hi2c = &hi2c1;
    lcd.address = 0x4E;   // ho?c 0x27 << 1 = 0x4E (tu? vào module PCF8574 c?a b?n)

    // Kh?i t?o LCD
    lcd_init(&lcd);

    // Xóa màn hình
    lcd_clear(&lcd);

    // Hi?n th? dòng d?u tiên
    

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		 adc_avg = HAL_ADC_ReadAverage(&hadc1, ADC_CHANNEL_0, 10); // Ð?c trung bình 10 m?u
     adc_voltage = ((float)adc_avg / 4095) * 3.3;
     tem = adc_voltage*100; 
		 HAL_Delay(200); 
     sprintf(lcd_buffer, "Nhiet do: %.1f C", tem);

     lcd_gotoxy(&lcd, 0, 0);    // Di chuy?n d?n dòng d?u
     lcd_puts(&lcd, lcd_buffer); // In ra LCD
		 if(tem < 90  && check == 0) {
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); //relay
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); //led
     lcd_gotoxy(&lcd, 0, 1);
     lcd_puts(&lcd, "dang dun nuoc");
		 }
		 else if( tem >90 && check == 0){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);//còi
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_Delay(200);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			 HAL_Delay(200);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);//còi
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_Delay(200);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

      lcd_gotoxy(&lcd, 0, 1);
      lcd_puts(&lcd, "da dun xong  ");
			check = 1;
		 }
		 else
			 {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    lcd_gotoxy(&lcd, 0, 1);
    lcd_puts(&lcd, "da dun xong  ");
		 }
		  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET)
    {
        HAL_Delay(50); // ch?ng d?i
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET)
        {
            check = 0;
            
        }
    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  
  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
