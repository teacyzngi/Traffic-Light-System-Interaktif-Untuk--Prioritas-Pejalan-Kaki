/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Traffic Light System UGM 
  * @author         : Shofwah Syazwina
  * @date           : 2025
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h> // for abs()

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define LCD_ADDR_1 (0x27 << 1)   

uint8_t LCD_ADDR = LCD_ADDR_1; 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    RED = 0,
    YELLOW = 1,
    GREEN = 2
} TrafficColor;

typedef struct {
    GPIO_TypeDef* red_port;
    uint16_t red_pin;
    GPIO_TypeDef* yellow_port;
    uint16_t yellow_pin;
    GPIO_TypeDef* green_port;
    uint16_t green_pin;
} RoadLights;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN_TIMING 1000   
#define MAX_TIMING 10000  
#define YELLOW_TIME 2000  

#define BUZZER_PORT GPIOA
#define BUZZER_PIN  GPIO_PIN_1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
RoadLights roads[3] = {
    {GPIOA, GPIO_PIN_3, GPIOA, GPIO_PIN_5, GPIOA, GPIO_PIN_6},
  
    {GPIOB, GPIO_PIN_7, GPIOB, GPIO_PIN_6, GPIOA, GPIO_PIN_10},
    
    {GPIOA, GPIO_PIN_7, GPIOB, GPIO_PIN_3, GPIOA, GPIO_PIN_8}
};

// Global Variables
volatile uint8_t currentMessage = 0; 
volatile uint8_t emergencyFlag = 0;  
uint8_t currentRoad = 0;             
uint32_t greenTiming = 5000;         
uint8_t lastButtonState = 1;         
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
void setTrafficLight(uint8_t road, TrafficColor color);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_clear(void);
void lcd_goto_xy(int row, int col);
void lcd_print(char *str);
void toggleLCDMessage(void);
void initLCD(void);
void emergencyStop(void);
uint16_t readPotentiometer(void);
uint32_t mapTimingValue(uint16_t adcValue);
uint8_t checkButton1(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void setTrafficLight(uint8_t road, TrafficColor color) {
    if (road > 2) return; 
    
    HAL_GPIO_WritePin(roads[road].red_port, roads[road].red_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(roads[road].yellow_port, roads[road].yellow_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(roads[road].green_port, roads[road].green_pin, GPIO_PIN_RESET);
    
    switch(color) {
        case RED:
            HAL_GPIO_WritePin(roads[road].red_port, roads[road].red_pin, GPIO_PIN_SET);
            break;
        case YELLOW:
            HAL_GPIO_WritePin(roads[road].yellow_port, roads[road].yellow_pin, GPIO_PIN_SET);
            break;
        case GREEN:
            HAL_GPIO_WritePin(roads[road].green_port, roads[road].green_pin, GPIO_PIN_SET);
            break;
    }
}
void toggleLCDMessage(void) {
    lcd_clear();
    
    if (currentMessage == 0) {
        lcd_goto_xy(0, 0);
        lcd_print("  Hati-hati");
        lcd_goto_xy(1, 0);
        lcd_print("   di jalan");
        currentMessage = 1; 
    } else {
        lcd_goto_xy(0, 0);
        lcd_print("Selamat datang");
        lcd_goto_xy(1, 0);
        lcd_print("di DTETI UGM");
        currentMessage = 0; 
    }
}
void emergencyStop(void) {
    for(int blink = 0; blink < 5; blink++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
        
        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
        
        HAL_Delay(100); 

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);
        
        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
        
        HAL_Delay(100); 
    }

    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    
    for (int i = 0; i < 3; i++) {
        setTrafficLight(i, RED);
    }
    
    uint8_t savedMessage = currentMessage;
    
    lcd_clear();
    lcd_goto_xy(0, 0);
    lcd_print("!! EMERGENCY !!");
    lcd_goto_xy(1, 0);
    lcd_print("Traffic Stopped");
    
    HAL_Delay(3000); 
    
    lcd_clear();
    if (savedMessage == 0) {
        lcd_goto_xy(0, 0);
        lcd_print("  Hati-hati");
        lcd_goto_xy(1, 0);
        lcd_print("   di jalan");
    } else {
        lcd_goto_xy(0, 0);
        lcd_print("Selamat datang");
        lcd_goto_xy(1, 0);
        lcd_print("di DTETI UGM");
    }
    
    emergencyFlag = 0; 
}
uint16_t readPotentiometer(void) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    uint16_t adcValue = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return adcValue;
}

uint32_t mapTimingValue(uint16_t adcValue) {
    uint32_t timing = ((adcValue * 9000) / 4095) + 1000;
    return timing;
}

uint8_t checkButton1(void) {
    uint8_t currentState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
    
    if (lastButtonState == GPIO_PIN_SET && currentState == GPIO_PIN_RESET) {
        HAL_Delay(50); 
        lastButtonState = currentState;
        return 1; 
    }
    
    lastButtonState = currentState;
    return 0; 
}

void lcd_send_cmd(char cmd) {
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    data_t[0] = data_u | 0x0C; data_t[1] = data_u | 0x08;
    data_t[2] = data_l | 0x0C; data_t[3] = data_l | 0x08;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, (uint8_t*)data_t, 4, 100);
    HAL_Delay(2);
}

void lcd_send_data(char data) {
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    data_t[0] = data_u | 0x0D; data_t[1] = data_u | 0x09;
    data_t[2] = data_l | 0x0D; data_t[3] = data_l | 0x09;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, (uint8_t*)data_t, 4, 100);
    HAL_Delay(1);
}

void lcd_clear(void) {
    lcd_send_cmd(0x01);
    HAL_Delay(5);
}

void lcd_goto_xy(int row, int col) {
    uint8_t pos = (row == 0) ? (0x80 + col) : (0xC0 + col);
    lcd_send_cmd(pos);
    HAL_Delay(1);
}

void lcd_print(char *str) {
    while (*str) lcd_send_data(*str++);
}

void initLCD(void) {
    HAL_Delay(50);
    if (HAL_I2C_IsDeviceReady(&hi2c1, 0x27 << 1, 3, 100) == HAL_OK) LCD_ADDR = 0x27 << 1;
    else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        return;
    }
    HAL_Delay(100);
    lcd_send_cmd(0x33); HAL_Delay(10);
    lcd_send_cmd(0x32); HAL_Delay(10);
    lcd_send_cmd(0x28); HAL_Delay(5);
    lcd_send_cmd(0x0C); HAL_Delay(5);
    lcd_send_cmd(0x06); HAL_Delay(5);
    lcd_send_cmd(0x01); HAL_Delay(5);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t lastUpdate = 0;
  uint32_t lastADCRead = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
  HAL_Delay(200); 
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
  
  HAL_Delay(1800); 
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(500);

  HAL_Delay(200);
  initLCD();
  HAL_Delay(200);
  
  currentMessage = 0;
  toggleLCDMessage();
  HAL_Delay(1000);

  for (int i = 0; i < 3; i++) setTrafficLight(i, RED);
  HAL_Delay(1000);
  
  currentRoad = 0;
  setTrafficLight(currentRoad, GREEN);
  setTrafficLight(1, RED);
  setTrafficLight(2, RED);
  lastUpdate = HAL_GetTick();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    
    /* USER CODE BEGIN 3 */
    
    static uint8_t lastPC14State = GPIO_PIN_SET;
    uint8_t currentPC14State = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14);
    
    if (lastPC14State == GPIO_PIN_SET && currentPC14State == GPIO_PIN_RESET) {
        HAL_Delay(50); 
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET) {
            emergencyFlag = 1;
        }
    }
    lastPC14State = currentPC14State;
    
    if (emergencyFlag) {
        emergencyStop();
    }
    
    if (HAL_GetTick() - lastADCRead >= 500) {
        uint16_t adcValue = readPotentiometer();
        uint32_t newTiming = mapTimingValue(adcValue);
        
        if (abs((int)newTiming - (int)greenTiming) > 500) {
            greenTiming = newTiming;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_Delay(50);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        }
        lastADCRead = HAL_GetTick();
    }
    
    if (checkButton1()) {
        toggleLCDMessage();
        HAL_Delay(50);
    }
    
    uint32_t currentTime = HAL_GetTick();
    uint32_t elapsed = currentTime - lastUpdate;
    
    if (elapsed >= greenTiming) {
        setTrafficLight(currentRoad, YELLOW);
        HAL_Delay(YELLOW_TIME);
        
        setTrafficLight(currentRoad, RED);
        
        currentRoad = (currentRoad + 1) % 3;
        
        setTrafficLight(currentRoad, GREEN);
        
        for (int i = 0; i < 3; i++) {
            if (i != currentRoad) setTrafficLight(i, RED);
        }
        
        lastUpdate = HAL_GetTick();
    }
    
    HAL_Delay(10);
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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10707DBC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  /* --- CONFIG LEDS --- */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /* --- CONFIG BUZZER (PA1) --- */
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = BUZZER_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);

  /* --- CONFIG GPIOA LEDS --- */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* --- CONFIG GPIOB LEDS --- */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* --- CONFIG BUTTON PC13 --- */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* --- CONFIG BUTTON PC14  --- */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; 
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* --- CONFIG LCD I2C PINS --- */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif