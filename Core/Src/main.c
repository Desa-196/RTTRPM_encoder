/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/


#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "enc28j60.h"
#include "send_RTTRPM.h"
#include <stdio.h>
#include <string.h>
#include <uip.h>
#include <uip_arp.h>

#define DEVICE  2

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
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
/* USER CODE BEGIN PV */



#define ALL 3

//При еденице уже не справляется, отдает только 700 пакетов/с вместо 1000
#define DELAY_READ_ENCODER 2

double test = 0;
int dir = 1;

//Множитель
int multiplier = 1;
//Смещение
int offset = 0;

uint16_t stat_link;
uint8_t lstat = 0;

uint32_t old_time[3] = {0};
uint32_t latency[3] = {0};

int8_t config_mode = 0;

//время бездействия в режиме конфигурации, для отключения режима при бездействии
uint32_t time_to_config_mode = 0;

xQueueHandle q;
xQueueHandle xQueueCommand;

double EncoderPosition[3] = {0, 0, 0};
double oldEncoderPosition[3] = {0, 0, 0};

uint16_t lan_stat;
extern uint8_t isConnected;
int8_t old_isConnected;

uint16_t reset_encoder(int i)
{
	EncoderPosition[i] = 0;
}

uint16_t read_encoder(int i)
{
	uint16_t counter = 0;
	switch(i)
	{
		case 0:
			counter = TIM3->CNT;
			break;
		case 1:
			counter = TIM2->CNT;
			break;
		case 2:
			counter = TIM4->CNT;
			break;
	}
	return counter;
}

void led_chanel_on(int i)
{
	switch(i)
	{
		case 0:
			LD_0_ON;
			break;
		case 1:
			LD_1_ON;
			break;
		case 2:
			LD_2_ON;
			break;
		case 3:
			LD_0_ON;
			LD_1_ON;
			LD_2_ON;
			break;
	}
}
void led_chanel_toggle(int i)
{
	switch(i)
	{
		case 0:
			LD_0_TG;
			break;
		case 1:
			LD_1_TG;
			break;
		case 2:
			LD_2_TG;
			break;
		case 3:
			LD_0_TG;
			LD_1_TG;
			LD_2_TG;
			break;
	}
}
void led_chanel_off(int i)
{
	switch(i)
	{
		case 0:
			LD_0_OFF;
			break;
		case 1:
			LD_1_OFF;
			break;
		case 2:
			LD_2_OFF;
			break;
		case 3:
			LD_0_OFF;
			LD_1_OFF;
			LD_2_OFF;
			break;
	}
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);

/* USER CODE BEGIN PFP */

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
  MX_TIM4_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

	#if DEVICE == 1
		struct uip_eth_addr mac = { {0x00,0x15,0x48,0xBF,0xF0,0x51}};
	#elif DEVICE == 2
		struct uip_eth_addr mac = { {0x00,0x15,0x48,0xBF,0xF0,0x51}};
	#endif
		
		
	q = xQueueCreate( 1, sizeof(double[3]));
	xQueueCommand = xQueueCreate( 10, sizeof(uint8_t));
	
	enc28j60_ini(&mac);
	
	uip_init();
	uip_arp_init();
	uip_setethaddr(mac);	
	
	HAL_Delay(100);
	
	hello_world_init();
	
	struct timer periodic_timer, arp_timer;
  LD_OFF;
	/* Initialise the uIP stack. */
	timer_set(&periodic_timer, configTICK_RATE_HZ / 2);
	timer_set(&arp_timer, configTICK_RATE_HZ * 10);
	
	RTTRPM_init(&uip_ethaddr, sizeof(uip_ethaddr));
	
	uip_ipaddr_t ipaddr;
	
	#if DEVICE == 1
	uip_ipaddr(ipaddr, 192, 168, 1, 50);
	#elif DEVICE == 2
	uip_ipaddr(ipaddr, 192, 168, 1, 51);
	#endif
	uip_sethostaddr(ipaddr);
	uip_ipaddr(ipaddr, 192, 168, 1, 1);
	uip_setdraddr(ipaddr);
	uip_ipaddr(ipaddr, 255, 255, 255, 0);
	uip_setnetmask(ipaddr);


	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);



  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityHigh, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityLow, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityHigh, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, StartTask04, osPriorityLow, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65534;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65534;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65534;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void uip_log(char *msg) {

} 
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (int i=0; 1; i++) {
		osDelay(1000);
		if(i == 10)
		{
			uip_arp_timer();
			i = 0;
		}
		
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
	uint8_t short_state = 0;
  uint8_t long_state = 0;
  uint32_t time_key1 = 0;
	
  /* Infinite loop */
  for(;;)
  {
    uip_len = enc28j60_packetReceive((uint8_t *) uip_buf, UIP_BUFSIZE);
		if (uip_len > 0) {
			if (BUF->type == htons(UIP_ETHTYPE_IP)) {
				uip_arp_ipin();
				uip_input();
				if (uip_len > 0) {
					uip_arp_out();
					enc28j60_packetSend((uint8_t *) uip_buf, uip_len);
				}
			} else if (BUF->type == htons(UIP_ETHTYPE_ARP)) {
				uip_arp_arpin();
				if (uip_len > 0) {
					enc28j60_packetSend((uint8_t *) uip_buf, uip_len);	
				}
			}
		}
		
		for(int i = 0; i < UIP_UDP_CONNS; i++) {

			uip_udp_periodic(i);
			if(uip_len > 0) {
				uip_arp_out();
				enc28j60_packetSend((uint8_t *) uip_buf, uip_len);
			}
		}
		
		for (uint8_t i = 0; i < UIP_CONNS; i++) {
			uip_periodic(i);
			if (uip_len > 0) {
				uip_arp_out();
				enc28j60_packetSend((uint8_t *) uip_buf, uip_len);
			}
		}
		
		
		//Отслеживаем нажатие кнопки на контроллере	
		uint32_t ms = HAL_GetTick();
		uint8_t key1_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	
		
		if(key1_state == 0 && !short_state && (ms - time_key1) > 50) 
		{
			short_state = 1;
			long_state = 0;
			time_key1 = ms;
		}
		else if(key1_state == 0 && !long_state && (ms - time_key1) > 2000) 
		{
			long_state = 1;
			// действие на длинное нажатие
			
			if(config_mode == 0)
			{
				time_to_config_mode = HAL_GetTick();
				
				config_mode = 1;
				led_chanel_on(0);
				led_chanel_off(1);
				led_chanel_off(2);
			}
			else
			{
				if(config_mode == 4) {	
					
					led_chanel_off(ALL);
					HAL_Delay(100);
					led_chanel_on(ALL);
					HAL_Delay(100);
					led_chanel_off(ALL);
					HAL_Delay(100);
					led_chanel_on(ALL);
					HAL_Delay(100);
					led_chanel_off(ALL);
					reset_encoder(0);
					reset_encoder(1);
					reset_encoder(2);
				}
				else
				{				
					led_chanel_off(config_mode-1);
					HAL_Delay(100);
					led_chanel_on(config_mode-1);
					HAL_Delay(100);
					led_chanel_off(config_mode-1);
					HAL_Delay(100);
					led_chanel_on(config_mode-1);
					HAL_Delay(100);
					led_chanel_off(config_mode-1);					
					reset_encoder(config_mode-1);
				}
				
				led_chanel_off(ALL);
				config_mode = 0;
				
			}
		}
		else if(key1_state == 1 && short_state && (ms - time_key1) > 50) 
		{
			short_state = 0;
			time_key1 = ms;

			if(!long_state)
			{
				// действие на короткое нажатие
				if(config_mode != 0)
				{
					time_to_config_mode = HAL_GetTick();
					
					if(config_mode == 4)
					{
						config_mode = 1;
					}
					else
					{
						config_mode ++;
					}
					
					switch(config_mode)
					{
						case 1:
							led_chanel_on(0);
							led_chanel_off(1);
							led_chanel_off(2);
							break;
						case 2:
							led_chanel_off(0);
							led_chanel_on(1);
							led_chanel_off(2);
							break;
						case 3:
							led_chanel_off(0);
							led_chanel_off(1);
							led_chanel_on(2);
							break;
						case 4:
							led_chanel_on(ALL);
							break;
					}
				}
			}
		}
		
		if(HAL_GetTick() - time_to_config_mode > 10000 && config_mode != 0)
		{
			config_mode = 0;
			led_chanel_off(ALL);
		}
		
		taskYIELD();
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
	
	//Запускаем автообновление PHY регистра PHSTAT1 для опроса состояния линка
	enc28j60PhystartScan(PHSTAT1);
	
	
	uint16_t counter[3] = {TIM2->CNT, TIM3->CNT, TIM4->CNT};
	uint16_t oldCounter[3] = {counter[0], counter[1], counter[2]};
	uint8_t count_period_read_encoder = 0;
	xQueueSend(q, ( void * ) &EncoderPosition, portMAX_DELAY  );
	
	uint32_t time = 0;
	
  /* Infinite loop */
  for(;;)
  {
		
		//Если получили комманду по telnet
		if( uxQueueMessagesWaiting(xQueueCommand) > 0)
		{
			uint8_t commandName;
			xQueueReceive( xQueueCommand, &( commandName ), portMAX_DELAY ); 
			switch(commandName){
				case COMMAND_RESET_COUNTER:	
					
					break;
				case SEND_MOTOR_POSITION:
				//	if( isConnected == 1 ) xQueueSend(q, ( void * ) &motorPosition, portMAX_DELAY  );
					break;	
		}
		}
		
		if((HAL_GetTick() - time) > 500) // интервал 1000мс = 1сек
		{ 
			 // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);
				time = HAL_GetTick();
		}
		
		
		for(int i=0; i<=2; i++)
		{
			//Считываем значение энкодера для канала i (от 0 до 2 - всего 3 канала)
			counter[i] = read_encoder(i);
			
			
			latency[i] = HAL_GetTick() - old_time[i];
			old_time[i] = HAL_GetTick();
			
		
			if (abs(oldCounter[i] - counter[i]) > 32767)
			{
					if (oldCounter[i] < counter[i])
					{
							EncoderPosition[i] -= 65535 - counter[i] + oldCounter[i];
					}
					else
					{
							EncoderPosition[i] += 65535 - oldCounter[i] + counter[i];
					}
			}
			else 
			{
					EncoderPosition[i] += (counter[i] - oldCounter[i]);
			}
			
			//Мигаем синими светодиодами, если не включен режим конфигурации и есть сигнал на каналах.
			if(config_mode == 0)
				{
					//Если изменение больше чем две еденицы, т.е данные счетчик получил по двум каналам таймера
					if(abs((int)(oldEncoderPosition[i] - EncoderPosition[i])) >= 2)
					{
						//То каждые 10 иттераций меняем состояние светодиода данного канала
						if(count_period_read_encoder == 10)
						{
							led_chanel_toggle(i);
							count_period_read_encoder = 0;
						}
						count_period_read_encoder++;
					}
					else led_chanel_off(i);
				}
				
			oldEncoderPosition[i] = EncoderPosition[i];
			oldCounter[i] = counter[i];
			

		}
		if(test <= 0 ) dir = 1;
		else if(test >= 1000) dir = 0;
		
		if(dir == 1) test += 0.1;
		if(dir == 0) test -= 0.1;
		
		EncoderPosition[2] = (int)test;
		//Отправляем значение энкодера  в очередь на отправку 
		xQueueSend(q, ( void * ) &EncoderPosition, portMAX_DELAY  );
		osDelay(DELAY_READ_ENCODER);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
		

  /* Infinite loop */
  for(;;)
  {
		//Читаем информацию о линке, и проверяем, если за 100 проходов линк ни разу не появился, то зажигаем красный светодиод.
		//stat_link = enc28j60PhyReadH(PHSTAT2);
//		if(stat_link == 0) lstat = (lstat==10)?10:lstat+1;
//		else lstat = 0;
//		
//		if(lstat == 10) LD_RED_ON;
//		else LD_RED_OFF;
//		stat_link = enc28j60PhyReadScan()&0x04;
//		if(stat_link != 0) LD_RED_OFF;
//		else LD_RED_ON;

    osDelay(100);
  }
  /* USER CODE END StartTask04 */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
