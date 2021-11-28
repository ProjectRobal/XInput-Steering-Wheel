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
#include "Xbox.h"

//#define DEBUG



/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define HIGH GPIO_PIN_SET
#define LOW GPIO_PIN_RESET

#define BUTTON_GPIO_GROUP GPIOB

#define BUTTON_REFRESH 100

#define ROT_GPIO_GROUP GPIOB

#define ROT1 18
#define ROT2 19

#define START_GPIO_GROUP GPIOA

#define START_BUTTON GPIO_PIN_8

#define PEDAL_GPIO_GROUP GPIOA

#define PEDAL_LOW 682.f //ADC value when pedal is released
#define PEDAL_HIGH 4095.f //ADC value when pedal is pressed
#define PEDAL_PRE 3413.f

//#define ROT_ENC_A_GPIO 29
//#define ROT_ENC_B_GPIO 30

#define AXIS_ZERO 0

#define one_rotation 13 // 360 degress

#define MAX_PEDAL_VALUE 255

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

//pedal input pin gpio
const uint8_t pedal_pin[]={GPIO_PIN_1,GPIO_PIN_0};

//columns and rows gpio numbers for keypad
const uint16_t columns[]={GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_8};

const uint16_t rows[]={GPIO_PIN_12,GPIO_PIN_13,GPIO_PIN_14,GPIO_PIN_15};

int rowCounter =0; // row counter
int columnCounter =0; // column counter
int captureColumn =0;
bool foundCol = false;

long start=0;
long stop=0;

float max_angle=180.f;

int position=0;
int lastPos=0;

long TimeOfLastDebounce=0;
float DelayofDebounce = 5;

//hw_timer_t * timer = NULL;

int keysState=0x0;

int PreviousCLK=0;
int PreviousDATA=0;

uint8_t curr_adc=0;

//#include "usbd_customhid.h"

//#include "usb_xinput.h"

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

uint8_t GET_USB_STATUS()
{
	if (hUsbDeviceFS.pClassData == NULL)
		  {
		    return (uint8_t)USBD_FAIL;
		  }

	return hUsbDeviceFS.dev_state;
}

bool IS_USB_SUSPEND()
{
	return GET_USB_STATUS()==USBD_STATE_SUSPENDED;
}

void switch_ADC(uint8_t adc)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = adc;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  curr_adc=adc;

}

void rotary_task()
{
	if((HAL_GetTick()- TimeOfLastDebounce) > DelayofDebounce)
	  {

	    if((PreviousCLK == 0)&&(PreviousDATA == 1))
	    {

	      if((HAL_GPIO_ReadPin(ROT_GPIO_GROUP,ROT1)==1)&&(HAL_GPIO_ReadPin(ROT_GPIO_GROUP,ROT2)==0))
	      {
	        position++;
	      }
	       if((HAL_GPIO_ReadPin(ROT_GPIO_GROUP,ROT1)==1)&&(HAL_GPIO_ReadPin(ROT_GPIO_GROUP,ROT2)==1))
	      {
	        position--;
	      }



	    }

	    if((PreviousCLK == 1)&&(PreviousDATA == 0))
	    {

	      if((HAL_GPIO_ReadPin(ROT_GPIO_GROUP,ROT1)==0)&&(HAL_GPIO_ReadPin(ROT_GPIO_GROUP,ROT2)==1))
	      {
	        position++;
	      }
	       if((HAL_GPIO_ReadPin(ROT_GPIO_GROUP,ROT1)==0)&&(HAL_GPIO_ReadPin(ROT_GPIO_GROUP,ROT2)==0))
	      {
	        position--;
	      }

	      // make_y_axis_querry(pos_to_angle(position));

	    }

	      if((PreviousCLK == 1)&&(PreviousDATA == 1))
	    {

	      if((HAL_GPIO_ReadPin(ROT_GPIO_GROUP,ROT1)==0)&&(HAL_GPIO_ReadPin(ROT_GPIO_GROUP,ROT2)==1))
	      {
	        position++;
	      }
	       if((HAL_GPIO_ReadPin(ROT_GPIO_GROUP,ROT1)==0)&&(HAL_GPIO_ReadPin(ROT_GPIO_GROUP,ROT2)==0))
	      {
	        position--;
	      }

	       //make_y_axis_querry(pos_to_angle(position));

	    }

	      if((PreviousCLK == 0)&&(PreviousDATA == 0))
	    {

	      if((HAL_GPIO_ReadPin(ROT_GPIO_GROUP,ROT1)==1)&&(HAL_GPIO_ReadPin(ROT_GPIO_GROUP,ROT2)==0))
	      {
	        position++;
	      }
	       if((HAL_GPIO_ReadPin(ROT_GPIO_GROUP,ROT1)==1)&&(HAL_GPIO_ReadPin(ROT_GPIO_GROUP,ROT2)==1))
	      {
	        position--;
	      }

	       //make_y_axis_querry(pos_to_angle(position));

	    }


	    PreviousCLK=HAL_GPIO_ReadPin(ROT_GPIO_GROUP,ROT1);
	    PreviousDATA=HAL_GPIO_ReadPin(ROT_GPIO_GROUP,ROT2);



	    TimeOfLastDebounce=HAL_GetTick();

	  }
}

void button_task()
{

for(int j =0; j < 4; j++){

HAL_GPIO_WritePin(BUTTON_GPIO_GROUP,rows[j],HIGH);

}


HAL_GPIO_WritePin(BUTTON_GPIO_GROUP,rows[rowCounter],LOW);

for(columnCounter=0;columnCounter<4;columnCounter++)
{

if(!HAL_GPIO_ReadPin(BUTTON_GPIO_GROUP,columns[columnCounter]))
{
keysState |= (1<<(4*rowCounter + columnCounter));
}
else
{
keysState &= ~(1<<(4*rowCounter + columnCounter));
}

}

rowCounter++;

if(rowCounter>=4)
{
  rowCounter=0;
}

}


void pedals(Xbox* _pad)
{

if(HAL_ADC_PollForConversion(&hadc1,10)== HAL_OK)
{


	uint16_t adc_read=0;

adc_read=HAL_ADC_GetValue(&hadc1);


//out=map(adc_read,PEDAL_LOW,PEDAL_HIGH,0,255);

_pad->setTrigger((XInputControl)(TRIGGER_LEFT+curr_adc),adc_read);

HAL_ADC_Stop(&hadc1);

if(curr_adc==0)
{
	curr_adc=1;
}
else
{
	curr_adc=0;
}

switch_ADC(curr_adc);

HAL_ADC_Start(&hadc1);

}

//memmove(msg+(sizeof(int)*2)+(i*sizeof(short)),(const void*)&out,sizeof(short));


}

bool read_button_state(int row , int column)
{
	return keysState ^= (1<<(4*row + column));
}

#define _DPAD_LEFT read_button_state(0,3)
#define _DPAD_DOWN read_button_state(1,3)
#define _DPAD_UP read_button_state(2,3)
#define _DPAD_RIGHT read_button_state(3,3)

#define A_BUTTON read_button_state(0,2)
#define B_BUTTON read_button_state(1,2)
#define X_BUTTON read_button_state(2,2)
#define Y_BUTTON read_button_state(3,2)

#define R2_BUTTON read_button_state(0,1)
#define _START_BUTTON read_button_state(1,1)
#define _BACK_BUTTON read_button_state(2,1)
#define L2_BUTTON read_button_state(3,1)

#define R3_BUTTON read_button_state(0,0)
#define L3_BUTTON read_button_state(3,0)

void decode_buttons(Xbox *pad)
{
	pad->setButton(DPAD_LEFT,_DPAD_LEFT);
	pad->setButton(DPAD_RIGHT,_DPAD_RIGHT);
	pad->setButton(DPAD_UP,_DPAD_UP);
	pad->setButton(DPAD_DOWN,_DPAD_DOWN);

	pad->setButton(BUTTON_A,A_BUTTON);
	pad->setButton(BUTTON_B,B_BUTTON);
	pad->setButton(BUTTON_X,X_BUTTON);
	pad->setButton(BUTTON_Y,Y_BUTTON);

	pad->setButton(BUTTON_RB,R2_BUTTON);
	pad->setButton(BUTTON_START,_START_BUTTON);
	pad->setButton(BUTTON_BACK,_BACK_BUTTON);
	pad->setButton(BUTTON_LB,L2_BUTTON);

	pad->setButton(BUTTON_R3,R3_BUTTON);
	pad->setButton(BUTTON_L3,L3_BUTTON);

}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */


Xbox *gamepad;

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
  //MX_USB_DEVICE_Init();
  gamepad=new Xbox();

  gamepad->setTriggerRange(PEDAL_LOW, PEDAL_HIGH);
  gamepad->setJoystickRange(-500,500);

  //uint8_t index=3;
  //uint8_t mask=(1<<4);

  //tx[index] |= mask;




  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);

HAL_Delay(2000);

HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);





//bool but=false;

//gamepad->press(BUTTON_A);

HAL_ADC_Start(&hadc1);


  while (1)
  {

    /* USER CODE END WHILE */

	 // USBD_LL_Transmit(&hUsbDeviceFS,0x81,tx,20);
	  //gamepad->setButton(BUTTON_A,but);

	  //HAL_Delay(2000);

	  if(IS_USB_SUSPEND())
	  {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	  }

	  while(!IS_USB_SUSPEND())
	  {

		  rotary_task();

		  //Gamepad.yAxis(pos_to_angle(position));
		  if(position!=lastPos)
		  {
			  gamepad->setJoystickX(JOY_LEFT, position);

		   lastPos=position;
		  }

		  if(stop-start>=BUTTON_REFRESH)
		  {

		  button_task();

		  decode_buttons(gamepad);

		  start=HAL_GetTick();
		  }

		  pedals(gamepad);

	  /*if(gamepad->getLEDPattern()!=0)
	  {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);
	  }*/

	 // but=!but;

	  gamepad->send();


	  gamepad->receive();
	 // HAL_Delay(500);

    /* USER CODE BEGIN 3 */

	  stop=HAL_GetTick();

	  }

	//  HAL_Delay(10000);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  //ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  /*sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }*/

  switch_ADC(curr_adc);
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
  hi2c1.Init.ClockSpeed = 400000;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
