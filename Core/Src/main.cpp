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

/* USER CODE BEGIN PV */

//pedal input pin gpio
const uint8_t pedal_pin[]={11,10};

//columns and rows gpio numbers for keypad
const uint8_t columns[]={43,42,41,40};

const uint8_t rows[]={25,26,27,28};

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

//#include "usbd_customhid.h"

//#include "usb_xinput.h"

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
  //MX_USB_DEVICE_Init();
  gamepad=new Xbox();

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

gamepad->press(BUTTON_A);




  while (1)
  {

    /* USER CODE END WHILE */

	 // USBD_LL_Transmit(&hUsbDeviceFS,0x81,tx,20);
	  //gamepad->setButton(BUTTON_A,but);

	  //HAL_Delay(2000);

	  while(!IS_USB_SUSPEND())
	  {

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
  __HAL_RCC_GPIOC_CLK_ENABLE();

  //GPIO_InitTypeDef gpio; // obiekt gpio będący konfiguracją portów GPIO
  GPIO_InitStruct.Pin = GPIO_PIN_13; // konfigurujemy pin 13
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // jako wyjście
  GPIO_InitStruct.Pull = GPIO_NOPULL; // rezystory podciągające są wyłączone
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // wystarczą nieskie częstotliwości przełączania
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
