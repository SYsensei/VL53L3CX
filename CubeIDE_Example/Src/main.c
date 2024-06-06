/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2024 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "main.h"
/* Include VL53L3CX Bare Driver */
#include "vl53lx_api.h"
/* Include VL53L3CX Ultra Low Power Driver */
#include "VL53L3CX_ULP_API.h"
#include "53L3A2.h"
/* USER CODE END Includes */

/* Use this macro to run a Xtalk calibration. This is only require for the Bare
 * driver, using the ranging mode. Ultra low power does not require Xtalk
 * calibration.
 */
// #define ENABLE_XTALK_CALIBRATION

/* 150 frames at 33ms means approx 5 seconds before enabling low power */
#define NB_FRAMES_BEFORE_LOW_POWER		150
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
VL53LX_Dev_t                   dev;
VL53LX_DEV                     Dev = &dev;
int status;
volatile int IntCount;
#define isInterrupt 1 /* If isInterrupt = 1 then device working in hardware interrupt mode, else device working in polling mode */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void RangingLoop(void); /*  */
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);
  return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin==VL53L1X_INT_Pin)
  {
    IntCount++;
  }
}

/* This function is used to boot the VL53L3CX and start the ultra low power
 * detection. The example of configuration allows to have the following current
 * consumption in 2V8:
 *  - 65¦ÌA when no target is detected
 *  - 90¦ÌA when a target is detected.
 * The maximum ranging distance is approx 400mm.
 */

int InitAndStartLowPower(){
	uint8_t continue_loop = 1;

	printf("**************************************************\n");
	printf("Starting ultra low power - power consumption 65uA\n");
	printf("**************************************************\n");

	status = VL53L3CX_ULP_SensorInit((uint16_t)Dev->I2cDevAddr);

	/*  Example of configuration for generating an interrupt if something is detected below 300mm  */
	status = VL53L3CX_ULP_SetInterruptConfiguration((uint16_t)Dev->I2cDevAddr, 300, 1);

	status = VL53L3CX_ULP_SetInterMeasurementInMs((uint16_t)Dev->I2cDevAddr, 1000);
	status = VL53L3CX_ULP_SetMacroTiming((uint16_t)Dev->I2cDevAddr, 1);
	status = VL53L3CX_ULP_SetSignalThreshold((uint16_t)Dev->I2cDevAddr, 6000);
	status = VL53L3CX_ULP_SetSigmaThreshold((uint16_t)Dev->I2cDevAddr, 35);
	status = VL53L3CX_ULP_SetROI((uint16_t)Dev->I2cDevAddr, 16);

	status = VL53L3CX_ULP_StartRanging((uint16_t)Dev->I2cDevAddr);
	if(status)
	{
		printf("VL53L3CX_ULP_StartRanging failed: error = %d\n", status);
		return status;
	}

	/* Clear first interrupt (raised when ranging starts) */
	IntCount = 0;

	/* Wait for HW interrupt */
	printf("Ranging started. Put a target below 300mm to wake up the sensor...\n");
	do
	{
		__WFI();
		if(IntCount !=0 ){
			IntCount=0;
			status = VL53L3CX_ULP_ClearInterrupt((uint16_t)Dev->I2cDevAddr);
			continue_loop = 0;
			printf("Target detected ! Exit low power mode\n\n");
		}
	}while(continue_loop);

	status = VL53L3CX_ULP_StopRanging((uint16_t)Dev->I2cDevAddr);

	return status;
}


/* This function is used to boot the VL53L3CX and start a ranging session using
 * the standard histogram mode. It is used to measure target distances.
 */
int InitAndStartRanging(){
	uint8_t continue_loop = 1;
	uint16_t nb_frames_before_low_power = NB_FRAMES_BEFORE_LOW_POWER;
	VL53LX_MultiRangingData_t MultiRangingData;

	printf("***********************************************\n");
	printf("Starting ranging mode - power consumption 16mA\n");
	printf("***********************************************\n");

	status = VL53LX_WaitDeviceBooted(Dev);
	status = VL53LX_DataInit(Dev);

#ifdef ENABLE_XTALK_CALIBRATION
	 status = VL53LX_SetCalibrationData(Dev, &bare_driver_xtalk_caldata);
#endif

	status = VL53LX_StartMeasurement(Dev);
	if(status)
	{
		printf("VL53LX_StartMeasurement failed: error = %d\n", status);
		return status;
	}

	/* Clear first interrupt (raised when ranging starts) */
	IntCount = 0;

	/* Wait for HW interrupt */
	do
	{
		__WFI();
		if(IntCount !=0 ){
			IntCount=0;
			status = VL53LX_GetMultiRangingData(Dev, &MultiRangingData);

			/* In this example, we consider that the user is not anymore
			 * close to the sensor when :
			 *  - No object is found (user very far from sensor)
			 *  - Or range status is invalid (cannot trust measured distance).
			 *    The example only accepts range status 0 (100% valid) and 6
			 *    (first measurement).
			 *  - Or measured distance is above 1000 mm
			 */

			if((MultiRangingData.NumberOfObjectsFound == 0)
				||((MultiRangingData.RangeData[0].RangeStatus != 0)
						&& (MultiRangingData.RangeData[0].RangeStatus != 6))
				|| (MultiRangingData.RangeData[0].RangeMilliMeter > 1000))
			{
				nb_frames_before_low_power--;
				printf("No object found below 1000mm - sensor will go in low power in %4u frames\n",
						nb_frames_before_low_power);
			}
			else
			{
				nb_frames_before_low_power = NB_FRAMES_BEFORE_LOW_POWER;
				printf("Distance %d mm, Status %u\n",
						MultiRangingData.RangeData[0].RangeMilliMeter,
						MultiRangingData.RangeData[0].RangeStatus);
			}

			/* Exit ranging loop if counter reached */
			if(nb_frames_before_low_power == 0)
			{
				printf("No user detected since a few seconds. Exit ranging mode and go in low power.\n\n");
				continue_loop = 0;
			}
			status = VL53LX_ClearInterruptAndStartMeasurement(Dev);
		}
	}while(continue_loop);

	status = VL53LX_StopMeasurement(Dev);

	return status;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  
  uint8_t byteData;
  uint16_t wordData;
  uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
  
  /* MCU Configuration----------------------------------------------------------*/
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  
  MX_USART2_UART_Init();
  
  MX_I2C1_Init();
  
  XNUCLEO53L3A2_Init();
  
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();

//  /* Configure the system clock */
//  SystemClock_Config();

//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_I2C1_Init();
//  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
#ifdef DistanceMode
  printf("VL53L1X Examples...\n");
  Dev->I2cHandle = &hi2c1;
  Dev->I2cDevAddr = 0x52;
  
  /* Allow to select the sensor to be used, multi-sensor is not managed in this example.
  Only when use use the Left ToF in interrupt mode,  solder the U7 on the X-Nucleo-53L3A2 board 
  Only when use the Right ToF in interrupt mode, solder the U7 on the X-Nucleo-53L3A2 board
  See "Solder drop configurations" in the X-Nucleo-53L3A2 User Manual for more details */
  ToFSensor = 1; // Select ToFSensor: 0=Left, 1=Center, 2=Right
  status = XNUCLEO53L3A2_ResetId(ToFSensor, 0); // Reset ToF sensor
  HAL_Delay(2);
  status = XNUCLEO53L3A2_ResetId(ToFSensor, 1); // Reset ToF sensor
  HAL_Delay(2);
  
  VL53LX_RdByte(Dev, 0x010F, &byteData);
  printf("VL53LX Model_ID: %02X\n\r", byteData);
  VL53LX_RdByte(Dev, 0x0110, &byteData);
  printf("VL53LX Module_Type: %02X\n\r", byteData);
  VL53LX_RdWord(Dev, 0x010F, &wordData);
  printf("VL53LX: %02X\n\r", wordData);
  RangingLoop();
#endif
    
    printf("Starting example with L3CX bare driver and ultra low power\n");

	/* Assign the default address to the sensor */
	Dev->I2cHandle = &hi2c1;
	Dev->I2cDevAddr = 0x52;

	/* Allow to select the sensor to be used, multi-sensor is not managed in this example.
	Only when use use the Left ToF in interrupt mode,  solder the U7 on the X-Nucleo-53L3A2 board
	Only when use the Right ToF in interrupt mode, solder the U7 on the X-Nucleo-53L3A2 board
	See "Solder drop configurations" in the X-Nucleo-53L3A2 User Manual for more details */
	status = XNUCLEO53L3A2_ResetId(1, 0);
	HAL_Delay(2);
	status = XNUCLEO53L3A2_ResetId(1, 1);
	HAL_Delay(2);

	/* Print the software version */
	printf("Ultra low power driver version : %s / Bare driver driver version : %u.%u.%u\n\n",
			VL53L3CX_ULP_VERSION,
			VL53LX_IMPLEMENTATION_VER_MAJOR,
			VL53LX_IMPLEMENTATION_VER_MINOR,
			VL53LX_IMPLEMENTATION_VER_SUB);


#ifdef ENABLE_XTALK_CALIBRATION

	/* If the macro is enabled, run a Xtalk calibration and store the data in a
	 * global variable. For running the calibration, a black5 % target (Full FOV)
	 * needs to be put at at 600mm from the sensor. */

	  status = VL53LX_WaitDeviceBooted(Dev);
	  status = VL53LX_DataInit(Dev);

	  printf("Running Xtalk calibration...\n");
	  status = VL53LX_PerformXTalkCalibration(Dev);
	  if(status){
		  printf("Error during Xtalk calibration : %d\n", status);
	  }

	  memset(&bare_driver_xtalk_caldata, 0, sizeof(bare_driver_xtalk_caldata));
	  status = VL53LX_GetCalibrationData(Dev, &bare_driver_xtalk_caldata);
	  printf("xtalk_cal.algo__crosstalk_compensation_plane_offset_kcps:\t%f\n"
			    "xtalk_cal.algo__crosstalk_compensation_x_plane_gradient_kcps:\t%f\n,"
			    "xtalk_cal.algo__crosstalk_compensation_y_plane_gradient_kcps:\t%f\n",
				bare_driver_xtalk_caldata.customer.algo__crosstalk_compensation_plane_offset_kcps* 1.0 / (1<<9),
				bare_driver_xtalk_caldata.customer.algo__crosstalk_compensation_x_plane_gradient_kcps * 1.0 / (1<<11),
				bare_driver_xtalk_caldata.customer.algo__crosstalk_compensation_y_plane_gradient_kcps * 1.0 / (1<<11));
#endif
    status = 0;
	while(!status)
	{
		status |= InitAndStartLowPower();
		status |= InitAndStartRanging();
	}

	printf("Program exit after error raised\n");
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks
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

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VL53L1X_INT_Pin */
  GPIO_InitStruct.Pin = VL53L1X_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(VL53L1X_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

/* ranging and display loop */
void RangingLoop(void)
{
  VL53LX_MultiRangingData_t MultiRangingData;
  VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
  uint8_t NewDataReady=0;
  int no_of_object_found=0,j;
  printf("Ranging loop starts\n");
  
  status = VL53LX_WaitDeviceBooted(Dev);
  status = VL53LX_DataInit(Dev);
  status = VL53LX_StartMeasurement(Dev);
  
  if(status){
    printf("VL53LX_StartMeasurement failed: error = %d \n", status);
    while(1);
  }
  
  if (isInterrupt){
    do // HW interrupt mode
    {
      __WFI();
      if(IntCount !=0 ){
        IntCount=0;
        status = VL53LX_GetMultiRangingData(Dev, pMultiRangingData);
        no_of_object_found=pMultiRangingData->NumberOfObjectsFound;
        printf("Count=%5d, ", pMultiRangingData->StreamCount);
        printf("#Objs=%1d ", no_of_object_found);
        printf("Xtalk=%5d ", pMultiRangingData->HasXtalkValueChanged);
        printf("Spad=%5d ", pMultiRangingData->EffectiveSpadRtnCount);
        for(j=0;j<no_of_object_found;j++){
          if(j!=0)printf("\n                     ");
          printf("Max=%d, Min=%d, Signal=%2.2f Mcps, Ambient=%2.2f Mcps, Sigma=%2.2f mm, D=%5dmm, status=%d, ER=%d",
                 pMultiRangingData->RangeData[j].RangeMaxMilliMeter,
                 pMultiRangingData->RangeData[j].RangeMinMilliMeter,
                 pMultiRangingData->RangeData[j].SignalRateRtnMegaCps/65536.0,
                 pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps/65536.0,
                 pMultiRangingData->RangeData[j].SigmaMilliMeter/65536.0,
                 pMultiRangingData->RangeData[j].RangeMilliMeter,
                 pMultiRangingData->RangeData[j].RangeStatus,
                 pMultiRangingData->RangeData[j].ExtendedRange);
        }
        printf ("\n");
        if (status==0){
          status = VL53LX_ClearInterruptAndStartMeasurement(Dev);
        }
        
      }			 
    }
    while(1);
  }
  else{
    do{ // polling mode
      status = VL53LX_GetMeasurementDataReady(Dev, &NewDataReady);                        
      HAL_Delay(1); // 1 ms polling period, could be longer.
      if((!status)&&(NewDataReady!=0)){
        status = VL53LX_GetMultiRangingData(Dev, pMultiRangingData);
        no_of_object_found=pMultiRangingData->NumberOfObjectsFound;
        printf("Count=%5d, ", pMultiRangingData->StreamCount);
        printf("#Objs=%1d ", no_of_object_found);
        for(j=0;j<no_of_object_found;j++){
          if(j!=0)printf("\n                     ");
          printf("status=%d, D=%5dmm, Signal=%2.2f Mcps, Ambient=%2.2f Mcps",
                 pMultiRangingData->RangeData[j].RangeStatus,
                 pMultiRangingData->RangeData[j].RangeMilliMeter,
                 pMultiRangingData->RangeData[j].SignalRateRtnMegaCps/65536.0,
                 pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps/65536.0);
        }
        printf ("\n");
        if (status==0){
          status = VL53LX_ClearInterruptAndStartMeasurement(Dev);
        }
      }
    }
    while (1);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
