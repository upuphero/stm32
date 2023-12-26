/**
******************************************************************************
* File Name          : main.c
* Description        : Main program body
******************************************************************************
*
* COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32xxx_hal.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "VL53L1X_API.h"
#include "VL53l1X_calibration.h"
#include "X-NUCLEO-53L1A1.h"
/* USER CODE END Includes */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
uint16_t	dev=0x52;
int status = 0;
volatile int IntCount;
#define isInterrupt 1 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int SystemClock_Config(void);
static void MX_GPIO_Init(void);
static int MX_USART2_UART_Init(void);
static int MX_I2C1_Init(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

// People Counting defines
#define NOBODY                    0
#define SOMEONE                   1
#define LEFT                      0
#define RIGHT                     1

#define DOOR_JAM_2400             1
#define DOOR_JAM_2000             2
#define ON_SIDE                   3

#define DISTANCE_MODE_LONG        2


// define here the profile for your case.
#define PPC_PROFILE DOOR_JAM_2400

#define TRACE_PPC				     1

// PPC_PROFILE DOOR_JAM_2400
#if PPC_PROFILE == DOOR_JAM_2400
#define PROFILE_STRING                               "DOOR_JAM_2400"
#define DISTANCES_ARRAY_SIZE                         10   // nb of samples
#define MAX_DISTANCE                                 2400 // mm
#define MIN_DISTANCE                                 0   // mm
#define DIST_THRESHOLD                               1600  // mm
#define ROWS_OF_SPADS                                8 // 8x16 SPADs ROI
#define TIMING_BUDGET                                33  // was 20 ms, I found 33 ms has better succes rate with lower reflectance target
#define DISTANCE_MODE                                DISTANCE_MODE_LONG
#endif

#if ROWS_OF_SPADS == 4
#define FRONT_ZONE_CENTER                            151
#define BACK_ZONE_CENTER                             247
#elif ROWS_OF_SPADS == 6
#define FRONT_ZONE_CENTER                            159
#define BACK_ZONE_CENTER                             239
#elif ROWS_OF_SPADS == 8
#define FRONT_ZONE_CENTER                            175 // was 167, see UM2555 on st.com, centre = 175 has better return signal rate for the ROI #1
#define BACK_ZONE_CENTER                             231 
#endif

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

/* USER CODE END 0 */

int ProcessPeopleCountingData(int16_t Distance, uint8_t zone, uint8_t RangeStatus) {
  static int PathTrack[] = {0,0,0,0};
  static int PathTrackFillingSize = 1; // init this to 1 as we start from state where nobody is any of the zones
  static int LeftPreviousStatus = NOBODY;
  static int RightPreviousStatus = NOBODY;
  static int PeopleCount = 0;
  static uint16_t Distances[2][DISTANCES_ARRAY_SIZE];
  static uint8_t DistancesTableSize[2] = {0,0};
  
  uint16_t MinDistance;
  uint8_t i;

#ifdef TRACE_PPC
#define TIMES_WITH_NO_EVENT 10// was 40
  static uint32_t trace_count = TIMES_WITH_NO_EVENT;  // replace by 0 if you want to trace the first TIMES_WITH_NO_EVENT values
#endif
  
  int CurrentZoneStatus = NOBODY;
  int AllZonesCurrentStatus = 0;
  int AnEventHasOccured = 0;
  
  // Add just picked distance to the table of the corresponding zone
  if (DistancesTableSize[zone] < DISTANCES_ARRAY_SIZE) {
    Distances[zone][DistancesTableSize[zone]] = Distance;
    DistancesTableSize[zone] ++;
  }
  else {
    for (i=1; i<DISTANCES_ARRAY_SIZE; i++)
      Distances[zone][i-1] = Distances[zone][i];
    Distances[zone][DISTANCES_ARRAY_SIZE-1] = Distance;
  }
  
  // pick up the min distance
  MinDistance = Distances[zone][0];
  if (DistancesTableSize[zone] >= 2) {
    for (i=1; i<DistancesTableSize[zone]; i++) {
      if (Distances[zone][i] < MinDistance)
        MinDistance = Distances[zone][i];
    }
  }
  
  if (MinDistance < DIST_THRESHOLD) {
    // Someone is in !
    CurrentZoneStatus = SOMEONE;
  }
  
  // left zone
  if (zone == LEFT) {
    if (CurrentZoneStatus != LeftPreviousStatus) {
      // event in left zone has occured
      AnEventHasOccured = 1;
      
      if (CurrentZoneStatus == SOMEONE) {
        AllZonesCurrentStatus += 1;
      }
      // need to check right zone as well ...
      if (RightPreviousStatus == SOMEONE) {
        // event in left zone has occured
        AllZonesCurrentStatus += 2;
      }
      // remember for next time
      LeftPreviousStatus = CurrentZoneStatus;
    }
  }
  // right zone
  else {
    
    if (CurrentZoneStatus != RightPreviousStatus) {
      
      // event in left zone has occured
      AnEventHasOccured = 1;
      if (CurrentZoneStatus == SOMEONE) {
        AllZonesCurrentStatus += 2;
      }
      // need to left right zone as well ...
      if (LeftPreviousStatus == SOMEONE) {
        // event in left zone has occured
        AllZonesCurrentStatus += 1;
      }
      // remember for next time
      RightPreviousStatus = CurrentZoneStatus;
    }
  }
  
#ifdef TRACE_PPC
  // print debug data only when someone is within the field of view
  trace_count++;
  if ((CurrentZoneStatus == SOMEONE) || (LeftPreviousStatus == SOMEONE) || (RightPreviousStatus == SOMEONE))
    trace_count = 0;
  
  if (trace_count < TIMES_WITH_NO_EVENT)
//    printf ("%d,%d,%d,%d,%d\n", zone, Distance, MinDistance, RangeStatus, PeopleCount);
#endif
  
  // if an event has occured
  if (AnEventHasOccured) {
    if (PathTrackFillingSize < 4) {
      PathTrackFillingSize ++;
    }
    
    // if nobody anywhere lets check if an exit or entry has happened
    if ((LeftPreviousStatus == NOBODY) && (RightPreviousStatus == NOBODY)) {
      
      // check exit or entry only if PathTrackFillingSize is 4 (for example 0 1 3 2) and last event is 0 (nobobdy anywhere)
      if (PathTrackFillingSize == 4) {
        // check exit or entry. no need to check PathTrack[0] == 0 , it is always the case
        
        if ((PathTrack[1] == 1)  && (PathTrack[2] == 3) && (PathTrack[3] == 2)) {
          // This an entry
          PeopleCount ++;
          // reset the table filling size in case an entry or exit just found
          DistancesTableSize[0] = 0;
          DistancesTableSize[1] = 0;
          printf("Walk In, People Count=%d\n", PeopleCount);
        } else if ((PathTrack[1] == 2)  && (PathTrack[2] == 3) && (PathTrack[3] == 1)) {
          // This an exit
          PeopleCount --;
          // reset the table filling size in case an entry or exit just found
          DistancesTableSize[0] = 0;
          DistancesTableSize[1] = 0;
          printf("Walk Out, People Count=%d\n", PeopleCount);
        } else {
          // reset the table filling size also in case of unexpected path
          DistancesTableSize[0] = 0;
          DistancesTableSize[1] = 0;
          printf("Wrong path\n");
        }
      }
      
      PathTrackFillingSize = 1;
    }
    else {
      // update PathTrack
      // example of PathTrack update
      // 0
      // 0 1
      // 0 1 3
      // 0 1 3 1
      // 0 1 3 3
      // 0 1 3 2 ==> if next is 0 : check if exit
      PathTrack[PathTrackFillingSize-1] = AllZonesCurrentStatus;
    }
    
//#ifdef TRACE_PPC
//    if (AnEventHasOccured) {
//      for (int j=0; j<PathTrackFillingSize; j++)
//        printf ("%d ", PathTrack[j]);
//    }
//    printf("\n");
//#endif
  }
  
  // output debug data to main host machine
  return(PeopleCount);
}

int main(void)
{
  int8_t error;
  uint8_t byteData, sensorState=0;
  uint16_t wordData;
  uint16_t Distance, Signal;
  uint8_t RangeStatus;
  uint8_t dataReady;
  char DisplayStr[5];
  int PplCounter;
  int center[2] = {FRONT_ZONE_CENTER, BACK_ZONE_CENTER}; /* these are the spad center of the 2 4*16 zones */
  int Zone = 0;
  /* int PplCounter = 0;*/
  
  /* MCU Configuration----------------------------------------------------------*/
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* Configure the system clock */
  error = SystemClock_Config();
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  error += MX_USART2_UART_Init();
  error += MX_I2C1_Init();
  
  if (error != 0)
  {
    printf("Could not initialize the nucleo board\n");
    return -1;
  }
  
  status = XNUCLEO53L1A1_Init();
  printf("XNUCLEO53L1A1_Init Status : %X\n", status); 
  
  status = XNUCLEO53L1A1_ResetId(XNUCLEO53L1A1_DEV_LEFT, 0); // Reset ToF sensor
  HAL_Delay(2);
  status = XNUCLEO53L1A1_ResetId(XNUCLEO53L1A1_DEV_CENTER, 0); // Reset ToF sensor
  HAL_Delay(2);
  status = XNUCLEO53L1A1_ResetId(XNUCLEO53L1A1_DEV_RIGHT, 0); // Reset ToF sensor
  HAL_Delay(2);
  status = XNUCLEO53L1A1_ResetId(XNUCLEO53L1A1_DEV_CENTER, 1); // Reset ToF sensor
  HAL_Delay(2);
  
  // Those basic I2C read functions can be used to check your own I2C functions */
  status = VL53L1_RdByte(dev, 0x010F, &byteData);
  printf("VL53L1X Model_ID: %X\n", byteData);
  status = VL53L1_RdByte(dev, 0x0110, &byteData);
  printf("VL53L1X Module_Type: %X\n", byteData);
  status = VL53L1_RdWord(dev, 0x010F, &wordData);
  printf("VL53L1X: %X\n", wordData);
  while (sensorState == 0) {
    status = VL53L1X_BootState(dev, &sensorState);
    HAL_Delay(2);
  }
  printf("Chip booted\n");
  
  /* Initialize and configure the device according to people counting need */
  status = VL53L1X_SensorInit(dev);
  status += VL53L1X_SetDistanceMode(dev, DISTANCE_MODE); /* 1=short, 2=long */
  status += VL53L1X_SetTimingBudgetInMs(dev, TIMING_BUDGET); /* in ms possible values [15, 20, 50, 100, 200, 500] */
  status += VL53L1X_SetInterMeasurementInMs(dev, TIMING_BUDGET);
  status += VL53L1X_SetROI(dev, ROWS_OF_SPADS, 16); /* minimum ROI 4,4 */
  if (status != 0) {
    printf("Initialization or configuration of the device\n");
    return (-1);
  }
  printf("Start counting people with profile : %s...\n", PROFILE_STRING);
  status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
  while(1) { /* read and display data */
    while (dataReady == 0) {
      status = VL53L1X_CheckForDataReady(dev, &dataReady);
      HAL_Delay(1);
    }
    dataReady = 0;
    status += VL53L1X_GetRangeStatus(dev, &RangeStatus);
    status += VL53L1X_GetDistance(dev, &Distance);
    status += VL53L1X_GetSignalPerSpad(dev, &Signal);
    status += VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
    if (status != 0) {
      printf("Error in operating the device\n");
      return (-1);
    }  
    //HAL_Delay(WAIT_BEFORE_PROGRAMMING_OTHER_ZONE_CENTER);  // 10, 8, 7, 6 tested OK
    status = VL53L1X_SetROICenter(dev, center[Zone]);
    if (status != 0) {
      printf("Error in chaning the center of the ROI\n");
      return (-1);
    }
    // check the status of the ranging. In case of error, lets assume the distance is the max of the use case
    // Value RangeStatus string Comment
    // 0 VL53L1_RANGESTATUS_RANGE_VALID Ranging measurement is valid
    // 1 VL53L1_RANGESTATUS_SIGMA_FAIL Raised if sigma estimator check is above the internal defined threshold
    // 2 VL53L1_RANGESTATUS_SIGNAL_FAIL Raised if signal value is below the internal defined threshold
    // 4 VL53L1_RANGESTATUS_OUTOFBOUNDS_ FAIL Raised when phase is out of bounds
    // 5 VL53L1_RANGESTATUS_HARDWARE_FAIL Raised in case of HW or VCSEL failure
    // 7 VL53L1_RANGESTATUS_WRAP_TARGET_ FAIL Wrapped target, not matching phases
    // 8 VL53L1_RANGESTATUS_PROCESSING_ FAIL Internal algorithm underflow or overflow
    // 14 VL53L1_RANGESTATUS_RANGE_INVALID The reported range is invalid
    if ((RangeStatus == 0) || (RangeStatus == 4) || (RangeStatus == 7)) {
      if (Distance <= MIN_DISTANCE) // wraparound case see the explanation at the constants definition place
        Distance = MAX_DISTANCE + MIN_DISTANCE;
    }
    else // severe error cases
      Distance = MAX_DISTANCE;
    // inject the new ranged distance in the people counting algorithm
    PplCounter = ProcessPeopleCountingData(Distance, Zone, RangeStatus);
    sprintf(DisplayStr, "%4d", PplCounter); // only use for special EVK with display
    XNUCLEO53L1A1_SetDisplayString(DisplayStr);
    printf("%d,%d,%d\n", Zone, Distance, Signal);
    Zone++;
    Zone = Zone%2;
  }
}

/** System Clock Configuration
*/
#ifdef STM32F401xE

int SystemClock_Config(void)
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
    return -1;
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
    return -1;
  }
  
  /**Configure the Systick interrupt time 
  */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  
  /**Configure the Systick 
  */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
  
  return 0;
}
#endif


#ifdef STM32F401xE

/* I2C1 init function */
static int MX_I2C1_Init(void)
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
    return -1;
  }
  
  return 0;
  
}
#endif

/* USART2 init function */
int MX_USART2_UART_Init(void)
{
  
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    return (-1);
  }
  return 0;
  
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


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
