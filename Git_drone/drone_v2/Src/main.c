/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 26/10/2014 01:42:54
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

#include "MPU6050.h"
#include <math.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

#define header_uart 0x7E

const double ACCELEROMETER_SENSITIVITY =8192.0;
const double GYROSCOPE_SENSITIVITY =65.536;
const double M_PI =3.14159265359	;    
const double dt = 0.004				;			// 250 hz sample rate!   


#define motor_limmit		100

double Kp_yaw		=2.2;
double Ki_yaw		=0.5;
double Kd_yaw		=0.75;

double Kp_pitch	 =2.2;
double Ki_pitch  =0.5;
double Kd_pitch  =0.75;

double Kp_roll		=2.2;
double Ki_roll  	=0.5;
double Kd_roll  	=0.75;

double prescal=50 ;

/* START USER CODE for SPPM Receiver  */

uint16_t Channel[10]={0,0,0,0,0,0,0,0,0,0} ;
int8_t Ch_count = 0 ;
uint16_t tmp_Ch =0 ;
int16_t ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,ch9 ;

/* END USER CODE for SPPM Receiver  */


/*  Debug*/

const uint8_t h_uart = 0x7E;

uint8_t buf_uart[10]={0};																			// buffer uart

int16_t  AccelGyro[6]={0};                                       	// RAW states value

float yaw=0, pitch=0, roll=0;																		// States value

int16_t motor_A=0, motor_B=0, motor_C=0, motor_D=0 ;							// Motors output value 

int16_t Ref_yaw=0, Ref_pitch=0, Ref_roll=0 ,T_centers= 0;	

int8_t Ref_yaw_H=0, Ref_pitch_H=0, Ref_roll_H=0 ,T_center_H = 0;											// referent input
int8_t Ref_yaw_L=0, Ref_pitch_L=0, Ref_roll_L=0 ,T_center_L = 0;	
int8_t enable=0,buf_enable=0 ;																																// enable

float T_center =0, yaw_center=0;

double Error_yaw=0, Errer_pitch=0, Error_roll=0; 												//States Error

double Sum_Error_yaw=0, Sum_Error_pitch=0, Sum_Error_roll=0;             // Sum of error

double D_Error_yaw=0, D_Error_pitch=0, D_Error_roll=0; 									// Diff of error
double Buf_D_Error_yaw=0, Buf_D_Errer_pitch=0, Buf_D_Error_roll=0; 


double Del_yaw=0, Del_pitch=0, Del_roll=0;												// Delta states value for rotate axis

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */

void Initial_MPU6050(void);
void read_mpu6050(void);
void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
void MPU6050_GetRawAccelGyro(int16_t* AccelGyro);
void ComplementaryFilter(int16_t AccelGyro[6], float *pitch, float *roll, float *yaw);
volatile void Controller(void);
volatile void Read_Angle(void);
volatile void PID_controller(void);
volatile void Drive_motor_output(void);
volatile void Read_serial(void);
volatile void Interrupt_call(void);


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN 3 */
	HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);	
	HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);


	Initial_MPU6050();
	
  HAL_TIM_Base_Start(&htim16);
	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
	
	
	HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
	HAL_Delay(300);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* Infinite loop */
  while (1)
  {

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  __SYSCFG_CLK_ENABLE();

}

/* ADC init function */
void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
  hadc.Init.Resolution = ADC_RESOLUTION12b;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = ENABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = OVR_DATA_PRESERVED;
  HAL_ADC_Init(&hadc);

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 104;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

    /**Configure Analogue filter 
    */
  HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2399;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

}

/* TIM14 init function */
void MX_TIM14_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 2399;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim14);

  HAL_TIM_PWM_Init(&htim14);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1);

}

/* TIM16 init function */
void MX_TIM16_Init(void)
{

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 47;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 0xffff;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim16);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  huart1.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PF0 PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */
void Initial_MPU6050(void)
	{

				//    Reset to defalt 
			MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, ENABLE);
				//	  SetClockSource(MPU6050_CLOCK_PLL_XGYRO)
			MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);	
				//    SetFullScaleGyroRange(MPU6050_GYRO_FS_250)
			MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_250);
				//    SetFullScaleAccelRange(MPU6050_ACCEL_FS_2)
			MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_ACCEL_FS_2);
				//    interupt(Enable)
			MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, ENABLE);
			 //    SetSleepModeStatus(DISABLE)
			MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, DISABLE);
			//			SetDLPF(MPU6050_DLPF_BW_5)
			MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_10);
			
			MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, DISABLE);
				
			HAL_Delay(50); // for stability
}

void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t tmp;
		HAL_I2C_Mem_Read(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing byte
    tmp |= data; // combine data with existing byte
    HAL_I2C_Mem_Write(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
}

void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t tmp;
		HAL_I2C_Mem_Read(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    HAL_I2C_Mem_Write(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
}

void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t tmp;
		HAL_I2C_Mem_Read(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    tmp &= mask;
    tmp >>= (bitStart - length + 1);
    *data = tmp;
}

void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
    uint8_t tmp;
		HAL_I2C_Mem_Read(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
    *data = tmp & (1 << bitNum);
}
void MPU6050_GetRawAccelGyro(int16_t* AccelGyro)
{
    uint8_t tmpBuffer[14];
	  HAL_I2C_Mem_Read(&hi2c1,MPU6050_DEFAULT_ADDRESS,MPU6050_RA_ACCEL_XOUT_H,1,tmpBuffer,14,1);
    /* Get acceleration */
    for (int i = 0; i < 3; i++)
        AccelGyro[i] = ((int16_t) ((uint16_t) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);
    /* Get Angular rate */
    for (int i = 4; i < 7; i++)
        AccelGyro[i - 1] = ((int16_t) ((uint16_t) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);

}

volatile void Read_Angle(void)
{
	MPU6050_GetRawAccelGyro(AccelGyro);
	ComplementaryFilter( AccelGyro, &pitch, &roll, &yaw);
}

void ComplementaryFilter(int16_t AccelGyro[6], float *pitch, float *roll, float *yaw)
{
    float pitchAcc, rollAcc;               
 
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *pitch -= ((float)AccelGyro[3] / GYROSCOPE_SENSITIVITY) * dt; 	// Angle around the X-axis
    *roll  -= ((float)AccelGyro[4] / GYROSCOPE_SENSITIVITY) * dt;   // Angle around the Y-axis
		*yaw   +=	((float)AccelGyro[5] / GYROSCOPE_SENSITIVITY) * dt;   // Angle around the Z-axis

    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int16_t forceMagnitudeApprox = abs(AccelGyro[0]) + abs(AccelGyro[1]) + abs(AccelGyro[2]);
    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
	// Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2((float)AccelGyro[1], (float)AccelGyro[2]) * 180 / M_PI;
        *pitch = *pitch * 0.96 - pitchAcc * 0.04 ;
 
	// Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2((float)AccelGyro[0], (float)AccelGyro[2]) * 180 / M_PI;
        *roll = *roll * 0.96 + rollAcc * 0.04 ;
    }
} 


volatile void PID_controller(void)
{

	T_center   = ch3*3  ;
	yaw_center +=(ch4/5)*dt     ;
	
	
	Error_yaw 	= yaw_center -yaw	;
	Errer_pitch = (ch2/30)	-pitch	;
	Error_roll 	= (ch1/30)	-roll	;
	
	Sum_Error_yaw 	+= Error_yaw*dt ;
	Sum_Error_pitch += Errer_pitch*dt ;
	Sum_Error_roll 	+= Error_roll*dt ;
	
	if(Sum_Error_yaw>50)Sum_Error_yaw =50;
	if(Sum_Error_yaw<-50)Sum_Error_yaw =-50;
	if(Sum_Error_pitch>50)Sum_Error_pitch =50;
	if(Sum_Error_pitch<-50)Sum_Error_pitch =-50;
	if(Sum_Error_roll>50)Sum_Error_roll =50;
	if(Sum_Error_roll<-50)Sum_Error_roll =-50;
	
	D_Error_yaw =  (Error_yaw-Buf_D_Error_yaw)/dt ;
	D_Error_pitch =(Errer_pitch-Buf_D_Errer_pitch)/dt ;
	D_Error_roll = (Error_roll-Buf_D_Error_roll)/dt ;
	
	
	Buf_D_Error_yaw =Error_yaw;
	Buf_D_Errer_pitch=Errer_pitch;
	Buf_D_Error_roll=Error_roll; 
	

	
	Del_yaw		= (Kp_yaw   * Error_yaw)		+ (Ki_yaw		* Sum_Error_yaw)   + (Kd_yaw * D_Error_yaw) ;
	Del_pitch	= (Kp_pitch * Errer_pitch)	+ (Ki_pitch	* Sum_Error_pitch) + (Kd_pitch * D_Error_pitch) ;
	Del_roll	= (Kp_roll  * Error_roll)		+ (Ki_roll	* Sum_Error_roll)  + (Kd_roll * D_Error_roll) ;

	motor_A=	T_center +Del_pitch	-Del_roll -Del_yaw ;
	motor_B=	T_center +Del_pitch	+Del_roll +Del_yaw ;
	motor_C=	T_center -Del_pitch	+Del_roll -Del_yaw ;
	motor_D=	T_center -Del_pitch	-Del_roll +Del_yaw ;
	
	
	if(motor_A < 1) motor_A = 0 ;
	if(motor_B < 1) motor_B = 0 ;
	if(motor_C < 1) motor_C = 0 ;
	if(motor_D < 1) motor_D = 0 ;
	
	if(motor_A > 2399) motor_A = 2399 ;
	if(motor_B > 2399) motor_B = 2399 ;
	if(motor_C > 2399) motor_C = 2399 ;
	if(motor_D > 2399) motor_D = 2399 ;
	
}

volatile void Drive_motor_output(void)
{

	TIM3 ->CCR1 = motor_D ;
	TIM3 ->CCR2 = motor_A ;
	TIM3 ->CCR4 = motor_C ;
	TIM14->CCR1 = motor_B ;
}
volatile void Interrupt_call(void)
{
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_SET);
		HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_0);    
	
		/* Read data from sensor */
	  Read_Angle();
	
		/* Controller */
		 PID_controller();
		if(ch5 > 200){
			
			Drive_motor_output();
		}else{
			Sum_Error_yaw=0;
			Sum_Error_pitch=0;
			Sum_Error_roll=0;             
			motor_A=0;
			motor_B=0;
			motor_C=0;
			motor_D=0;
			T_center=0;
			yaw_center=0;
			yaw =0 ;
			Drive_motor_output();
		}
		
		
	  ch1=Channel[1]- 1510 ;
		ch2=Channel[2]- 1510 ;
		ch3=Channel[3]- 1110 ;
		ch4=Channel[4]- 1510 ;
		ch5=Channel[5]- 1510 ;
		ch6=Channel[6]- 1110 ;
		ch7=Channel[7]- 1110 ;
		ch8=Channel[8]- 1110 ;
		ch9=Channel[9]- 1110 ;
	
		
		if (ch6<0) ch6=0;
		if (ch7<0) ch7=0;
		if (ch8<0) ch8=0;
		
		
		
		
//		 Kp_pitch		= ch6/prescal;
//		 Ki_pitch		= ch7/prescal;
//		 Kd_pitch		= ch8/prescal;
		 
		 
//			 Kp_roll		=ch6/prescal;
//			 Ki_roll  	=ch7/prescal;
//			 Kd_roll  	=ch8/prescal;
		
//			 Kp_yaw		=ch6/prescal;
//			 Ki_yaw		=ch7/prescal;
//			 Kd_yaw		=ch8/prescal;
	 
//	  /* Sent & eceive data from Bluetooth serial */
//		HAL_UART_Transmit(&huart1,(uint8_t*)&h_uart,1,1);
//		HAL_UART_Receive_IT(&huart1,buf_uart,10);
		


	 


	
	
	
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_RESET);

}
volatile void Read_SPPM(void){
	
	Ch_count++;
	tmp_Ch = TIM16->CNT ;
	TIM16->CNT = 0 ;
	if (tmp_Ch > 3000)Ch_count = 0;
	Channel[Ch_count] = tmp_Ch ;

}


/* USER CODE END 4 */

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
