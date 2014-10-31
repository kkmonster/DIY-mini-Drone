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

#define beta                        0.1
#define ACCELEROMETER_SENSITIVITY   16384   
#define GYROSCOPE_SENSITIVITY       131.07  
#define M_PI                        3.14159265359	    
#define sampleFreq                  250 			    // 250 hz sample rate!   
#define limmit_I                    300
#define battary_low_level           2961                // 1v = ~846   @ 3.5 v = 2961
float Kp_yaw    =2.2;
float Ki_yaw    =0.5;
float Kd_yaw    =0.75;

float Kp_pitch	=2.2;
float Ki_pitch  =0.5;
float Kd_pitch  =0.75;

float Kp_roll	=2.2;
float Ki_roll  	=0.5;
float Kd_roll  	=0.75;

float Ref_yaw=0, Ref_pitch=0, Ref_roll=0 ,T_centers= 0;	
float q_yaw, q_pitch, q_roll;                                       // States value
float q1=1, q2=0, q3=0, q4=0 ;
float gx_diff = 0, gy_diff=0, gz_diff=0;
float start_pitch=0, start_roll=0;
float T_center =0, yaw_center=0;
float Error_yaw=0, Errer_pitch=0, Error_roll=0; 						//States Error
float Sum_Error_yaw=0, Sum_Error_pitch=0, Sum_Error_roll=0;             // Sum of error
float D_Error_yaw=0, D_Error_pitch=0, D_Error_roll=0; 					// error dot
float Del_yaw=0, Del_pitch=0, Del_roll=0;												// Delta states value for rotate axis

uint16_t battery_voltage ;
float T_center_minus = 0;
/* USER CODE for SPPM Receiver  */

uint16_t    tmp_Ch =0 ;
uint16_t    Channel[10]={0} ;
int16_t     ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,ch9 ;
int8_t      Ch_count = 0 ;
uint8_t     buf_uart[10]={0};	     // buffer uart
int16_t     AccelGyro[6]={0};       // RAW states value
int16_t     motor_A=0, motor_B=0, motor_C=0, motor_D=0 ;// Motors output value 

//int8_t bt_Ref_yaw_L=0, bt_Ref_pitch_L=0, bt_Ref_roll_L=0 ,bt_T_center_L = 0;	// data from bluetooth
																															// enable


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
volatile void Controller(void);
volatile void PID_controller(void);
volatile void Drive_motor_output(void);
volatile void Interrupt_call(void);
volatile void ahrs(void);
//void ComplementaryFilter(int16_t AccelGyro[6], float *pitch, float *roll, float *yaw);
//volatile void Read_Angle(void);

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
	
	HAL_Delay(1000);
	
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);
	uint8_t xxx = 30;
	while (xxx > 0){
		MPU6050_GetRawAccelGyro(AccelGyro);

		gx_diff += AccelGyro[3];
		gy_diff += AccelGyro[4];
		gz_diff += AccelGyro[5];

        xxx--;
		HAL_Delay(50);
	}
		gx_diff /= 30;
		gy_diff /= 30;
		gz_diff /= 30;
	
	HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);  // interrupt from imu
	
    HAL_Delay(300); // read start angle
    
    start_pitch = q_pitch ; 
    start_roll  = q_roll  ;
    
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn); // interrupt from sppm

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
			MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_98);
			
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

volatile void PID_controller(void)
{

    float Buf_D_Error_yaw =Error_yaw;
	float Buf_D_Errer_pitch=Errer_pitch;
	float Buf_D_Error_roll=Error_roll; 
    
	T_center    = ch3*3 - T_center_minus  ;
	yaw_center +=(ch4/5)/sampleFreq     ;

	Error_yaw 	= yaw_center - q_yaw	;
	Errer_pitch = start_pitch + (ch2/30)-q_pitch	;
	Error_roll 	= start_roll  + (ch1/30)-q_roll	;
	
	Sum_Error_yaw 	+= Error_yaw   /sampleFreq ;
	Sum_Error_pitch += Errer_pitch /sampleFreq ;
	Sum_Error_roll 	+= Error_roll  /sampleFreq ;
	
    // protect 
	if(Sum_Error_yaw>limmit_I)Sum_Error_yaw =limmit_I;
	if(Sum_Error_yaw<-limmit_I)Sum_Error_yaw =-limmit_I;
	if(Sum_Error_pitch>limmit_I)Sum_Error_pitch =limmit_I;
	if(Sum_Error_pitch<-limmit_I)Sum_Error_pitch =-limmit_I;
	if(Sum_Error_roll>limmit_I)Sum_Error_roll =limmit_I;
	if(Sum_Error_roll<-limmit_I)Sum_Error_roll =-limmit_I;
	
	D_Error_yaw =  (Error_yaw-Buf_D_Error_yaw)    *sampleFreq ;
	D_Error_pitch =(Errer_pitch-Buf_D_Errer_pitch)*sampleFreq ;
	D_Error_roll = (Error_roll-Buf_D_Error_roll)  *sampleFreq ;

	Buf_D_Error_yaw =Error_yaw;
	Buf_D_Errer_pitch=Errer_pitch;
	Buf_D_Error_roll=Error_roll; 

	Del_yaw		= (Kp_yaw   * Error_yaw)		+ (Ki_yaw		* Sum_Error_yaw)   + (Kd_yaw * D_Error_yaw) ;
	Del_pitch	= (Kp_pitch * Errer_pitch)	    + (Ki_pitch	* Sum_Error_pitch)     + (Kd_pitch * D_Error_pitch) ;
	Del_roll	= (Kp_roll  * Error_roll)		+ (Ki_roll	* Sum_Error_roll)      + (Kd_roll * D_Error_roll) ;

	motor_A=	T_center +Del_pitch	-Del_roll -Del_yaw ;
	motor_B=	T_center +Del_pitch	+Del_roll +Del_yaw ;
	motor_C=	T_center -Del_pitch	+Del_roll -Del_yaw ;
	motor_D=	T_center -Del_pitch	-Del_roll +Del_yaw ;
	
	// limmit output max, min
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
		MPU6050_GetRawAccelGyro(AccelGyro);
	
		ahrs();
	
		/* Controller */
		PID_controller();
    
		if(ch3 > 50){
			
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
			Drive_motor_output();
		}
		
		// update sppm read
        
	    ch1=Channel[1]- 1510 ;
		ch2=Channel[2]- 1510 ;
		ch3=Channel[3]- 1110 ;
		ch4=Channel[4]- 1510 ;
		ch5=Channel[5]- 1510 ;
		ch6=Channel[6]- 1110 ;
		ch7=Channel[7]- 1110 ;
		ch8=Channel[8]- 1110 ;
		ch9=Channel[9]- 1110 ;

        
        
        
//	  /* Sent & eceive data from Bluetooth serial */
//		HAL_UART_Receive_IT(&huart1,buf_uart,10);
        
    // read battery voltage
    
	battery_voltage = HAL_ADC_GetValue(&hadc);
    T_center_minus -= 0.5 ; 
    if( battery_voltage > battary_low_level )   T_center_minus = 0; 
    HAL_ADC_Start(&hadc);
	
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_RESET);

}
volatile void Read_SPPM(void)
{
	Ch_count++;
	tmp_Ch = TIM16->CNT ;
	TIM16->CNT = 0 ;
	if (tmp_Ch > 3000)Ch_count = 0;
	Channel[Ch_count] = tmp_Ch ;

}
volatile void ahrs(void)
{
	// quaternion base process 
	float Norm;
	float ax = AccelGyro[0];
	float ay = AccelGyro[1];
	float az = AccelGyro[2];
	float gx =((AccelGyro[3]-gx_diff)/ GYROSCOPE_SENSITIVITY )*M_PI/180 ;
	float gy =((AccelGyro[4]-gy_diff)/ GYROSCOPE_SENSITIVITY )*M_PI/180 ;
	float gz =((AccelGyro[5]-gz_diff)/ GYROSCOPE_SENSITIVITY )*M_PI/180 ;
	
	float q1_dot = 0.5 * (-q2 * gx - q3 * gy - q4 * gz);
	float q2_dot = 0.5 * ( q1 * gx + q3 * gz - q4 * gy);
	float q3_dot = 0.5 * ( q1 * gy - q2 * gz + q4 * gx);
	float q4_dot = 0.5 * ( q1 * gz + q2 * gy - q3 * gx);

	if(!((ax == 0) && (ay == 0) && (az == 0))) {
		// Normalise 
		Norm = sqrt(ax * ax + ay * ay + az * az);
		ax /= Norm;
		ay /= Norm;
		az /= Norm;   

		float _2q1 = 2 * q1;
		float _2q2 = 2 * q2;
		float _2q3 = 2 * q3;
		float _2q4 = 2 * q4;
		float _4q1 = 4 * q1;
		float _4q2 = 4 * q2;
		float _4q3 = 4 * q3;
		float _8q2 = 8 * q2;
		float _8q3 = 8 * q3;
		float q1q1 = q1 * q1;
		float q2q2 = q2 * q2;
		float q3q3 = q3 * q3;
		float q4q4 = q4 * q4;
		// Gradient decent 
		float s1 = _4q1 * q3q3 + _2q4 * ax + _4q1 * q2q2 - _2q2 * ay;
		float s2 = _4q2 * q4q4 - _2q4 * ax + 4 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
		float s3 = 4 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
		float s4 = 4 * q2q2 * q4 - _2q2 * ax + 4 * q3q3 * q4 - _2q3 * ay;
		// Normalise 
		Norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
		s1 /= Norm;
		s2 /= Norm;
		s3 /= Norm;
		s4 /= Norm;
		// compensate acc
		q1_dot -= (beta * s1);
		q2_dot -= (beta * s2);
		q3_dot -= (beta * s3);
		q4_dot -= (beta * s4);
	}
	// Integrate 
	q1 += q1_dot / sampleFreq;
	q2 += q2_dot / sampleFreq;
	q3 += q3_dot / sampleFreq;
	q4 += q4_dot / sampleFreq;
	// Normalise
	Norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4 );
	q1 /= Norm;
	q2 /= Norm;
	q3 /= Norm;
	q4 /= Norm;
	// convert to euler
	float y =  2*(q3*q4 + q1*q2);
	float x =  2*(0.5 - q2*q2 - q3*q3);
			q_pitch = atan2 (y,x) * -180 / M_PI;
			y = -2*(q2*q4 - q1*q3);
			x = 1;
			q_roll = asin(y/x) * -180 / M_PI;		
    
//			y =  2*(q2*q3 + q1*q4);
//			x =  2*(0.5 - q3*q3 - q4*q4);			
//			q_yaw = atan2 (y,x) * 180 / M_PI;
     
     // 
            q_yaw   += gz/sampleFreq * 180 / M_PI ;
}



//volatile void Read_Angle(void)
//{

//	ComplementaryFilter( AccelGyro, &pitch, &roll, &yaw);
//}

//void ComplementaryFilter(int16_t AccelGyro[6], float *pitch, float *roll, float *yaw)
//{
//    float pitchAcc, rollAcc;               
// 
//    // Integrate the gyroscope data -> int(angularSpeed) = angle
//    *pitch -= (AccelGyro[3] / GYROSCOPE_SENSITIVITY)/sampleFreq ;   // Angle around the X-axis
//    *roll  -= (AccelGyro[4] / GYROSCOPE_SENSITIVITY)/sampleFreq ;   // Angle around the Y-axis
//	  *yaw   += (AccelGyro[5] / GYROSCOPE_SENSITIVITY)/sampleFreq ;   // Angle around the Z-axis

//    // Compensate for drift with accelerometer data if !bullshit
//    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
//    int16_t forceMagnitudeApprox = abs(AccelGyro[0]) + abs(AccelGyro[1]) + abs(AccelGyro[2]);
//    if (((forceMagnitudeApprox > 8192) && (forceMagnitudeApprox < 32768)))
//    {
//	// Turning around the X axis results in a vector on the Y-axis
//        pitchAcc = atan2((float)AccelGyro[1], (float)AccelGyro[2]) * 180 / M_PI;
//        *pitch = *pitch * 0.96 - pitchAcc * 0.04 ;
// 
//	// Turning around the Y axis results in a vector on the X-axis
//        rollAcc = atan2((float)AccelGyro[0], (float)AccelGyro[2]) * 180 / M_PI;
//        *roll = *roll * 0.96 + rollAcc * 0.04 ;
//    }
//} 
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
