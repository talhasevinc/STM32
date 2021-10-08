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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*                           MPU6050                     */
bool mpuInit=0;
float X_cal,Y_cal,Z_cal,A_Pitch,A_Roll,A_Z;
int sampleT=4;



uint32_t XRaw,YRaw,ZRaw;
/*                           HCSR04                      */


bool timeDone=0;
bool captureFirst=0;

uint32_t timeDistance=0;
uint32_t distance=0;

uint32_t microsecond=0;
uint32_t milisecond=0;
uint32_t second=0;

uint32_t targetTime=0;
uint32_t FirstValue=0;
uint32_t SecondValue=0;



/*                          UART Message                */
char message[150];
/*                          Systick Timer               */

uint32_t SystickMs=0;
uint32_t SystickSecond=0;

/*                          Game Play               */
bool foundTarget=0;
bool distanceFound=0;
bool slopeFound=0;
bool targetDegressFound=0;
bool targetDegreeFound=0;
bool targetDistanceFound=0;
bool newGame=0;
float target_X=0.00;        //Target Pitch Value
float target_Y=0.00;        //Target Roll Value
float targetDistance=0.00;  //Target Distance

float maxHeight=50.00;      //Maximum height value. This valus can be changed. HCSR-04 is a sensor don't work stabil.
                            //So this value has to be small as soon as possible. This value effect led brigtness.
float maxDifference=0.00;
float differentHeight=0.00;
int ledPulse=0;
int acceptableHeightDifference=2;  // For example your target distance is 18. When measured value equals 16 or 20, target found flag will be true. Because this value 2.
float PitchValue,RollValue;        //Measured values in the game.
float accaptableDegree=3.0;        //AccaptableAngle for target. Can be changed.
float differenceAngle=0.00;

/* Necessary time measurement value*/

uint32_t timeServo=0;
uint32_t timeLed=0;
uint32_t timeNow=0;
uint32_t timePrev=0;
uint32_t gameTimer=0;
uint32_t timeCount=0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void HCSR04_ReadDistance();
void servoAct(uint32_t pulse);
void ledDrive(uint32_t pulse);
void getSlope();
void Write_To_Flash(uint32_t myData,uint32_t myAddress);
void sendDataviaUART();
void calibrationStage();
void firstOpenOfSystem();
void GameStart();
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

//For new game starting.

void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
    newGame=1;
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}


//Servo drive timer

void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	timeCount++;
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}


//For HCSR-04 measurement. When Echo pin is high, timer1 is starting to measure time. Then, when echo pin is low, timer1 stop to measure time.
//This measured time value give us distance.

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if (captureFirst==0)
		{

			FirstValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			captureFirst = 1;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (captureFirst==1)
		{
			SecondValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			__HAL_TIM_SET_COUNTER(htim, 0);

			if (SecondValue > FirstValue)
			{
				timeDistance = SecondValue-FirstValue;
			}

			else if (FirstValue > SecondValue)
			{
				timeDistance = (255 - FirstValue) + SecondValue;
			}

			distance = timeDistance * .034/2;
			if(distance>0 && distance<400)
			{
			   captureFirst = 0; // set it back to false
			   distanceFound=1;
			}
			else
			{
				captureFirst = 0; // set it back to false
				distanceFound=0;
				distance=0;

			}
			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
}



void Write_To_Flash(uint32_t myData,uint32_t myAddress)
{

      HAL_FLASH_Unlock();
      FLASH_Erase_Sector(FLASH_SECTOR_11, FLASH_VOLTAGE_RANGE_3);
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, myAddress, myData);
      HAL_FLASH_Lock();

}


uint32_t Read_From_Flash(uint32_t myAddress)
{
	uint32_t readData;
	readData=*(uint32_t*)myAddress;

    return readData;
}


//Delay function for HCSR-04. For starting HCSR-04, first you have to pull trigger pin logic-1. After 10us, pull logic-0.
//Then active Timer input capture and measure distance.

void delay(uint32_t us)
{
     __HAL_TIM_SET_COUNTER(&htim1,0);
     while(__HAL_TIM_GET_COUNTER(&htim1)<us);
}

void HCSR04_ReadDistance (void)
{
	 HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low
     delay(1000);



     timeCount=0;
     HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	 delay(10);
	 HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low
	 __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);


}



void servoAct(uint32_t pulse)
{
	   HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

	   __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1, 500+pulse*40);

}


/*This functions are my driving servo motor library. I generate my pwm signal thanks to timer values.*/

//Servo pin configuration
void servoGpioInit()
{

	 GPIO_InitTypeDef GPIO_InitStruct = {0};

	  __HAL_RCC_GPIOH_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();

	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

	  GPIO_InitStruct.Pin = GPIO_PIN_12;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

// Necessary timer calculation and move servo motor desired angle.
void myServoMove(int degree)
{

	 servoGpioInit();

	 HAL_TIM_Base_Start_IT(&htim2);

	 int value=(int)(25+(100*degree/180));
	 if(value>125)
		 value=125;
	 for(int i=0;i<50;i++)
	 {
		      timeCount=0;

              HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
			  while(timeCount <value );

			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			  while(timeCount<1000);
			  timeCount=0;

	 }

     HAL_TIM_Base_Stop_IT(&htim2);
     HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12);


}

//Led PWM

void ledDrive(uint32_t pulse)
{

	   __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,pulse);

}

//Send message buffer via uart to PC

void sendDataviaUART()
{
     HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message),150);

}

//This function is necessary for calibrating MPU6050 sensor.

void calibrationStage()
{
	  Write_To_Flash(0xFFFFFFFF,CALIB_OK);
	  HAL_Delay(100);

	  if(Read_From_Flash(CALIB_OK) == 0xFFFFFFFF)
	  {


		  MPU6050Calibration(5);
		  X_cal=X_calib();
		  Y_cal=Y_calib();
		  Z_cal=Z_calib();

		  Write_To_Flash((uint32_t)X_cal , X_CALIB);
		  Write_To_Flash((uint32_t)Y_cal , Y_CALIB);
		  Write_To_Flash((uint32_t)Z_cal , Z_CALIB);
		  Write_To_Flash(sampleT,CALIB_OK);

	  }

	  else
	  {
	      setSampleTime(Read_From_Flash(CALIB_OK));
	      X_cal=Read_From_Flash(X_CALIB);
	      Y_cal=Read_From_Flash(Y_CALIB);
	      Z_cal=Read_From_Flash(Z_CALIB);
	      setCalibrationValue(X_cal, Y_cal, Z_cal);

	  }



}


void firstOpenOfSystem()
{
		sprintf(message,"You've got 5 seconds to take first position your system\r");
		sendDataviaUART();

		SystickSecond=0;

		while(SystickSecond<5)
		{
			if(MPU6050Init())
			{
				getAllDatas();
            }

			else
			{
				sprintf(message,"MPU6050 Error...\n");
				sendDataviaUART();

			}


		}

		bool keepDone=false;
		SystickMs=0;

		sprintf(message,"\r\nBeginnig target values will measured....\r\n");
		sendDataviaUART();
		sprintf(message,"Do not change position when new message arrives you...\r");
		sendDataviaUART();

		while(!keepDone)
		{
			 targetDegressFound=0;
			 if(MPU6050Init())
			 {
				    getAllDatas();
					target_X=getPitch();
					target_Y=getRoll();
					targetDegressFound=1;

			  }

			  if(SystickMs>100)
			  {
				SystickMs=0;
				HCSR04_ReadDistance();
				if(distanceFound && distance<maxHeight)
				{
					distanceFound=0;
					targetDistance=distance;
					if((maxHeight-targetDistance)>targetDistance)
						maxDifference=maxHeight-targetDistance;
					else
						maxDifference=targetDistance;
					if(targetDegressFound)
						keepDone=1;

				}
			  }
		}

		sprintf(message,"Calculation Okay. Target_Pitch:%0.4f,Target_Roll:%0.4f,Target_Distance:%d \n\r",target_X,target_Y,(int)targetDistance);
		sendDataviaUART();
		sprintf(message,"Let it go the system...\r\n");
		sendDataviaUART();
}

void GameStart()
{
	SystickSecond=0;
	sprintf(message,"Game is starting...\r");
	sendDataviaUART();

	SystickMs=0;

	foundTarget=0;
	targetDistanceFound=0;
	targetDegreeFound=0;
	timeServo=0;
	timeLed=0;
	distance=0;
    int totalAngle=0;
    float rollOnly=0.00;
    float pitchOnly=0.0;
    while(!foundTarget)
    {
    	targetDegreeFound=0;
    	targetDistanceFound=0;

    	if(MPU6050Init())
    	{
    		totalAngle=0;
    		getAllDatas();
    		PitchValue=getPitch();
    	    RollValue=getRoll();

            if(RollValue>45 &&target_Y<30 )
            {
            	rollOnly=(65-RollValue+target_Y);
            }
            else if(RollValue>45 &&target_Y>30 )
            {
                rollOnly=abs(RollValue-target_Y);
             }
            else if(RollValue<45 && target_Y<30 )
            {
            	rollOnly=abs(RollValue-target_Y);
            }
            else if(RollValue<45 &&target_Y>30 )
            {
            	rollOnly=(65+RollValue-target_Y);
             }

            if(PitchValue>-45 &&target_X<-45 )
            {
            	pitchOnly=(65-PitchValue+target_X);
            }
            else if(PitchValue>-45 &&target_X>-45 )
            {
            	pitchOnly=abs(PitchValue-target_X);
             }
            else if(PitchValue<-45 && target_X<-45 )
            {
            	pitchOnly=abs(PitchValue-target_X);
            }
            else if(PitchValue<-45 &&target_X>-45 )
            {
            	pitchOnly=(65+PitchValue-target_X);
             }

    	    differenceAngle=sqrt( pitchOnly*pitchOnly +rollOnly*rollOnly);

    	    if( differenceAngle  <accaptableDegree)
    	    {   targetDegreeFound=1; differenceAngle=0; servoAct((uint32_t)differenceAngle); }
    	    else
    	    {
    	    	targetDegreeFound=0;
    	    	if(differenceAngle>50) differenceAngle=50;
    	    }
    	    if(timeServo>700)
    	    {
    	       timeServo=0;
    	       if(!targetDegreeFound)
    	          servoAct((uint32_t)differenceAngle);
    	    }

        }

		if(SystickMs>400)
		{
		  SystickMs=0;
		  HCSR04_ReadDistance();
		  HAL_Delay(1);
		  if(distanceFound && distance<maxHeight )
		  {

			 distanceFound=0;
			 differentHeight=abs(distance-targetDistance);
			 if(differentHeight>maxDifference)
				differentHeight=maxDifference;

			 if(differentHeight<=acceptableHeightDifference)
			 {
				 differentHeight=targetDistance;
				 targetDistanceFound=1;
				 ledDrive(0);
			 }

			 else
				 targetDistanceFound=0;

			 if(timeLed>700)
			 {
				 timeLed=0;
				 if(!targetDistanceFound)
				 {
					 ledPulse=255*differentHeight/maxDifference;
					 ledDrive(ledPulse);
				 }
			 }
		   }
		   distanceFound=0;



		}

		if(targetDistanceFound && targetDegreeFound)
		{
			foundTarget=1;
			sprintf(message,"\nTarget_Pitch:%0.4f, Target_Roll:%0.4f,Target_Distance:%d ",target_X,target_Y,(int)targetDistance);
			sendDataviaUART();
			sprintf(message,"\nPitch:%0.4f,Roll:%0.4f,Distance:%d",PitchValue,RollValue,distance);
			sendDataviaUART();

		}
		if(gameTimer>700)
		{
			gameTimer=0;
			sprintf(message,"\nTarget_Pitch:%0.4f, Target_Roll:%0.4f,Target_Distance:%d ",target_X,target_Y,(int)targetDistance);
			sendDataviaUART();
			sprintf(message,"\nPitch:%0.4f,Roll:%0.4f,Distance:%d",PitchValue,RollValue,distance);
			sendDataviaUART();

		}
        while(SystickMs %5 !=0);

    }

	sprintf(message,"\n\rGame finish Succesfully. Congratulations\r");
	sendDataviaUART();
	foundTarget=0;
	targetDistanceFound=0;
	targetDegreeFound=0;

}

void waitNewPosition()
{

	SystickSecond=0;
	sprintf(message,"Please change position and wait 10 seconds...\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), 1000);
	while(SystickSecond<10)
	{

		if(MPU6050Init())
		{
			getAllDatas();

		}
	}

}

void testLed()
{
	for(int i=0;i<255;i+=10)
	{
		ledDrive(i);
		HAL_Delay(20);

	}
}

void testServo()
{
	for(int i=0;i<50;i+=1)
	{
		servoAct(i);
		HAL_Delay(20);

	}

}
void waitNewGame()
{
	sprintf(message,"For new game, press the user button...\r");
	sendDataviaUART();

	while(!newGame);
	newGame=0;

}
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

  testLed();
  testServo();

  getAccelerationRawData();
  while(!MPU6050Init())
  {

	  sprintf(message,"MPU6050 Error.Check wire connections...\n");
	  sendDataviaUART();
	  HAL_Delay(3000);

  }

  sprintf(message,"Wait for initialization and calibration\n");
  sendDataviaUART();
  MPU6050Initialization();
  calibrationStage();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	firstOpenOfSystem();
	waitNewPosition();
	GameStart();
    waitNewGame();



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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1291;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 255-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PE10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
