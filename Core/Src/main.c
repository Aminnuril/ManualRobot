/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CAN_SPI.h"
#include "stdio.h"
#include "4kinematik.h"
#include "motor_ctrl.h"
#include "ds4.h"
#include "ReadRPM.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define Calibrate 1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart3;

osThreadId InputTaskHandle;
osThreadId MechanismTaskHandle;
osThreadId OmniTaskHandle;
/* USER CODE BEGIN PV */
//input stik
int atas1, bawah1, kanan1,kiri1,kotak1,silang1,bulat1,segitiga1,tpad1,r3a;
int atas, bawah, kanan,kiri,kotak,silang,bulat,segitiga;
int l1, r1, l3, r3, share, options, ps, tpad;
int rx, ry,lx,ly,l2,r2,lx1,ly1,lxs,lys;
int rxm,rym,lxm,lym,th;
//omni
int wheel[4];
int pulse_enc, pulse_dist;
double rpm1 = 0, rpm2 = 0, rpm3 = 0, rpm4 = 0;
int ret;
int i=0;
double nos=1;
int putar=0, putar1=0;
//sistem mode
int mode=1;
int lsflag_1=0,lsflag_2=0,lsflag_3=0,lsflag_4=0,lsflag_5=0,lsflag_6=0,lsflag_7=0,flagA=0,flagFirst=0,capit=0;
int stepLoading=0;
int loopPadi=0, stepPadi=0,p1=0,p2=0,p3=0,p4=0,p5=0;
//BLDC
int kec=0,kec1=0;
int adj=0;
int dutyAdj=0;
uint32_t dutyR=3000;
int k=3000;
//debounce
uint32_t currentTick,nowTick;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
void InputHandle_Task(void const * argument);
void MechanismHandle_Task(void const * argument);
void OmniHandle_Task(void const * argument);

/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uCAN_MSG txMessage;
uCAN_MSG rxMessage;

int map(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
    return (int) (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

int _write(int file, char *ptr, int len){
	HAL_UART_Transmit_IT(&huart3, (uint8_t*)ptr, len);
	return len;
}
void mode_padi(){
	lsflag_6=HAL_GPIO_ReadPin(LS_6_GPIO_Port, LS_6_Pin);
	lsflag_7=HAL_GPIO_ReadPin(LS_7_GPIO_Port, LS_7_Pin);
	if (kanan==1){//nutup manual 1, buka -1
		motor_drive(&MExtendRight, 1, 750);//kanan 1
		lsflag_7=1;
	}
	else{
		motor_drive(&MExtendRight, 1, 0);
	}
	if (kiri==1){//nutup manual -1, buka 1
		motor_drive(&MExtendLeft, -1, 750);//kiri nutup -1
		lsflag_6=1;
	}
	else{
		motor_drive(&MExtendLeft, 1, 0);
	}

	if (atas==1) { //buka(extend) otomatis
		atas1=1;
	}
	if (atas1==1){
		if (lsflag_7==1){
			motor_drive(&MExtendRight, -1, 750);//buka
		}
		else {
			motor_drive(&MExtendRight, 0, 0);
		}
		if (lsflag_6==1){
			motor_drive(&MExtendLeft, 1, 750);//buka
		}
		else {
			motor_drive(&MExtendLeft, 0, 0);
		}
		if(lsflag_6 == 0 && lsflag_7 == 0){
			atas1=0;
		}
	}

	if (bawah==1){//buka manual
		atas1=0;
		if (lsflag_7==1){
			motor_drive(&MExtendRight, -1, 500);//kiri
		}
		else {
			motor_drive(&MExtendRight, 0, 0);
		}

		if (lsflag_6==1){
			motor_drive(&MExtendLeft, 1, 500);
		}
		else {
			motor_drive(&MExtendLeft, 0, 0);
		}
	}

	if (kotak==1){
		if(currentTick-nowTick>200){
			HAL_GPIO_TogglePin(PISTON_A_GPIO_Port, PISTON_A_Pin);
			HAL_GPIO_TogglePin(PISTON_B_GPIO_Port, PISTON_B_Pin);
			HAL_GPIO_TogglePin(PISTON_C_GPIO_Port, PISTON_C_Pin);
			HAL_GPIO_TogglePin(PISTON_D_GPIO_Port, PISTON_D_Pin);
			nowTick = currentTick;
		}
	}
	else if (silang==1){
		if(currentTick-nowTick>500){
			HAL_GPIO_TogglePin(PISTON_A_GPIO_Port, PISTON_A_Pin);
			nowTick = currentTick;
		}
	}
	else if (bulat==1){
		if(currentTick-nowTick>500){
			HAL_GPIO_TogglePin(PISTON_B_GPIO_Port, PISTON_B_Pin);
			nowTick = currentTick;
		}
	}
	if (l1==1){
		if (lsflag_4==0){//turun
			motor_drive(&MPadi, -1, 600);
		}
		else {
			motor_drive(&MPadi, 0 , 0);
		}
		lsflag_3=0;
	}
	else if (r1==1){
		if (lsflag_3==0){//naik
			motor_drive(&MPadi, 1, 850);
		}
		else {
			motor_drive(&MPadi, 0 , 0);
		}
		lsflag_4=0;
	}
	else{
		motor_drive(&MPadi, 1, 0);
	}

	if (r3==1){//start zone/ posisi awal tekuk dan bawah nyentuh ls4
		if(currentTick-nowTick>500){
			HAL_GPIO_TogglePin(PISTON_PADI_GPIO_Port, PISTON_PADI_Pin);
			nowTick = currentTick;
		}
		vTaskDelay(300);
		p4 = HAL_GPIO_ReadPin(PISTON_D_GPIO_Port, PISTON_D_Pin);
		p5 = HAL_GPIO_ReadPin(PISTON_PADI_GPIO_Port, PISTON_PADI_Pin);
		if(p5==0){
			atas1=1;
			HAL_GPIO_TogglePin(PISTON_D_GPIO_Port, PISTON_D_Pin);
		}
		lsflag_4=1;
		loopPadi=0;
		stepPadi=0;
	}
	else if (share==1){
		if(currentTick-nowTick>500){
			HAL_GPIO_TogglePin(PISTON_C_GPIO_Port, PISTON_C_Pin);
			nowTick = currentTick;
		}
	}
	else if (options==1){
		if(currentTick-nowTick>500){
			HAL_GPIO_TogglePin(PISTON_D_GPIO_Port, PISTON_D_Pin);
			nowTick = currentTick;
		}
	}
	else if (tpad==1){
		if(currentTick-nowTick>500){
			loopPadi+=1;
			nowTick = currentTick;
		}
		stepPadi=0;
	}
	if (loopPadi==1){
		if (stepPadi==0) {//posisi awal dibawah
			if (lsflag_4==0){//turun
				motor_drive(&MPadi, -1, 1000);
				lsflag_3=0;
			}
			else {
				motor_drive(&MPadi, 0 , 0);
				stepPadi=1;
			}
		}
		else if (stepPadi==1) {//cek buka capit padi
			p1 = HAL_GPIO_ReadPin(PISTON_A_GPIO_Port, PISTON_A_Pin);
			p2 = HAL_GPIO_ReadPin(PISTON_B_GPIO_Port, PISTON_B_Pin);
			p3 = HAL_GPIO_ReadPin(PISTON_C_GPIO_Port, PISTON_C_Pin);
			p4 = HAL_GPIO_ReadPin(PISTON_D_GPIO_Port, PISTON_D_Pin);
			if(p1==1 && p2==1 && p3==1 && p4==1){
				if(currentTick-nowTick>200){
					HAL_GPIO_TogglePin(PISTON_A_GPIO_Port, PISTON_A_Pin);
					HAL_GPIO_TogglePin(PISTON_B_GPIO_Port, PISTON_B_Pin);
					HAL_GPIO_TogglePin(PISTON_C_GPIO_Port, PISTON_C_Pin);
					HAL_GPIO_TogglePin(PISTON_D_GPIO_Port, PISTON_D_Pin);
					nowTick = currentTick;
				}
			}
			else if(p1==1){
				if(currentTick-nowTick>200){
					HAL_GPIO_TogglePin(PISTON_A_GPIO_Port, PISTON_A_Pin);
					nowTick = currentTick;
				}
			}
			else if(p2==1){
				if(currentTick-nowTick>200){
					HAL_GPIO_TogglePin(PISTON_B_GPIO_Port, PISTON_B_Pin);
					nowTick = currentTick;
				}
			}
			else if(p3==1){
				if(currentTick-nowTick>200){
					HAL_GPIO_TogglePin(PISTON_C_GPIO_Port, PISTON_C_Pin);
					nowTick = currentTick;
				}
			}
			else if(p4==1){
				if(currentTick-nowTick>200){
					HAL_GPIO_TogglePin(PISTON_D_GPIO_Port, PISTON_D_Pin);
					nowTick = currentTick;
				}
			}
			else {
				lsflag_4=1;
				lsflag_3=0;
				stepPadi=0;
			}
		}
	}
	else if (loopPadi==2){
		if(stepPadi==0){
			if(kotak==1){
				stepPadi=1;
			}
		}
		else if (stepPadi==1){
			if (lsflag_3==0){//naik
				motor_drive(&MPadi, 1, 1000);
			}
			else {
				motor_drive(&MPadi, 1 , 0);
				lsflag_4=0;
				stepPadi=5;
			}
		}
	}
	else if(loopPadi==3){
		if(stepPadi==0){
			if (lsflag_4==0){//turun
				motor_drive(&MPadi, -1, 800);
				lsflag_3=0;
			}
			else {
				motor_drive(&MPadi, 0 , 0);
				stepPadi=1;
			}
		}
		else if(stepPadi==1){
				HAL_GPIO_TogglePin(PISTON_A_GPIO_Port, PISTON_A_Pin);
				HAL_GPIO_TogglePin(PISTON_C_GPIO_Port, PISTON_C_Pin);
			stepPadi=2;
		}
		else if (stepPadi==2){
			if (lsflag_3==0){//naik
				motor_drive(&MPadi, 1, 1000);
				lsflag_4=0;
			}
			else {
				motor_drive(&MPadi, 0 , 0);
				lsflag_4=0;
				stepPadi=5;
			}
		}
	}
	else if(loopPadi==4){
		if(stepPadi==0){
			if (lsflag_4==0){//turun
				motor_drive(&MPadi, -1, 800);
			}
			else {
				motor_drive(&MPadi, 0 , 0);
				stepPadi=1;
				lsflag_3=0;
			}
		}
		else if(stepPadi==1){
			if(currentTick-nowTick>500){
				HAL_GPIO_TogglePin(PISTON_B_GPIO_Port, PISTON_B_Pin);
				HAL_GPIO_TogglePin(PISTON_D_GPIO_Port, PISTON_D_Pin);
				nowTick = currentTick;
			}
			stepPadi=2;
			lsflag_4=0;
		}
		else if (stepPadi==2){
			if (lsflag_3==0){//naik
				motor_drive(&MPadi, 1, 1000);
				lsflag_4=0;
			}
			else {
				motor_drive(&MPadi, 0 , 0);
				stepPadi=3;
				lsflag_3=1;
			}
		}
		else if(stepPadi==3){
			stepPadi=0;
			loopPadi=0;
		}
	}
	else{
		stepPadi=0;
		loopPadi=0;
	}
	bldc_drive(&roller1, 3000);
	bldc_drive(&roller2, 3000);
	motor_drive(&MPelontar, 0, 0);
	motor_drive(&MBola, 0, 0);
}

void mode_bola(){
	if (bawah==1) {
		if (lsflag_5==0){//turun
			motor_drive(&MPelontar, -1, 550);
		}
		else {
			motor_drive(&MPelontar, 0 , 0);
		}
	}
	else if (atas==1){//naik
		motor_drive(&MPelontar, 1, 450);
		lsflag_5=0;
	}
	else{
		motor_drive(&MPelontar, 0, 0);
	}

	if (kanan==1){
		if(currentTick-nowTick>500 && kec<5){
			adj=0;
			kec+=1;
			nowTick = currentTick;
		}
	}

	else if (kiri==1){
		if(currentTick-nowTick>500 && kec>0){
			adj=0;
			kec-=1;
			nowTick = currentTick;
		}
	}
	if (kec==1 && adj==0){
		if (k < 3499){
			for (k = 3000; k < 3500; ++k) {
				dutyR=k;
				kec1=k;
				osDelay(1);
			}
		}
		else{
			dutyR=3500;
			kec1=3500;
		}
	}
	else if (kec==2 && adj==0){dutyR=4000;kec1=4000;}
	else if (kec==3 && adj==0){dutyR=4500;kec1=4500;}
	else if (kec==4 && adj==0){dutyR=5000;kec1=5000;}
	else if (kec==0 && adj==0){dutyR=3000;kec1=3000;}

	if (kotak==1){
		HAL_GPIO_WritePin(PISTON_PELONTAR_GPIO_Port, PISTON_PELONTAR_Pin, 1);
	}
	else {
		HAL_GPIO_WritePin(PISTON_PELONTAR_GPIO_Port, PISTON_PELONTAR_Pin, 0);
	}

	if (silang==1){
		if (currentTick - nowTick > 500) {
			HAL_GPIO_TogglePin(PISTON_BOLA_GPIO_Port, PISTON_BOLA_Pin);
			capit = HAL_GPIO_ReadPin(PISTON_BOLA_GPIO_Port, PISTON_BOLA_Pin);
			nowTick = currentTick;
		}
	}
	if (bulat==1){
		motor_drive(&MPelontar, 0, 0);
		lsflag_5=1;
		stepLoading=0;
	}
	if (l1==1){
		if(lsflag_2==0){
			motor_drive(&MBola, -1, 800);
			lsflag_1=0;
		}
		else{
			motor_drive(&MBola, 0, 0);
		}
	}
	else if (r1==1){
		if(lsflag_1==0){
			motor_drive(&MBola, 1, 800);
			lsflag_2=0;
		}
		else{
			motor_drive(&MBola, 0, 0);
		}
	}
	else{
		motor_drive(&MBola, 0, 0);
	}

	if (share==1){
		adj=1;
		if(currentTick-nowTick>100 && kec1>3000){
			kec1-=10;
			nowTick = currentTick;
		}
	}
	else if (options==1){
		adj=1;
		if(currentTick-nowTick>100 && kec1<6000){
			kec1+=10;
			nowTick = currentTick;
		}

	}
	else if (tpad==1){
		tpad1=1;
	}
	if(tpad1==1){
		if(stepLoading==0){//ambil bola
			if	(lsflag_2==0){
				motor_drive(&MBola, 1, 800);
			}
			else{
				motor_drive(&MBola, 0, 0);
				lsflag_1=0;
			}
			if (lsflag_5==1 && lsflag_2==1 && capit==1){
				stepLoading=1;
			}
		}
		if(stepLoading==1){//taruh ke pelontar
			if(lsflag_1==0){
				motor_drive(&MBola, 1, 1000);
			}
			else {
				motor_drive(&MBola, 0, 0);
				HAL_GPIO_TogglePin(PISTON_BOLA_GPIO_Port, PISTON_BOLA_Pin);
				capit = HAL_GPIO_ReadPin(PISTON_BOLA_GPIO_Port, PISTON_BOLA_Pin);
				osDelay(3);
				stepLoading=2;
			}
			lsflag_2=0;
		}
		else if(stepLoading==2){//kembali posisi ambil
			if	(lsflag_2==0){
				motor_drive(&MBola, -1, 800);
			}
			else{
				motor_drive(&MBola, 0, 0);
				stepLoading=3;//end
			}
		}
		else{
//			lsflag_5=0;
			tpad1=0;
			stepLoading=0;
		}
	}
	else{
		lsflag_1=0;
		tpad1=0;
		stepLoading=0;
	}
	if (r3==1){
		flagFirst=1;
		lsflag_5=1;
		lsflag_1=0;
	}

	if (flagFirst==1){
		if(lsflag_2==0){
			motor_drive(&MBola, -1, 800);
		}
		else{
			motor_drive(&MBola, 0, 0);
			flagFirst=0;
		}
	}

	if(adj==1){
		bldc_drive(&roller1, kec1);
		bldc_drive(&roller2, kec1);
	}
	else{
		bldc_drive(&roller1, dutyR);
		bldc_drive(&roller2, dutyR);
	}
}
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
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM12_Init();
  MX_TIM1_Init();
  MX_SPI5_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
	#if Calibrate
	TIM12->CCR1 = 6000;  // Set the maximum pulse (2ms)
	HAL_Delay (2000);  // wait for 1 beep
	TIM12->CCR1 = 3000;   // Set the minimum Pulse (1ms)
	HAL_Delay (1000);  // wait for 2 beeps
	#endif
  //inisiasi bldc roller
  bldc_init(&roller1, 3000);
  bldc_init(&roller2, 3000);

//inisiasi canbus spi
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    ret = CANSPI_Initialize();
    if(ret < 0){
    	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    	  while(1){}
    }

  motor_init(&LeftFront);
  motor_init(&LeftBack);
  motor_init(&RightBack);
  motor_init(&RightFront);
  motor_init(&MPelontar);
  motor_init(&MBola);
  motor_init(&MExtendLeft);
  motor_init(&MExtendRight);
  motor_init(&MPadi);

  enable_motor(&LeftFront);
  enable_motor(&LeftBack);
  enable_motor(&RightBack);
  enable_motor(&RightFront);
  enable_motor(&MPelontar);
  enable_motor(&MBola);
  enable_motor(&MExtendLeft);
  enable_motor(&MExtendRight);
  enable_motor(&MPadi);
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
  /* definition and creation of InputTask */
  osThreadDef(InputTask, InputHandle_Task, osPriorityAboveNormal, 0, 512);
  InputTaskHandle = osThreadCreate(osThread(InputTask), NULL);

  /* definition and creation of MechanismTask */
  osThreadDef(MechanismTask, MechanismHandle_Task, osPriorityNormal, 0, 512);
  MechanismTaskHandle = osThreadCreate(osThread(MechanismTask), NULL);

  /* definition and creation of OmniTask */
  osThreadDef(OmniTask, OmniHandle_Task, osPriorityNormal, 0, 512);
  OmniTaskHandle = osThreadCreate(osThread(OmniTask), NULL);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 10-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 5-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 5-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 10-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 10-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 30-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 60000-1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PISTON_PELONTAR_Pin|PISTON_A_Pin|PISTON_B_Pin|PISTON_BOLA_Pin
                          |PISTON_PADI_Pin|PISTON_D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|EN_EXDR_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, EN_LF_Pin|EN_RF_Pin|EN_EXDL_Pin|EN_RB_Pin
                          |EN_PADI_Pin|EN_PELONTAR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, EN_LB_Pin|PISTON_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_BOLA_GPIO_Port, EN_BOLA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENC4_A_Pin ENC4_B_Pin ENC3_B_Pin ENC1_A_Pin
                           ENC2_A_Pin ENC3_A_Pin ENC2_B_Pin */
  GPIO_InitStruct.Pin = ENC4_A_Pin|ENC4_B_Pin|ENC3_B_Pin|ENC1_A_Pin
                          |ENC2_A_Pin|ENC3_A_Pin|ENC2_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PISTON_PELONTAR_Pin PISTON_A_Pin PISTON_B_Pin PISTON_BOLA_Pin
                           PISTON_PADI_Pin PISTON_D_Pin */
  GPIO_InitStruct.Pin = PISTON_PELONTAR_Pin|PISTON_A_Pin|PISTON_B_Pin|PISTON_BOLA_Pin
                          |PISTON_PADI_Pin|PISTON_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin EN_EXDR_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|EN_EXDR_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LS_2_Pin LS_3_Pin LS_1_Pin ENC1_B_Pin */
  GPIO_InitStruct.Pin = LS_2_Pin|LS_3_Pin|LS_1_Pin|ENC1_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_LF_Pin EN_RF_Pin EN_EXDL_Pin EN_RB_Pin
                           EN_PADI_Pin EN_PELONTAR_Pin */
  GPIO_InitStruct.Pin = EN_LF_Pin|EN_RF_Pin|EN_EXDL_Pin|EN_RB_Pin
                          |EN_PADI_Pin|EN_PELONTAR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LS_4_Pin LS_5_Pin */
  GPIO_InitStruct.Pin = LS_4_Pin|LS_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LS_7_Pin */
  GPIO_InitStruct.Pin = LS_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LS_7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_LB_Pin */
  GPIO_InitStruct.Pin = EN_LB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_LB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PISTON_C_Pin */
  GPIO_InitStruct.Pin = PISTON_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(PISTON_C_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LS_6_Pin */
  GPIO_InitStruct.Pin = LS_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LS_6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_BOLA_Pin */
  GPIO_InitStruct.Pin = EN_BOLA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_BOLA_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == LS_1_Pin) {
		lsflag_1 = 1;
	}
	if (GPIO_Pin == LS_2_Pin) {
		lsflag_2 = 1;
	}
	if (GPIO_Pin == LS_3_Pin) {
		lsflag_3 = 1;
	}
	if (GPIO_Pin == LS_4_Pin) {
		lsflag_4 = 1;
	}
	if (GPIO_Pin == LS_5_Pin) {
		lsflag_5 = 1;
	}

//	if(GPIO_Pin == ENC1_A_Pin) read_pulse_A(&R_front);
//	else if(GPIO_Pin == ENC1_B_Pin) read_pulse_B(&R_front);

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_InputHandle_Task */
/**
  * @brief  Function implementing the InputTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_InputHandle_Task */
void InputHandle_Task(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//	  printf("input task\r\n");
	  ds4();
	  lxm = lx - lxs;
	  lym = ly - lys;

//	  printf("\r\n at:%d, ki:%d, bw:%d, ka:%d, se3:%d, se4:%d, x:%d, o:%d,r1:%d, l1:%d, r3:%d, l3:%d, shr:%d, opt:%d, ps:%d, tpad:%d, lx:%d, ly:%d  ",
//			  atas, kiri, bawah, kanan, segitiga, kotak,silang,bulat,r1, l1, r3, l3, share, options,ps,tpad, lxm, lym);
//	  printf("\r\n lxs:%d , lys:%d, lx:%d, ly:%d, lxm:%d, lym:%d",
//	  			   lxs, lys, lx, ly, lxm, lym);
//	  printf("w1:%d, w2:%d, w3:%d, w4:%d, putar:%d", wheel[0], wheel[1], wheel[2], wheel[3], putar);
	  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_MechanismHandle_Task */
/**
* @brief Function implementing the MechanismTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MechanismHandle_Task */
void MechanismHandle_Task(void const * argument)
{
  /* USER CODE BEGIN MechanismHandle_Task */
  /* Infinite loop */
  for(;;)
  {
	  currentTick = HAL_GetTick(); //kayak millis
	  if(segitiga==1){
		  if(currentTick-nowTick>100 && nos<2){
			  nos+=0.2;
			  nowTick = currentTick;
		  }
	  }
	  else nos = 1;
	  if (ps == 1) {
		  if (currentTick - nowTick > 500) {
			  kec=0;
			  adj=0;
			  mode=mode*-1;
			  nowTick = currentTick;
		  }
	  }

	  if (mode>0) {
		  mode_padi();
	  }
	  else if (mode<0) {
		  mode_bola();
	  }

	  //printf("mechanism task\r\n");
	  osDelay(1);
  }
  /* USER CODE END MechanismHandle_Task */
}

/* USER CODE BEGIN Header_OmniHandle_Task */
/**
* @brief Function implementing the OmniTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OmniHandle_Task */
void OmniHandle_Task(void const * argument)
{
  /* USER CODE BEGIN OmniHandle_Task */
  /* Infinite loop */
  for(;;)
  {
//	  printf("omni task\r\n");
	  if (l2 < r2) putar = -1*r2;
	  else if (l2 > r2) putar = l2;
	  else putar = 0;

	  if (mode>0) putar1=putar/1.5;
	  else if (mode<0) putar1=putar/1.5;

	  th = putar1;

	  kinematikM(lxm, lym, th, nos);
	  for (uint8_t i = 0; i < 4; i++) {
		  wheel[0] = map(out[0], -360, 400, -1000, 1000);//386
		  wheel[1] = map(out[1], -360, 400, -1000, 1000);//360
		  wheel[2] = map(out[2], -360, 400, -1000, 1000);
		  wheel[3] = map(out[3], -360, 400, -1000, 1000);
	  }
//	  read_pulse_A(&R_front);
//	  read_pulse_B(&R_front);
//	  rpm4 = L_front.RPM;
//	  rpm2 = L_back.RPM;
//	  rpm3 = R_back.RPM;
//	  rpm1 = R_front.RPM;

//	  PID_Compute(&lf, wheel[0], rpm1);
//	  PID_Compute(&lb, wheel[1], rpm2);
//	  PID_Compute(&rb, wheel[2], rpm3);
//	  PID_Compute(&rf, wheel[3], rpm1);

//	  motor_drive(&LeftFront, out[0], lf.output);
//	  motor_drive(&LeftBack, out[1], lb.output);
//	  motor_drive(&RightBack, out[2], rb.output);
//	  motor_drive(&RightFront, out[3], rf.output);

	  motor_drive(&LeftFront, out[0], wheel[0]);
	  motor_drive(&LeftBack, out[1], wheel[1]);
	  motor_drive(&RightBack, out[2], wheel[2]);
	  motor_drive(&RightFront, out[3], wheel[3]);
//	  printf("out0:%d,out1:%d,out2:%d,out3:%d", out[0],out[1],out[2],out[3]);
	  osDelay(1);
  }
  /* USER CODE END OmniHandle_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
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
