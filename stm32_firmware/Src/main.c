/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "can.h"
#include "gpio.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "math.h"

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

/* USER CODE BEGIN PV */
uint8_t pc_cmd,pc_psnum,pc_cmdid,last_cmd;
uint8_t led_yflag,led_rflag,buzzer_yflag,buzzer_rflag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_CAN_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);

  OLED_Init();
  //接收过滤器配置，接收pc端报警状态
  CAN_FilterTypeDef filter;
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterIdHigh = 0x202 << 5;      /* 接收ID=0x123，左移5位 */
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x7F0 << 5;   /* 比较全部11位ID */
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan, &filter);


  HAL_CAN_Start(&hcan);              /* 启动CAN */
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  /* 开启接收中断 */

  OLED_ShowString(1,5,"PC-0X202");
  OLED_ShowString(2,1,"STATE:");
  OLED_ShowString(3,1,"P_NUM:");
  OLED_ShowString(4,1,"CMD_ID:");
  OLED_ShowString(2,7,"Normal");
  OLED_ShowNum(3,7,0,2);
  OLED_ShowNum(4,8,0,2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {
    if(pc_cmd==0){
      HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);

    }
    if(pc_cmd==1){
      HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
      if(led_yflag==1){
        HAL_GPIO_TogglePin(LED_Y_GPIO_Port,LED_Y_Pin);
        led_yflag=0;
      }
      if(buzzer_yflag==1){
        HAL_GPIO_TogglePin(BUZZER_GPIO_Port,BUZZER_Pin);
        buzzer_yflag=0;
      }
    }

    if(pc_cmd==2){
      HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
      if(led_rflag==1){
        HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);
        led_rflag=0;
      }
      if(buzzer_rflag==1){
        HAL_GPIO_TogglePin(BUZZER_GPIO_Port,BUZZER_Pin);
        buzzer_rflag=0;
      }
    }

    if(pc_cmd==3){
      HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_RESET);
    }
    
    if(pc_cmd!=last_cmd){
      last_cmd=pc_cmd;
      if(pc_cmd==0){OLED_ShowString(2,7,"      ");OLED_ShowString(2,7,"Normal");}
      if(pc_cmd==1){OLED_ShowString(2,7,"      ");OLED_ShowString(2,7,"Alarm");}
      if(pc_cmd==2){OLED_ShowString(2,7,"      ");OLED_ShowString(2,7,"Danger");}
      OLED_ShowNum(3,7,pc_psnum,2);
      OLED_ShowNum(4,8,pc_cmdid,2);
    }
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
}

/* USER CODE BEGIN 4 */
//CAN回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_Header;
  uint8_t rx_data[8];
  /* 读收到的帧 */
  HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &rx_Header, rx_data);
  if(rx_Header.StdId==0x202){
    pc_cmd=rx_data[0];
    pc_psnum=rx_data[1];
    pc_cmdid=rx_data[2];
    //ACK回发帧头结构体
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8] = {0};
    uint32_t txMailbox;

    txHeader.StdId = 0x301;         //ACK ID
    txHeader.IDE   = CAN_ID_STD;    //标准帧（11位）
    txHeader.RTR   = CAN_RTR_DATA;  // 数据帧（非请求帧）
    txHeader.DLC   = 8;             //8字节

    txData[0]=rx_data[2];
    txData[1]=0;
    txData[2]=0;
    HAL_CAN_AddTxMessage(hcan, &txHeader, txData, &txMailbox);
  }
  else if(rx_Header.StdId == 0x7F0){
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8] = {0};
    uint32_t txMailbox;

    txHeader.StdId = 0x7F0;
    txHeader.IDE   = CAN_ID_STD;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 8;

    txData[0] = 2;     // STM32节点ID
    txData[1] = 0;       // 0=运行
    
    HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
    HAL_CAN_AddTxMessage(hcan, &txHeader, txData, &txMailbox);
  }
}


uint8_t cnt_yled,cnt_rled,cnt_ybuzzer,cnt_rbuzzer;
//TIM回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
      cnt_yled++;cnt_rled++;cnt_ybuzzer++;cnt_rbuzzer++;
      if(cnt_yled==80){
        led_yflag=1;
         cnt_yled=0;
      }
      if(cnt_ybuzzer==80){
        buzzer_yflag=1;
        cnt_ybuzzer=0;
      }
      if(cnt_rled==15){
        led_rflag=1;
        cnt_rled=0;
      }
      if(cnt_rbuzzer==15){
        buzzer_rflag=1;
        cnt_rbuzzer=0;
      }
    }
}

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
#ifdef USE_FULL_ASSERT
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
