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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sbus.h"
#include "BottomControl.h"

#include "lk-mg_motor.h"

#include "bsp_uart6.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// BottomControl Bottom;  // 创建底盘
LKMG_t lk_motors[1];

int   g_state = 0;        // 存 S 后面的那个字段，比如 0
float g_joint_deg[6] = {0}; // 存最后 6 个数据

// 串口数据解析
static int parse_S_frame(const char *line, int *state_out, float joint_out[6])
{
  // line 形如：S,0,23.6,-31.2,38.4,-29.0,-32.6,76.7
  // 为了安全，复制到本地 buffer 再 strtok
  char buf[UART6_DMA_RX_SIZE];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  // 去掉末尾 \r \n（如果上位机发了）
  size_t n = strlen(buf);
  while (n > 0 && (buf[n-1] == '\r' || buf[n-1] == '\n')) {
    buf[n-1] = '\0';
    n--;
  }

  char *saveptr = NULL;
  char *tok = strtok_r(buf, ",", &saveptr);
  if (!tok) return 0;

  // 第 1 段：必须是 "S"
  if (!(tok[0] == 'S' && tok[1] == '\0')) return 0;

  // 第 2 段：state（整数）
  tok = strtok_r(NULL, ",", &saveptr);
  if (!tok) return 0;
  *state_out = (int)strtol(tok, NULL, 10);

  // 第 3~8 段：6 个 float
  for (int i = 0; i < 6; i++)
  {
    tok = strtok_r(NULL, ",", &saveptr);
    if (!tok) return 0;

    char *endp = NULL;
    float v = strtof(tok, &endp);
    if (endp == tok) return 0;   // 解析失败
    joint_out[i] = v;
  }

  return 1;
}

static void UART6_OnLine(char *str)
{
  int st;
  float j[6];

  if (parse_S_frame(str, &st, j))
  {
    g_state = st;
    for (int i = 0; i < 6; i++) {
      g_joint_deg[i] = j[i];
    }

    // 需要的话可打印确认（注意：中断里打印可能影响实时性）
    // UART6_Print("OK state=%d j=%.1f %.1f %.1f %.1f %.1f %.1f\r\n",
    //            g_state, g_joint_deg[0], g_joint_deg[1], g_joint_deg[2],
    //            g_joint_deg[3], g_joint_deg[4], g_joint_deg[5]);
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
  BSPLogInit();  // rtt调试器初始化
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_UART8_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  //SBUS_Init();
  BSP_UART6_Init();
  UART6_RegisterLineCallback(UART6_OnLine);
  //BottomInit(&Bottom, &hcan1);
  LKMG_InitAll(lk_motors, &hcan1);

  uint32_t last = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //PrintLog("hello\n");  // 发送日志
    // 发送波形 发送两个数据 可以在示波器上指定显示方式

    if (HAL_GetTick() - last >= 10)
    {
      last = HAL_GetTick();
      /*
      //摇杆输入 按照对应的通道对应控制方向
      SbusI6Mode(&Bottom, rc.channels[1], rc.channels[0], rc.channels[3]);
      BottomUpdate(&Bottom);

      BottomMotorSpeedlog(&Bottom, 2);
      */

      LKMG_SetPos(&lk_motors[0], g_joint_deg[0] * 100 * 10);
      LKMG_PosControl(lk_motors);
      LKMG_LogShow(&lk_motors[0]);

      UART6_Print("Hello %d ", g_state);
      for (int i = 0; i < 6; i++) {
        UART6_Print("%d ", (int)(g_joint_deg[i]*10));
      }
      UART6_Print("\n");
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
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
