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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<memory.h>
#include<math.h>
#include<stdlib.h>
/*
CAN_TxHeaderTypeDef tx_header;
CAN_RxHeaderTypeDef rx_header;
CAN_FilterTypeDef can_filter_conf;
uint32_t can1_used_mailbox;
uint32_t can2_used_mailbox;
uint8_t Send_Data[2]={0};
uint8_t Can1_Receive_Data[5]={0};
uint8_t Can2_Receive_Data[5]={0};
uint32_t CAN1_TXID=0x114;
uint32_t CAN2_TXID=0x115;
uint32_t Can1_FilterBank=0;
uint32_t Can2_FilterBank=14;
uint32_t RTR=0;
uint32_t IDE=0;
uint32_t EXID=0;*/
typedef struct{
  CAN_HandleTypeDef *can_handle;  //can句柄
  CAN_TxHeaderTypeDef txconf;     //CAN报文发送配置
  uint32_t tx_id;                 //发送id
  uint32_t tx_mailbox;            //CAN消息填入的邮箱号
  uint8_t tx_buff[8];             //发送缓存，发送的消息长度可以通过CANSetDLD()设定，最大为8
  uint8_t rx_buff[8];              //接收缓存，最大消息长度为8
  uint32_t rx_id;                 //接收id
uint8_t rx_len;                    //接收长度，可能为0——8；
}CANInstance;

/*DJI电机CAN反馈信息*/
typedef struct{
  //原始数据
  uint16_t ecd;               //0——8191，刻度总共有8192格
  uint16_t speed_rpm;         //转速，单位rpm
  int16_t real_current;         //实际电流
  uint8_t temperate;           //温度 Celsius
  
  //处理后的数据
  int16_t real_current_handle;     //处理后的电流
  float speed_aps;                //角速度，单位为：度/秒
  uint16_t last_ecd;             //上一次读取的编码器值
float angle_single_round;          //单圈角度，注意方向
float total_angle;                    //总角度，注意方向
int32_t total_round;           //总圈数
}DJI_Motor_Measure_s;

#define PI     3.14159265358979f   /* 注意：宏末尾不能带分号，否则 sin(t*PI) 会展开成 sin(t*3.14f;) 而报语法错 */
float t=0.0f;//用于正弦波计算的时间变量
CANInstance motor1;
DJI_Motor_Measure_s motor1_measure;
int16_t current;
float current_filter=0.9f;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 早期草稿版回调：引用了上面已被注释掉的变量，且与本文件下方的正式回调重名。
   保留在此备忘、不参与编译；真正生效的是下方调用 CANFIFOxCallback() 的两个版本。
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
 HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &rx_header, Can1_Receive_Data);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
 HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &rx_header, Can2_Receive_Data);
}
*/
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void MotorDataRed(void);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static void CANFIFOxCallback(CAN_HandleTypeDef *_hcan,uint32_t fifox)
{
    static CAN_RxHeaderTypeDef rxconf;//同上
    uint8_t can_rx_buff[8];            //修正拼写：bhuff -> buff
while(HAL_CAN_GetRxFifoFillLevel(_hcan,fifox))//FIFO不为空，有可能在其他中断时有多帧数据进入
{
 HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff);//从FIFO中获取数据（修正函数名与参数：HAL_CAN_getRXMessage -> HAL_CAN_GetRxMessage，_hcan.fifox -> fifox）
if(_hcan==motor1.can_handle&&rxconf.StdId==motor1.rx_id)
{
    //补：确认是 8 字节标准数据帧，避免异常帧混进解析（手册要求）
    if (rxconf.IDE != CAN_ID_STD || rxconf.RTR != CAN_RTR_DATA || rxconf.DLC != 8)
        continue;

    motor1.rx_len=rxconf.DLC;                       //保存接收到的数据长度
    memcpy(motor1.rx_buff,can_rx_buff,rxconf.DLC);  //消息拷贝到对应实例
    motor1_measure.last_ecd=motor1_measure.ecd;
  //解析电机数据
    MotorDataRed();
  //处理数据
    motor1_measure.angle_single_round=(360.0f/8192)*(float)motor1_measure.ecd;
    motor1_measure.speed_aps=(1.0f-0.85f)*motor1_measure.speed_aps+0.85f*(360.0f/60.0f)*(float)motor1_measure.speed_rpm;
    //修正拼写：current_fitter -> current_filter；一阶低通：α 越小越平滑、跟随越慢
    motor1_measure.real_current_handle=(1.0f-current_filter)*motor1_measure.real_current_handle+current_filter*(float)motor1_measure.real_current;
  //多圈角度计算，前提是假设两次采样间电机转过的角度小于180°，自己画个图就秦楚计算过程了
  //补：uint16_t 在 32 位 ARM 上会整数提升为 int，所以这里的差值是“有符号”的，过零时能得到负数，原判断逻辑成立
    if(motor1_measure.ecd-motor1_measure.last_ecd>4096)
    motor1_measure.total_round--;
    else if(motor1_measure.ecd-motor1_measure.last_ecd<-4096)
    motor1_measure.total_round++;
    motor1_measure.total_angle=motor1_measure.total_round*360+motor1_measure.angle_single_round;

   }
  }
   //修正：原来 return 在 while 内部，收到第一帧就退出，无法把 FIFO 读空；移到循环外
   return;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
     CANFIFOxCallback(hcan,CAN_RX_FIFO0); //调用我们自己写的函数来处理消息
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan,CAN_RX_FIFO1); //调用自己写的函数来处理消息
}
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
/*初始化CAN服务*/
HAL_CAN_Start(&hcan1);
HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING );
HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING );
HAL_CAN_Start(&hcan2);
HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING ); 
HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING );
 /*配置一个电机的基本参数*/
 motor1.txconf.StdId=0x200;             //发送id
 motor1.txconf.IDE=CAN_ID_STD;          // 使用标准id,扩展id则使用CAN_ID_EXT
 motor1.txconf.RTR=CAN_RTR_DATA;        //发送数据帧
 motor1.txconf.DLC=8;                    //默认发送长度为8
 motor1.can_handle=&hcan1;               
 motor1.tx_id=0x200;              
 motor1.rx_id=0x201;
 /*配置这个电机的过滤器*/
 CAN_FilterTypeDef can_filter_conf;   //修正拼写：fliter -> filter（下方用的就是这个变量）
 static uint8_t can1_filter_idx=0,can2_filter_idx=14;  //修正拼写：cna1 -> can1
 can_filter_conf.FilterMode=CAN_FILTERMODE_IDLIST;             // 使用 id_list 模式
 can_filter_conf.FilterScale=CAN_FILTERSCALE_16BIT;             // 使用 16 位 id 模式，即只有 16 位有效
 can_filter_conf.FilterFIFOAssignment=(motor1.tx_id&1)?CAN_RX_FIFO0:CAN_RX_FIFO1; //奇数id模块会被分到FIFO0,偶数id模块会被分到FIFO1
 //补：本例 tx_id=0x200 是偶数，所以反馈落到 FIFO1，对应 CAN1 RX1 中断 / FIFO1 回调
can_filter_conf.SlaveStartFilterBank=14;                         // 从第 14 个过滤器组开始配置从机过滤器
can_filter_conf.FilterIdLow=motor1.rx_id<<5;                       // 配置过滤器的低 16 位，低 5 位为 0，接收 id 为 0x201
//补：16 位列表模式下一组有 4 个匹配项，只接一个电调时四项都填同一个 ID（标准 ID 占 15~5 位，故左移 5 位）
can_filter_conf.FilterIdHigh=motor1.rx_id<<5;
can_filter_conf.FilterMaskIdLow=motor1.rx_id<<5;
can_filter_conf.FilterMaskIdHigh=motor1.rx_id<<5;
can_filter_conf.FilterBank=motor1.can_handle==&hcan1?(can1_filter_idx++):(can2_filter_idx++); //更具can_handle来判断是CAN1还是CAN2,然后自增
can_filter_conf.FilterActivation=CAN_FILTER_ENABLE;                  // 启用过滤器
HAL_CAN_ConfigFilter(motor1.can_handle,&can_filter_conf);
 /* 以下是前一版环回实验的旧配置草稿，已被上面的 C620 配置取代，保留备忘、不参与编译。
    原稿中间的 CAN2 标记用了块注释，导致注释提前闭合、后半段代码被意外编译，这里统一改成行注释形式。
 HAL_CAN_Start(&hcan1);
  __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
  // 监视 &114 id 进行匹配
  can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;       // 使用 id_list 模式
  can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;       // 使用 16 位 id 模式，即只有 16 位有效
  can_filter_conf.FilterFIFOAssignment = CAN_FILTER_FIFO1;        // 将该 id 分配到 FIFO1
  can_filter_conf.SlaveStartFilterBank = 14;              // 从第 14 个过滤器组开始分配给 CAN2
  can_filter_conf.FilterIdLow=(CAN1_TXID<<5)|(RTR<<4)|(IDE<<3)|EXID;  // 配置过滤器的低 16 位，低 5 位为 0，接收 id 为 0x114
  can_filter_conf.FilterBank=Can1_FilterBank;                       // 配置过滤器的编号为 0
  can_filter_conf.FilterActivation=CAN_FILTER_ENABLE;                  // 启用过滤器
  HAL_CAN_ConfigFilter(&hcan1,&can_filter_conf);
  // CAN2
  HAL_CAN_Start(&hcan2);
  __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
  // 监视 &115 id 进行匹配
  can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;
  can_filter_conf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can_filter_conf.SlaveStartFilterBank = 14;
  can_filter_conf.FilterIdLow=(CAN2_TXID<<5)|(RTR<<4)|(IDE<<3)|EXID;
  can_filter_conf.FilterBank=Can2_FilterBank;
  can_filter_conf.FilterActivation=CAN_FILTER_ENABLE;
  HAL_CAN_ConfigFilter(&hcan2,&can_filter_conf);
  */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
current=1000*sin(t*PI);
t+=0.02f;
if(HAL_CAN_GetTxMailboxesFreeLevel(motor1.can_handle)==0);//等待邮箱空间
else
{
//补：先转 uint16_t 保留 16 位补码再拆字节；直接对 int16_t 右移会因算术右移把负数高位填成 0xFF
uint16_t raw=(uint16_t)current;
motor1.tx_buff[0]=(uint8_t)(raw>>8);    //高八位
motor1.tx_buff[1]=(uint8_t)(raw & 0xFFU);//低八位
motor1.tx_buff[2]=0;
motor1.tx_buff[3]=0;
motor1.tx_buff[4]=0;
motor1.tx_buff[5]=0;
motor1.tx_buff[6]=0;
motor1.tx_buff[7]=0;
HAL_CAN_AddTxMessage(motor1.can_handle,&motor1.txconf,motor1.tx_buff,&motor1.tx_mailbox);
}
HAL_Delay(10);//每10ms发送一次
  }
/*Send_Data[0]++;
   Send_Data[1]--;
    // 配置 Can1 发送的报文
    tx_header.StdId=CAN1_TXID;
    tx_header.ExtId=EXID;
    tx_header.IDE=IDE;
    tx_header.RTR=RTR;
    tx_header.DLC=sizeof(Send_Data);
    HAL_CAN_AddTxMessage(&hcan1,&tx_header,Send_Data,&can1_used_mailbox);
    // 配置 Can2 发送的报文
    tx_header.StdId=CAN2_TXID;
    tx_header.ExtId=EXID;
    tx_header.IDE=IDE;
    tx_header.RTR=RTR;
    tx_header.DLC=sizeof(Send_Data);
    HAL_CAN_AddTxMessage(&hcan2,&tx_header,Send_Data,&can2_used_mailbox);

    HAL_Delay(250);
  }*/
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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  补：解析 C620 电调 0x201 反馈报文（8 字节，高字节在前）
  *         数据在 motor1.rx_buff 中，由 CANFIFOxCallback() 拷贝进来
  *         字节定义见《RM C620 无刷电机调速器使用说明》第 33 页「电调反馈报文格式」
  */
void MotorDataRed(void)
{
    // DATA[0..1]：转子单圈编码器值，0~8191（一圈 8192 格）
    motor1_measure.ecd = (uint16_t)((uint16_t)motor1.rx_buff[0] << 8 | motor1.rx_buff[1]);

    // DATA[2..3]：转速，单位 rpm；用 int16_t 保存，反转时读到负值
    motor1_measure.speed_rpm = (int16_t)((uint16_t)motor1.rx_buff[2] << 8 | motor1.rx_buff[3]);

    // DATA[4..5]：实际转矩电流；用 int16_t 保存，反转时读到负值
    motor1_measure.real_current = (int16_t)((uint16_t)motor1.rx_buff[4] << 8 | motor1.rx_buff[5]);

    // DATA[6]：电机温度，单字节直接读（摄氏度）
    motor1_measure.temperate = motor1.rx_buff[6];

    // DATA[7]：协议保留字节，不解析
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
