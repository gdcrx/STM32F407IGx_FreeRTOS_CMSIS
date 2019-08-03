
  
/* Includes ------------------------------------------------------------------*/
#include "string.h"

#include "main.h"
#include "bsp/bsp_led.h"
#include "bsp/bsp_key.h"
#include "bsp/bsp_usart.h"
#include "bsp/bsp_base_tim.h"

#include "cmsis_os.h"
    
/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */
  

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
// 接收緩衝
uint8_t aRxBuffer;

// 定时器计数测试
__IO uint16_t timer_count=0;

/****************任务配置参数 begin *****************/

//任务名称
#define START_TASK_NAME     "Start_Task"
//任务优先级
#define START_TASK_PRIO		1
//任务堆栈大小	
#define START_STK_SIZE 		128  
//任务句柄
TaskHandle_t xStartTask_Handler;
//任务函数原型
void vStart_Task(void *pvParameters);
 
 
//任务名称
#define LED_TASK_NAME     "LED_Task"
//任务优先级
#define LED_TASK_PRIO		2
//任务堆栈大小	
#define LED_STK_SIZE 		50  
//任务句柄
TaskHandle_t xLEDTask_Handler;
//任务函数原型
void vLED_Task(void *pvParameters);

//任务名称
#define USART_TASK_NAME     "USART_Task"
//任务优先级
#define USART_TASK_PRIO		2
//任务堆栈大小	
#define USART_STK_SIZE 		128  
//任务句柄
TaskHandle_t xUSARTTask_Handler;
//任务函数原型
void vUSART_Task(void *pvParameters);


/****************任务配置参数 end *****************/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

   uint8_t txbuf[50];
   
  /* 复位所有外设，初始化Flash接口和系统滴答定时器
  STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user 
             can eventually implement his proper time base source (a general purpose 
             timer for example or other time source), keeping in mind that Time base 
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
             handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* 配置系统时钟 Configure the system clock to 168 MHz */
  SystemClock_Config();
   
  /* 板载LED初始化 */
  LED_GPIO_Init();

  /* 板载KEY按键初始化*/  
  KEY_GPIO_Init();
  
  /* 初始化串口并配置串口中断优先级 */
  MX_USARTx_Init();
  
  /* 基本定时器初始化：1ms中断一次 */
  BASIC_TIMx_Init();
  
  /* 在中断模式下启动定时器 */
  HAL_TIM_Base_Start_IT(&htimx);
  
  
  memcpy(txbuf,"这是一个串口中断接收回显实验\r\n",50);
  HAL_UART_Transmit(&husartx,txbuf,strlen((char *)txbuf),1000);
  
  memcpy(txbuf,"输入数据并以回车键结束\r\n",50);
  HAL_UART_Transmit(&husartx,txbuf,strlen((char *)txbuf),1000);
  
  
  /* 中断模式启动串口接收（設置状态寄存器（SR）的RXNE爲1，使能接收中断，接收數據時进入中断回调函数） */
  HAL_UART_Receive_IT(&husartx,&aRxBuffer,1);
  
  
   //创建开始任务
    xTaskCreate(vStart_Task,            //任务函数
                START_TASK_NAME,       //任务名称
                START_STK_SIZE,        //任务堆栈大小
                NULL,                  //传递给任务函数的参数
                START_TASK_PRIO,       //任务优先级
                &xStartTask_Handler);   //任务句柄   
                
    vTaskStartScheduler();          //开启任务调度 
 
}

/**
  * 函数功能: 创建任务应用
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void vStart_Task (void *pvParameters)
{
    xTaskCreate( vLED_Task,   	        /* 任务函数  */
                 LED_TASK_NAME,     	/* 任务名    */
                 LED_STK_SIZE,        	/* 任务栈大小，单位word，也就是4字节 */
                 NULL,              	/* 任务参数  */
                 LED_TASK_PRIO,         /* 任务优先级*/
                 &xLEDTask_Handler );   /* 任务句柄  */
	
    xTaskCreate( vUSART_Task,   	    /* 任务函数  */
                 USART_TASK_NAME,     	/* 任务名    */
                 USART_STK_SIZE,        /* 任务栈大小，单位word，也就是4字节 */
                 NULL,              	/* 任务参数  */
                 USART_TASK_PRIO,       /* 任务优先级*/
                 &xUSARTTask_Handler ); /* 任务句柄  */
	
    vTaskDelete(xStartTask_Handler); //删除开始任务
}

/**
  * 函数功能: LED灯
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 
  */
void vLED_Task (void *pvParameters)
{
   for(;;)
   {
     if(timer_count >= 1000)
     {
       timer_count = 0;
       LED1_TOGGLE;  
      }
      vTaskDelay(100);
   }
}

/**
  * 函数功能: 串口打印输出
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 
  */
void vUSART_Task (void *pvParameters)
{
   static float float_num=0.00;
    
   for(;;)
   { 
     float_num+=0.01f;
     printf("float_num的值为: %.4f\r\n",float_num);
     vTaskDelay(1000);
   }
}

/**
  * 函数功能: 基本定时器中断回调函数
  * 输入参数: htim：基本定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: 
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    timer_count++;
}

/**
  * 函数功能: 串口接收完成回调函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  // 接收到的數據從husartx發送出去
  HAL_UART_Transmit(&husartx,&aRxBuffer,1,0);
  HAL_UART_Receive_IT(&husartx,&aRxBuffer,1);
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000 // 用8000000 外部晶振
  *            PLL_M                          = 25       // 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7        // 4
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* 使能PWR时钟(Enable Power Control clock) */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* 设置调压器输出电压级别1(The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.)  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* 使用外部晶振作为时钟源(Enable HSE Oscillator and activate PLL with HSE as source)
  参照STM32F407IGx_Clock_Config.png时钟配置图*/
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;        // 外部晶振，8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                          // 打开HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                      // 打开PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;              // PLL时钟源选择HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                   // 8分频MHz
  RCC_OscInitStruct.PLL.PLLN = 336;                                 // 336倍频
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                       // 2分频，得到168MHz主时钟
  RCC_OscInitStruct.PLL.PLLQ = 4;                                   // USB/SDIO/随机数产生器等的主PLL分频系数
 
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;     // 系统时钟：168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;            // AHB时钟： 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;             // APB1时钟：42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;             // APB2时钟：84MHz
  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  // 使能CSS功能，优先使用外部晶振，外部失效时启用内部时钟源。
  HAL_RCC_EnableCSS();                                            

    // HAL_RCC_GetHCLKFreq()/1000    1ms中断一次
    // HAL_RCC_GetHCLKFreq()/100000	 10us中断一次
	// HAL_RCC_GetHCLKFreq()/1000000 1us中断一次
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                // 配置并启动系统滴答定时器
  
    /* 系统滴答定时器时钟源 */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* 系统滴答定时器中断优先级配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
  
}

/**
*
*按键中断回调函数
*
*中断服务函数中使用了FreeRTOS的系统时延函数vTaskDelay，调试发现进入vTaskDelay函数后，
*会在函数xTaskResumeAll中进入临界段函数，会卡死在下面代码处:
*    if ( uxCriticalNesting == 1)
*    {
*        configASSERT( ( portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK ) == 0 );  
*     }
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin==KEY1_GPIO_PIN)
  {
    //HAL_Delay(20);/* 延时一小段时间，消除抖动 */
    if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL)
    {
       
      LED1_ON;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(KEY1_GPIO_PIN);
  }
  else if(GPIO_Pin==KEY2_GPIO_PIN)
  {
    //HAL_Delay(20);/* 延时一小段时间，消除抖动 */
    if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL)
    {
    
      LED2_ON;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(KEY2_GPIO_PIN);
  }
  else if(GPIO_Pin==KEY3_GPIO_PIN)
  {
    //HAL_Delay(20);/* 延时一小段时间，消除抖动 */
    if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL)
    {
    
      LED3_ON;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(KEY3_GPIO_PIN);
  }
  else if(GPIO_Pin==KEY4_GPIO_PIN)
  {
    //HAL_Delay(20);/* 延时一小段时间，消除抖动 */
    if(HAL_GPIO_ReadPin(KEY4_GPIO,KEY4_GPIO_PIN)==KEY4_DOWN_LEVEL)
    {
     
      LED1_OFF;
      LED2_OFF;
      LED3_OFF;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(KEY4_GPIO_PIN);
  }
  else if(GPIO_Pin==KEY5_GPIO_PIN)
  {
    //HAL_Delay(20);/* 延时一小段时间，消除抖动 */
    if(HAL_GPIO_ReadPin(KEY5_GPIO,KEY5_GPIO_PIN)==KEY5_DOWN_LEVEL)
    {
     
      LED1_ON;  
      LED2_ON;
      LED3_ON;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(KEY5_GPIO_PIN);
  }  
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
