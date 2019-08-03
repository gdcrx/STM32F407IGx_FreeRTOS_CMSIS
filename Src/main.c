
  
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
// ���վ��n
uint8_t aRxBuffer;

// ��ʱ����������
__IO uint16_t timer_count=0;

/****************�������ò��� begin *****************/

//��������
#define START_TASK_NAME     "Start_Task"
//�������ȼ�
#define START_TASK_PRIO		1
//�����ջ��С	
#define START_STK_SIZE 		128  
//������
TaskHandle_t xStartTask_Handler;
//������ԭ��
void vStart_Task(void *pvParameters);
 
 
//��������
#define LED_TASK_NAME     "LED_Task"
//�������ȼ�
#define LED_TASK_PRIO		2
//�����ջ��С	
#define LED_STK_SIZE 		50  
//������
TaskHandle_t xLEDTask_Handler;
//������ԭ��
void vLED_Task(void *pvParameters);

//��������
#define USART_TASK_NAME     "USART_Task"
//�������ȼ�
#define USART_TASK_PRIO		2
//�����ջ��С	
#define USART_STK_SIZE 		128  
//������
TaskHandle_t xUSARTTask_Handler;
//������ԭ��
void vUSART_Task(void *pvParameters);


/****************�������ò��� end *****************/

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
   
  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ��
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

  /* ����ϵͳʱ�� Configure the system clock to 168 MHz */
  SystemClock_Config();
   
  /* ����LED��ʼ�� */
  LED_GPIO_Init();

  /* ����KEY������ʼ��*/  
  KEY_GPIO_Init();
  
  /* ��ʼ�����ڲ����ô����ж����ȼ� */
  MX_USARTx_Init();
  
  /* ������ʱ����ʼ����1ms�ж�һ�� */
  BASIC_TIMx_Init();
  
  /* ���ж�ģʽ��������ʱ�� */
  HAL_TIM_Base_Start_IT(&htimx);
  
  
  memcpy(txbuf,"����һ�������жϽ��ջ���ʵ��\r\n",50);
  HAL_UART_Transmit(&husartx,txbuf,strlen((char *)txbuf),1000);
  
  memcpy(txbuf,"�������ݲ��Իس�������\r\n",50);
  HAL_UART_Transmit(&husartx,txbuf,strlen((char *)txbuf),1000);
  
  
  /* �ж�ģʽ�������ڽ��գ��O��״̬�Ĵ�����SR����RXNE��1��ʹ�ܽ����жϣ����Ք����r�����жϻص������� */
  HAL_UART_Receive_IT(&husartx,&aRxBuffer,1);
  
  
   //������ʼ����
    xTaskCreate(vStart_Task,            //������
                START_TASK_NAME,       //��������
                START_STK_SIZE,        //�����ջ��С
                NULL,                  //���ݸ��������Ĳ���
                START_TASK_PRIO,       //�������ȼ�
                &xStartTask_Handler);   //������   
                
    vTaskStartScheduler();          //����������� 
 
}

/**
  * ��������: ��������Ӧ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void vStart_Task (void *pvParameters)
{
    xTaskCreate( vLED_Task,   	        /* ������  */
                 LED_TASK_NAME,     	/* ������    */
                 LED_STK_SIZE,        	/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,              	/* �������  */
                 LED_TASK_PRIO,         /* �������ȼ�*/
                 &xLEDTask_Handler );   /* ������  */
	
    xTaskCreate( vUSART_Task,   	    /* ������  */
                 USART_TASK_NAME,     	/* ������    */
                 USART_STK_SIZE,        /* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,              	/* �������  */
                 USART_TASK_PRIO,       /* �������ȼ�*/
                 &xUSARTTask_Handler ); /* ������  */
	
    vTaskDelete(xStartTask_Handler); //ɾ����ʼ����
}

/**
  * ��������: LED��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: 
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
  * ��������: ���ڴ�ӡ���
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: 
  */
void vUSART_Task (void *pvParameters)
{
   static float float_num=0.00;
    
   for(;;)
   { 
     float_num+=0.01f;
     printf("float_num��ֵΪ: %.4f\r\n",float_num);
     vTaskDelay(1000);
   }
}

/**
  * ��������: ������ʱ���жϻص�����
  * �������: htim��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: 
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    timer_count++;
}

/**
  * ��������: ���ڽ�����ɻص�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  // ���յ��Ĕ�����husartx�l�ͳ�ȥ
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
  *            HSE Frequency(Hz)              = 25000000 // ��8000000 �ⲿ����
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

  /* ʹ��PWRʱ��(Enable Power Control clock) */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* ���õ�ѹ�������ѹ����1(The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.)  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* ʹ���ⲿ������Ϊʱ��Դ(Enable HSE Oscillator and activate PLL with HSE as source)
  ����STM32F407IGx_Clock_Config.pngʱ������ͼ*/
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;        // �ⲿ����8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                          // ��HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                      // ��PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;              // PLLʱ��Դѡ��HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                   // 8��ƵMHz
  RCC_OscInitStruct.PLL.PLLN = 336;                                 // 336��Ƶ
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                       // 2��Ƶ���õ�168MHz��ʱ��
  RCC_OscInitStruct.PLL.PLLQ = 4;                                   // USB/SDIO/������������ȵ���PLL��Ƶϵ��
 
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;     // ϵͳʱ�ӣ�168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;            // AHBʱ�ӣ� 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;             // APB1ʱ�ӣ�42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;             // APB2ʱ�ӣ�84MHz
  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  // ʹ��CSS���ܣ�����ʹ���ⲿ�����ⲿʧЧʱ�����ڲ�ʱ��Դ��
  HAL_RCC_EnableCSS();                                            

    // HAL_RCC_GetHCLKFreq()/1000    1ms�ж�һ��
    // HAL_RCC_GetHCLKFreq()/100000	 10us�ж�һ��
	// HAL_RCC_GetHCLKFreq()/1000000 1us�ж�һ��
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                // ���ò�����ϵͳ�δ�ʱ��
  
    /* ϵͳ�δ�ʱ��ʱ��Դ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* ϵͳ�δ�ʱ���ж����ȼ����� */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
  
}

/**
*
*�����жϻص�����
*
*�жϷ�������ʹ����FreeRTOS��ϵͳʱ�Ӻ���vTaskDelay�����Է��ֽ���vTaskDelay������
*���ں���xTaskResumeAll�н����ٽ�κ������Ῠ����������봦:
*    if ( uxCriticalNesting == 1)
*    {
*        configASSERT( ( portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK ) == 0 );  
*     }
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin==KEY1_GPIO_PIN)
  {
    //HAL_Delay(20);/* ��ʱһС��ʱ�䣬�������� */
    if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL)
    {
       
      LED1_ON;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(KEY1_GPIO_PIN);
  }
  else if(GPIO_Pin==KEY2_GPIO_PIN)
  {
    //HAL_Delay(20);/* ��ʱһС��ʱ�䣬�������� */
    if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL)
    {
    
      LED2_ON;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(KEY2_GPIO_PIN);
  }
  else if(GPIO_Pin==KEY3_GPIO_PIN)
  {
    //HAL_Delay(20);/* ��ʱһС��ʱ�䣬�������� */
    if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL)
    {
    
      LED3_ON;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(KEY3_GPIO_PIN);
  }
  else if(GPIO_Pin==KEY4_GPIO_PIN)
  {
    //HAL_Delay(20);/* ��ʱһС��ʱ�䣬�������� */
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
    //HAL_Delay(20);/* ��ʱһС��ʱ�䣬�������� */
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
