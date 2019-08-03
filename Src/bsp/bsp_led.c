/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp/bsp_led.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 板载LED灯IO引脚初始化.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：使用宏定义方法代替具体引脚号，方便程序移植，只要简单修改bsp_led.h
  *           文件相关宏定义就可以方便修改引脚。

  * STM32F407IGT6有9个GPIO(通用输入输出)外设，GPIOA、GPIB、GPIOC、GPIOD、GPIOE、GPIOF、GPIOG、GPIOH、GPIOI。
    每个外设有16个引脚，如PA0、PA1、PA2...PA15，GPIOI有12个引脚，一共引脚数是140个。
    LED灯1接的是GPIOH的第9号引脚
    LED灯2接的是GPIOE的第5号引脚
    LED灯3接的是GPIOE的第6号引脚
*/
void LED_GPIO_Init(void)
{
   /* 定义GPIO结构体变量 */
  GPIO_InitTypeDef GPIO_InitStruct;
	
	/* 使能(开启)LED引脚对应IO端口时钟 */  
  LED1_RCC_CLK_ENABLE();
  LED2_RCC_CLK_ENABLE();
  LED3_RCC_CLK_ENABLE();
  
  // PH9
  /* 设定LED1对应引脚IO编号 */
  GPIO_InitStruct.Pin = LED1_GPIO_PIN;
  /* 设定LED1对应引脚IO为输出模式 */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  /* 设定LED1对应引脚IO操作速度 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  /* 初始化LED1对应引脚GPIOH */
  HAL_GPIO_Init(LED1_GPIO, &GPIO_InitStruct);
  
  // PE5
  /* 设定LED2对应引脚IO编号 */
  GPIO_InitStruct.Pin = LED2_GPIO_PIN;
  /* 设定LED2对应引脚IO为输出模式 */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  /* 设定LED2对应引脚IO操作速度 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  /* 初始化LED2对应引脚IO */
  HAL_GPIO_Init(LED2_GPIO, &GPIO_InitStruct);
  
  // PE6 
  /* 设定LED3对应引脚IO编号 */
  GPIO_InitStruct.Pin = LED3_GPIO_PIN; 
  /* 设定LED3对应引脚IO为输出模式 */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  /* 设定LED3对应引脚IO操作速度 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  /* 初始化LED3对应引脚IO */
  HAL_GPIO_Init(LED3_GPIO, &GPIO_InitStruct);
  
  // LED 全灭灯
    /* 配置LED1引脚输出电压 */
  HAL_GPIO_WritePin(LED1_GPIO, LED1_GPIO_PIN, GPIO_PIN_RESET);
  /* 配置LED2引脚输出电压 */
  HAL_GPIO_WritePin(LED2_GPIO, LED2_GPIO_PIN, GPIO_PIN_RESET);
  /* 配置LED3引脚输出电压 */
  HAL_GPIO_WritePin(LED2_GPIO, LED3_GPIO_PIN, GPIO_PIN_RESET);  
  
}

/**
  * 函数功能: 设置板载LED灯的状态
  * 输入参数: LEDx:其中x可甚至为(1,2,3)用来选择对应的LED灯
  *           state:设置LED灯的输出状态。
  *           可选值:LED_OFF:LED灯灭
  *                   LED_ON:LED灯亮 
  *                   LED_TOGGLE:反转LED
  * 返 回 值: 无
  * 说    明: 该函数使用类似标准库函数的编程方法，方便理解标准库函数编程思想。
  */
void LEDx_StateSet(uint8_t LEDx,LEDState_TypeDef state)
{
  /* 检查输入参数是否合法 */
  assert_param(IS_LED_TYPEDEF(LEDx));
  assert_param(IS_LED_STATE(state));
  
  /* 判断设置的LED灯状态，如果设置为LED灯灭 */
  if(state==LED_OFF)
  {
    if(LEDx & LED1)            
      LED1_OFF;/* LED1灭 */
    
    if(LEDx & LED2)
      LED2_OFF;/* LED2灭 */
    
    if(LEDx & LED3)
      LED3_OFF;/* LED3灭 */    
  }
  else if(state==LED_ON) /* 设置LED灯为亮 */
  {
    if(LEDx & LED1)
      LED1_ON;/* LED1亮 */
    
    if(LEDx & LED2)
      LED2_ON;/* LED2亮 */
    
    if(LEDx & LED3)
      LED3_ON;/* LED3亮 */ 
  }
  else
  {
    if(LEDx & LED1)
      LED1_TOGGLE;/* 设置引脚输出反转 */
    
    if(LEDx & LED2)
      LED2_TOGGLE;/* 设置引脚输出反转 */
    
    if(LEDx & LED3)
      LED3_TOGGLE;/* 设置引脚输出反转 */ 
  }
}
