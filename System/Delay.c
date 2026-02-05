#include "stm32f10x.h"

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
	uint32_t tmp = SysTick->CTRL;			//保存控制寄存器
	SysTick->LOAD = 72 * xus;				//设置定时器重装值
	SysTick->VAL = 0x00;					//清空当前计数值
	SysTick->CTRL = 0x00000007;				//设置时钟源为HCLK，启动定时器，使能中断
	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
	SysTick->LOAD = 72000 - 1;				//恢复1ms中断 (72MHz/1000)
	SysTick->CTRL = tmp;					//恢复控制寄存器
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 
