/*
*	dusk@2010 initial version
*	miaofng@2010 update as bus driver type
*/

#include "config.h"
#include "stm32f10x.h"
#include "pwm.h"

#define TIMn TIM2

int pwm_init(const pwm_cfg_t *cfg)
{
	int f, div;
	RCC_ClocksTypeDef clks;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/*clock enable*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	RCC_GetClocksFreq(&clks);
	f = clks.PCLK1_Frequency;
	f <<= (clks.HCLK_Frequency == clks.PCLK1_Frequency) ? 0 : 1;
	div = f / cfg->hz / cfg -> fs;

	/* time base configuration */
	TIM_TimeBaseStructure.TIM_Period = cfg->fs;
	TIM_TimeBaseStructure.TIM_Prescaler = div - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIMn, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIMn, TIM_IT_CC1, ENABLE);//open the interrupt
	TIM_ARRPreloadConfig(TIMn, ENABLE);
	TIM_Cmd(TIMn, ENABLE);
	return 0;
}

int ch1_init(const pwm_cfg_t *cfg)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	pwm_cfg_t def = PWM_CFG_DEF;

	cfg = (cfg == NULL) ? &def : cfg;
	pwm_init(cfg);

	/*config pin*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*config ch*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = cfg -> fs >> 1; //default to 50%
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIMn, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIMn, TIM_OCPreload_Enable);
	return 0;
}

int ch2_init(const pwm_cfg_t *cfg)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	pwm_cfg_t def = PWM_CFG_DEF;

	cfg = (cfg == NULL) ? &def : cfg;
	pwm_init(cfg);

	/*config pin*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*config ch*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = cfg -> fs >> 1; //default to 50%
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIMn, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIMn, TIM_OCPreload_Enable);
	return 0;
}

int ch3_init(const pwm_cfg_t *cfg)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	pwm_cfg_t def = PWM_CFG_DEF;

	cfg = (cfg == NULL) ? &def : cfg;
	pwm_init(cfg);

	/*config pin*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*config ch*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = cfg -> fs >> 1; //default to 50%
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIMn, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIMn, TIM_OCPreload_Enable);
	return 0;
}

int ch4_init(const pwm_cfg_t *cfg)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	pwm_cfg_t def = PWM_CFG_DEF;

	cfg = (cfg == NULL) ? &def : cfg;
	pwm_init(cfg);

	/*config pin*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*config ch*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = cfg -> fs >> 1; //default to 50%
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIMn, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIMn, TIM_OCPreload_Enable);
	return 0;
}

int ch1_set(int val)
{
	TIM_SetCompare1(TIMn, val);
	return 0;
}

int ch2_set(int val)
{
	TIM_SetCompare2(TIMn, val);
	return 0;
}

int ch3_set(int val)
{
	TIM_SetCompare3(TIMn, val);
	return 0;
}

int ch4_set(int val)
{
	TIM_SetCompare4(TIMn, val);
	return 0;
}

const pwm_bus_t pwm2 = {
        .init = pwm_init,
};

const pwm_bus_t pwm21 = {
	.init = ch1_init,
	.set = ch1_set,
};

const pwm_bus_t pwm22 = {
	.init = ch2_init,
	.set = ch2_set,
};

const pwm_bus_t pwm23 = {
	.init = ch3_init,
	.set = ch3_set,
};

const pwm_bus_t pwm24 = {
	.init = ch4_init,
	.set = ch4_set,
};
