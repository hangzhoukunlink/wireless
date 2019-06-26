/*
*	jiamao.gu@2013 first version
*/

#include "config.h"
#include "stm32f10x.h"
#include "pwm.h"

#define TIMn TIM1

static int pwm_init(const pwm_cfg_t *cfg)
{
	int f, div;
	RCC_ClocksTypeDef clks;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/*clock enable*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	RCC_GetClocksFreq(&clks);
	f = clks.PCLK2_Frequency;
	f <<= (clks.HCLK_Frequency == clks.PCLK2_Frequency) ? 0 : 1;
	div = f / cfg->hz / cfg -> fs;

	/* time base configuration */
	TIM_TimeBaseStructure.TIM_Period = cfg->fs;
	TIM_TimeBaseStructure.TIM_Prescaler = div - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIMn, &TIM_TimeBaseStructure);
	//TIM_ITConfig(TIMn, TIM_IT_CC1, ENABLE);//open the interrupt
	TIM_ARRPreloadConfig(TIMn, ENABLE);
	TIM_Cmd(TIMn, ENABLE);
	//must add !!!
	TIM_CtrlPWMOutputs(TIMn, ENABLE);
	return 0;
}

static int ch1_init(const pwm_cfg_t *cfg)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	pwm_cfg_t def = PWM_CFG_DEF;

	cfg = (cfg == NULL) ? &def : cfg;
	pwm_init(cfg);

	/*config pin*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
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

static int ch1_set(int val)
{
	TIM_SetCompare1(TIMn, val);
	return 0;
}

const pwm_bus_t pwm1 = {
        .init = pwm_init,
};

const pwm_bus_t pwm11 = {
	.init = ch1_init,
	.set = ch1_set,
};

