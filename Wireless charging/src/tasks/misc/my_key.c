#include "my_key.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"

#define KEY1_CLOCK RCC_APB2Periph_GPIOE
#define KEY1_BANK GPIOE
#define KEY1_PIN GPIO_Pin_15
#define KEY2_CLOCK RCC_APB2Periph_GPIOC
#define KEY2_BANK GPIOC
#define KEY2_PIN GPIO_Pin_13
void key_init()
{
	RCC_APB2PeriphClockCmd(KEY1_CLOCK, ENABLE);
	RCC_APB2PeriphClockCmd(KEY2_CLOCK, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	//key1 PA0
	GPIO_InitStructure.GPIO_Pin = KEY1_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(KEY1_BANK, &GPIO_InitStructure);
	//key2 PC13
	GPIO_InitStructure.GPIO_Pin = KEY2_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(KEY2_BANK, &GPIO_InitStructure);
}
//down is 0, return TRUE
bool is_key1_down()
{	
	if(RESET == GPIO_ReadInputDataBit(KEY1_BANK,KEY1_PIN)) {
		return true;
	}
	
	return false;
}
bool is_key1_up()
{
	if(SET == GPIO_ReadInputDataBit(KEY1_BANK,KEY1_PIN)) {
		return true;
	}

	return false;
}
bool is_key2_down()
{
	if(RESET == GPIO_ReadInputDataBit(KEY2_BANK,KEY2_PIN)) {
		return true;
	}

	return false;
}
bool is_key2_up()
{
	if(SET == GPIO_ReadInputDataBit(KEY2_BANK,KEY2_PIN)) {
		return true;
	}
	
	return false;
}
//machao key
/*=============
低层按键（I/0）扫描函数，即低层按键设备驱动，只返回无键、短按和长按。具体双击不在此处判断。参考本人教材的例9-1，稍微有变化。教材中为连_发。
===============*/


unsigned char key_driver(void)
{
	static unsigned char key_state = key_state_0, key_time = 0;
	unsigned char key_press, key_return = N_key;
	key_press = GPIO_ReadInputDataBit(KEY1_BANK,KEY1_PIN);       // 读按键I/O电平
	
	switch (key_state) {
		case key_state_0:                              // 按键初始态
			if (!key_press) 
				key_state = key_state_1;             // 键被按下，状态转换到按键消抖和确认状态
				break;

		case key_state_1:                      // 按键消抖与确认态
			if (!key_press) {
				key_time = 0;                   // 
				key_state = key_state_2;          // 按键仍然处于按下，消抖完成，状态转换到按下键时间的计时状态，但返回的还是无键事件
			} else
				key_state = key_state_0;          // 按键已抬起，转换到按键初始态。此处完成和实现软件消抖，其实按键的按下和释放都在此消抖的。
			break;

		case key_state_2:
			if(key_press) {
				key_return = S_key;        // 此时按键释放，说明是产生一次短操作，回送S_key
				key_state = key_state_0;   // 转换到按键初始态
			} else if (++key_time >= 100) {     // 继续按下，计时加10ms（10ms为本函数循环执行间隔）
				key_return = L_key;        // 按下时间>1000ms，此按键为长按操作，返回长键事件
				key_state = key_state_3;   // 转换到等待按键释放状态
			}
			break;

		case key_state_3:                 // 等待按键释放状态，此状态只返回无按键事件
			if (key_press) 
				key_state = key_state_0;        //按键已释放，转换到按键初始态
			break;
	}        
	return key_return;
}

/*=============
中间层按键处理函数，调用低层函数一次，处理双击事件的判断，返回上层正确的无键、单键、双键、长键4个按键事件。
本函数由上层循环调用，间隔10ms
===============*/

unsigned char key_read(void)
{
    static unsigned char key_m = key_state_0, key_time_1 = 0;
    unsigned char key_return = N_key,key_temp;
    
    key_temp = key_driver();
    
    switch(key_m) {
		case key_state_0:
			if (key_temp == S_key ) {
				key_time_1 = 0;               // 第1次单击，不返回，到下个状态判断后面是否出现双击
				key_m = key_state_1;
			} else
				key_return = key_temp;        // 对于无键、长键，返回原事件
			break;

		case key_state_1:
			if (key_temp == S_key) {            // 又一次单击（间隔肯定<500ms）
				key_return = D_key;           // 返回双击键事件，回初始状态
				key_m = key_state_0;
			}
			else {                                  // 这里500ms内肯定读到的都是无键事件，因为长键>1000ms，在1s前低层返回的都是无键
				if(++key_time_1 >= 50) {
					key_return = S_key;      // 500ms内没有再次出现单键事件，返回上一次的单键事件
					key_m = key_state_0;     // 返回初始状态
				}
			}
			break;
    }
    return key_return;
}    

/*************************************************************************
 * Function Name: DelayResolution100us
 * Parameters: Int32U Dly
 *
 * Return: none
 *
 * Description: Delay ~ (arg * 100us)
 *
 *************************************************************************/
#define DLY_100US  450
void delay_resolution_100us(uint32_t wDly)
{
	for(; wDly; wDly--) {
		for(volatile uint32_t wJ = DLY_100US; wJ; wJ--) {
		}
	}
}

void delay_1ms(uint16_t hwTime)
{
	uint16_t hwI;
	while(hwTime--) {
		hwI = 12000;//
		while(hwI --);
	}
}

const key_t KEY = {
	.key_init = key_init,
	.is_key1_down = is_key1_down,
	.is_key1_up = is_key1_up,
	.is_key2_down = is_key2_down,
	.is_key2_up = is_key2_up,
};