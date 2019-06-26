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
�Ͳ㰴����I/0��ɨ�躯�������Ͳ㰴���豸������ֻ�����޼����̰��ͳ���������˫�����ڴ˴��жϡ��ο����˽̲ĵ���9-1����΢�б仯���̲���Ϊ��_����
===============*/


unsigned char key_driver(void)
{
	static unsigned char key_state = key_state_0, key_time = 0;
	unsigned char key_press, key_return = N_key;
	key_press = GPIO_ReadInputDataBit(KEY1_BANK,KEY1_PIN);       // ������I/O��ƽ
	
	switch (key_state) {
		case key_state_0:                              // ������ʼ̬
			if (!key_press) 
				key_state = key_state_1;             // �������£�״̬ת��������������ȷ��״̬
				break;

		case key_state_1:                      // ����������ȷ��̬
			if (!key_press) {
				key_time = 0;                   // 
				key_state = key_state_2;          // ������Ȼ���ڰ��£�������ɣ�״̬ת�������¼�ʱ��ļ�ʱ״̬�������صĻ����޼��¼�
			} else
				key_state = key_state_0;          // ������̧��ת����������ʼ̬���˴���ɺ�ʵ�������������ʵ�����İ��º��ͷŶ��ڴ������ġ�
			break;

		case key_state_2:
			if(key_press) {
				key_return = S_key;        // ��ʱ�����ͷţ�˵���ǲ���һ�ζ̲���������S_key
				key_state = key_state_0;   // ת����������ʼ̬
			} else if (++key_time >= 100) {     // �������£���ʱ��10ms��10msΪ������ѭ��ִ�м����
				key_return = L_key;        // ����ʱ��>1000ms���˰���Ϊ�������������س����¼�
				key_state = key_state_3;   // ת�����ȴ������ͷ�״̬
			}
			break;

		case key_state_3:                 // �ȴ������ͷ�״̬����״ֻ̬�����ް����¼�
			if (key_press) 
				key_state = key_state_0;        //�������ͷţ�ת����������ʼ̬
			break;
	}        
	return key_return;
}

/*=============
�м�㰴�������������õͲ㺯��һ�Σ�����˫���¼����жϣ������ϲ���ȷ���޼���������˫��������4�������¼���
���������ϲ�ѭ�����ã����10ms
===============*/

unsigned char key_read(void)
{
    static unsigned char key_m = key_state_0, key_time_1 = 0;
    unsigned char key_return = N_key,key_temp;
    
    key_temp = key_driver();
    
    switch(key_m) {
		case key_state_0:
			if (key_temp == S_key ) {
				key_time_1 = 0;               // ��1�ε����������أ����¸�״̬�жϺ����Ƿ����˫��
				key_m = key_state_1;
			} else
				key_return = key_temp;        // �����޼�������������ԭ�¼�
			break;

		case key_state_1:
			if (key_temp == S_key) {            // ��һ�ε���������϶�<500ms��
				key_return = D_key;           // ����˫�����¼����س�ʼ״̬
				key_m = key_state_0;
			}
			else {                                  // ����500ms�ڿ϶������Ķ����޼��¼�����Ϊ����>1000ms����1sǰ�Ͳ㷵�صĶ����޼�
				if(++key_time_1 >= 50) {
					key_return = S_key;      // 500ms��û���ٴγ��ֵ����¼���������һ�εĵ����¼�
					key_m = key_state_0;     // ���س�ʼ״̬
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