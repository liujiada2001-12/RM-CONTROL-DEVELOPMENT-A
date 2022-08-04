#include "bsp_buzzer.h"
#include "main.h"

extern TIM_HandleTypeDef htim12;
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim12, psc);
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, pwm);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 0);
}
