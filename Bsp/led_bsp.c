#include "led_bsp.h"
#include "gpio.h"

/**
  * @brief          LED函数
  * @param[in]      led：	序号/0-10/
	* @param[in] 			state：状态/1/打开，/0/关闭
  * @retval         none
  */
//例：LED(1,1);
//		LED(2,0);
void LED(uint8_t led,uint8_t state)
{
    switch(led)
    {
    case 1:
        if(state==0)
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);
        }
        break;
    case 2:
        if(state==0)
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);
        }
        break;
    case 3:
        if(state==0)
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_RESET);
        }
        break;
    case 4:
        if(state==0)
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_RESET);
        }
        break;
    case 5:
        if(state==0)
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,GPIO_PIN_RESET);
        }
        break;
    case 6:
        if(state==0)
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);
        }
        break;
    case 7:
        if(state==0)
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_RESET);
        }
        break;
    case 8:
        if(state==0)
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_8,GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_8,GPIO_PIN_RESET);
        }
        break;
    case 9:
        if(state==0)
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET);
        }
        break;
    case 10:
        if(state==0)
        {
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);
        }
        break;
        //default:break;
    }
}
/**
  * @brief          关闭全部LED
  * @param[in]      none
  * @retval         none
  */
//例:LED_OFF();
void LED_OFF(void)
{
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_SET);
}

/**
  * @brief          打开全部LED
  * @param[in]      none
  * @retval         none
  */
//LED_ON();
void LED_ON(void)
{
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET);
}
