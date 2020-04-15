/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
/* 开发板硬件bsp头文件 */
#include "./bsp/bsp_led.h"
#include "./bsp/bsp_usart.h"

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
extern void xPortSysTickHandler(void);
//systick中断服务函数
void SysTick_Handler(void)
{
#if (INCLUDE_xTaskGetSchedulerState  == 1 )

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
#endif  /* INCLUDE_xTaskGetSchedulerState */

        xPortSysTickHandler();

#if (INCLUDE_xTaskGetSchedulerState  == 1 )
    }

#endif  /* INCLUDE_xTaskGetSchedulerState */
}


/*********************************************************************************
  * @ 函数名  ： DEBUG_USART_IRQHandler
  * @ 功能说明： 串口中断服务函数
  * @ 参数    ： 无
  * @ 返回值  ： 无
  ********************************************************************************/
void DEBUG_USART_IRQHandler(void)
{
    debug_usart_rx_handle();
    debug_usart_tx_handle();
}


