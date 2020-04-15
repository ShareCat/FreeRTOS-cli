/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

/* FreeRTOSͷ�ļ� */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
/* ������Ӳ��bspͷ�ļ� */
#include "./bsp/bsp_led.h"
#include "./bsp/bsp_usart.h"

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
extern void xPortSysTickHandler(void);
//systick�жϷ�����
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
  * @ ������  �� DEBUG_USART_IRQHandler
  * @ ����˵���� �����жϷ�����
  * @ ����    �� ��
  * @ ����ֵ  �� ��
  ********************************************************************************/
void DEBUG_USART_IRQHandler(void)
{
    debug_usart_rx_handle();
    debug_usart_tx_handle();
}


