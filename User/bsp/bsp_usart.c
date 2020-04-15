/**
  ******************************************************************************
  * @file    Project/STM32F0xx_StdPeriph_Templates/main.c
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    05-December-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/


#include "./bsp_usart.h"
#include "./../cli/command_line.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void usart_send_byte(USART_TypeDef * pUSARTx, uint8_t ch);


/* -------------------------------------------------------------------- UART1 */
#if 0
/**
 * @brief  串口发送中断服务函数，在stm32f0xx_it.c串口中断中调用
 * @param  无
 * @retval 无
 */
void com2net_usart_tx_handle(void)
{
    /* 预留 */
}

/**
 * @brief  串口接收中断服务函数，在stm32f0xx_it.c串口中断中调用
 * @param  无
 * @retval 无
 */
void com2net_usart_rx_handle(void)
{
    uint8_t ucTemp;

    usart_send_byte(COM2NET_USARTx, 0x31);

    if (USART_GetITStatus(COM2NET_USARTx, USART_IT_RXNE) != RESET) {
        ucTemp = USART_ReceiveData(COM2NET_USARTx);
        //printf("%02x", ucTemp);

#include "./../app/app_com2net.h"
        QUEUE_IN(com2net_queue, ucTemp);
        com2net_com_rx_flag = TRUE;     /* 标记串口在接收数据 */
    }
}

/**
 * @brief  配置嵌套向量中断控制器NVIC
 * @param  无
 * @retval 无
 */
static void com2net_usart_nvic_config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 嵌套向量中断控制器组选择 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* 配置USART为中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = COM2NET_USART_IRQ;
    /* 抢断优先级*/
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    /* 子优先级 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    /* 使能中断 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* 初始化配置NVIC */
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  USART GPIO 配置,工作参数配置
 * @param  无
 * @retval 无
 */
void com2net_usart_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // 打开串口GPIO的时钟
    COM2NET_USART_GPIO_APBxClkCmd(COM2NET_USART_GPIO_CLK, ENABLE);

    // 打开串口外设的时钟
    COM2NET_USART_APBxClkCmd(COM2NET_USART_CLK, ENABLE);

    // 将USART Tx的GPIO配置为推挽复用模式
    GPIO_InitStructure.GPIO_Pin = COM2NET_USART_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(COM2NET_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    // 将USART Rx的GPIO配置为浮空输入模式
    GPIO_InitStructure.GPIO_Pin = COM2NET_USART_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(COM2NET_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    // 配置串口的工作参数
    // 配置波特率
    USART_InitStructure.USART_BaudRate = COM2NET_USART_BAUDRATE;
    // 配置 针数据字长
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    // 配置停止位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    // 配置校验位
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    // 配置硬件流控制
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    // 配置工作模式，收发一起
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    // 完成串口的初始化配置
    USART_Init(COM2NET_USARTx, &USART_InitStructure);

    // 串口中断优先级配置
    com2net_usart_nvic_config();

    // 使能串口接收中断
    USART_ITConfig(COM2NET_USARTx, USART_IT_RXNE, ENABLE);

    // 使能串口
    USART_Cmd(COM2NET_USARTx, ENABLE);

    // 清除发送完成标志
    //USART_ClearFlag(COM2NET_USARTx, USART_FLAG_TC);
}

/* -------------------------------------------------------------------- UART2 */
/**
 * @brief  串口发送中断服务函数，在stm32f0xx_it.c串口中断中调用
 * @param  无
 * @retval 无
 */
void gsm_wifi_usart_tx_handle(void)
{
    /* 预留 */
}

/**
 * @brief  串口接收中断服务函数，在stm32f0xx_it.c串口中断中调用
 * @param  无
 * @retval 无
 */
void gsm_wifi_usart_rx_handle(void)
{

}

/**
 * @brief  配置嵌套向量中断控制器NVIC
 * @param  无
 * @retval 无
 */
static void gsm_wifi_usart_nvic_config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 嵌套向量中断控制器组选择 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* 配置USART为中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = GSM_WIFI_USART_IRQ;
    /* 抢断优先级*/
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    /* 子优先级 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    /* 使能中断 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* 初始化配置NVIC */
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  USART GPIO 配置,工作参数配置
 * @param  无
 * @retval 无
 */
void gsm_wifi_usart_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // 打开串口GPIO的时钟
    DEBUG_USART_GPIO_APBxClkCmd(GSM_WIFI_USART_GPIO_CLK, ENABLE);

    // 打开串口外设的时钟
    DEBUG_USART_APBxClkCmd(GSM_WIFI_USART_CLK, ENABLE);

    // 将USART Tx的GPIO配置为推挽复用模式
    GPIO_InitStructure.GPIO_Pin = GSM_WIFI_USART_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GSM_WIFI_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    // 将USART Rx的GPIO配置为浮空输入模式
    GPIO_InitStructure.GPIO_Pin = GSM_WIFI_USART_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GSM_WIFI_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    // 配置串口的工作参数
    // 配置波特率
    USART_InitStructure.USART_BaudRate = GSM_WIFI_USART_BAUDRATE;
    // 配置 针数据字长
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    // 配置停止位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    // 配置校验位
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    // 配置硬件流控制
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    // 配置工作模式，收发一起
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    // 完成串口的初始化配置
    USART_Init(GSM_WIFI_USARTx, &USART_InitStructure);

    // 串口中断优先级配置
    gsm_wifi_usart_nvic_config();

    // 使能串口接收中断
    USART_ITConfig(GSM_WIFI_USARTx, USART_IT_RXNE, ENABLE);

    // 使能串口
    USART_Cmd(GSM_WIFI_USARTx, ENABLE);

    // 清除发送完成标志
    //USART_ClearFlag(GSM_WIFI_USARTx, USART_FLAG_TC);
}

/* -------------------------------------------------------------------- UART3 */
/**
 * @brief  串口发送中断服务函数，在stm32f0xx_it.c串口中断中调用
 * @param  无
 * @retval 无
 */
void pstn_wifi_usart_tx_handle(void)
{
    /* 预留 */
}

/**
 * @brief  串口接收中断服务函数，在stm32f0xx_it.c串口中断中调用
 * @param  无
 * @retval 无
 */
void pstn_wifi_usart_rx_handle(void)
{

}

/**
 * @brief  配置嵌套向量中断控制器NVIC
 * @param  无
 * @retval 无
 */
static void pstn_wifi_usart_nvic_config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 嵌套向量中断控制器组选择 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* 配置USART为中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = PSTN_WIFI_USART_IRQ;
    /* 抢断优先级*/
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    /* 子优先级 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    /* 使能中断 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* 初始化配置NVIC */
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  USART GPIO 配置,工作参数配置
 * @param  无
 * @retval 无
 */
void pstn_wifi_usart_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // 打开串口GPIO的时钟
    DEBUG_USART_GPIO_APBxClkCmd(PSTN_WIFI_USART_GPIO_CLK, ENABLE);

    // 打开串口外设的时钟
    DEBUG_USART_APBxClkCmd(PSTN_WIFI_USART_CLK, ENABLE);

    // 将USART Tx的GPIO配置为推挽复用模式
    GPIO_InitStructure.GPIO_Pin = PSTN_WIFI_USART_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PSTN_WIFI_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    // 将USART Rx的GPIO配置为浮空输入模式
    GPIO_InitStructure.GPIO_Pin = PSTN_WIFI_USART_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(PSTN_WIFI_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    // 配置串口的工作参数
    // 配置波特率
    USART_InitStructure.USART_BaudRate = PSTN_WIFI_USART_BAUDRATE;
    // 配置 针数据字长
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    // 配置停止位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    // 配置校验位
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    // 配置硬件流控制
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    // 配置工作模式，收发一起
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    // 完成串口的初始化配置
    USART_Init(PSTN_WIFI_USARTx, &USART_InitStructure);

    // 串口中断优先级配置
    pstn_wifi_usart_nvic_config();

    // 使能串口接收中断
    USART_ITConfig(PSTN_WIFI_USARTx, USART_IT_RXNE, ENABLE);

    // 使能串口
    USART_Cmd(PSTN_WIFI_USARTx, ENABLE);

    // 清除发送完成标志
    //USART_ClearFlag(PSTN_WIFI_USARTx, USART_FLAG_TC);
}
#endif
/* -------------------------------------------------------------------- UART4 */
/**
 * @brief  串口发送中断服务函数，在stm32f0xx_it.c串口中断中调用
 * @param  无
 * @retval 无
 */
void debug_usart_tx_handle(void)
{
    /* 预留 */
}

#include "./../../FreeRTOS/include/task.h"

/**
 * @brief  串口接收中断服务函数，在stm32f0xx_it.c串口中断中调用
 * @param  无
 * @retval 无
 */
void debug_usart_rx_handle(void)
{
    uint8_t ucTemp;
    uint32_t ulReturn;

    /* 进入临界段，临界段可以嵌套 */
    ulReturn = taskENTER_CRITICAL_FROM_ISR();

    if (USART_GetITStatus(DEBUG_USARTx, USART_IT_RXNE) != RESET) {
        ucTemp = USART_ReceiveData(DEBUG_USARTx);
        xQueueSendFromISR(cli_queue, &ucTemp, 0);
        //printf("%02x", ucTemp);
    }

    /* 退出临界段 */
    taskEXIT_CRITICAL_FROM_ISR(ulReturn);
}

/**
 * @brief  配置嵌套向量中断控制器NVIC
 * @param  无
 * @retval 无
 */
static void debug_usart_nvic_config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 嵌套向量中断控制器组选择 */
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* 配置USART为中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
    /* 抢断优先级*/
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
    /* 子优先级 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    /* 使能中断 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* 初始化配置NVIC */
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  USART GPIO 配置,工作参数配置
 * @param  无
 * @retval 无
 */
void debug_usart_init(uint32_t baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // 打开串口GPIO的时钟
    DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);

    // 打开串口外设的时钟
    DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

    // 将USART Tx的GPIO配置为推挽复用模式
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    // 将USART Rx的GPIO配置为浮空输入模式
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    // 配置串口的工作参数
    // 配置波特率
    USART_InitStructure.USART_BaudRate = baud;
    // 配置 针数据字长
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    // 配置停止位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    // 配置校验位
    USART_InitStructure.USART_Parity = USART_Parity_No;
    // 配置硬件流控制
    USART_InitStructure.USART_HardwareFlowControl =
        USART_HardwareFlowControl_None;
    // 配置工作模式，收发一起
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    // 完成串口的初始化配置
    USART_Init(DEBUG_USARTx, &USART_InitStructure);

    // 串口中断优先级配置
    debug_usart_nvic_config();

    // 使能串口接收中断
    USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);

    // 使能串口
    USART_Cmd(DEBUG_USARTx, ENABLE);
}

/* -------------------------------------------------------------------- UART5 */
#if 0
/**
 * @brief  串口发送中断服务函数，在stm32f0xx_it.c串口中断中调用
 * @param  无
 * @retval 无
 */
void rf_usart_tx_handle(void)
{
}

/**
 * @brief  串口接收中断服务函数，在stm32f0xx_it.c串口中断中调用
 * @param  无
 * @retval 无
 */
void rf_usart_rx_handle(void)
{

}

void rf_usart_tx_int_enable(void)
{
    USART_ITConfig(RF_USARTx, USART_IT_TXE, ENABLE);
}

void rf_usart_tx_int_disable(void)
{
    USART_ITConfig(RF_USARTx, USART_IT_TXE, DISABLE);
}

/**
 * @brief  USART GPIO 配置,工作参数配置
 * @param  无
 * @retval 无
 */
void rf_usart_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(RF_USART_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RF_USART_CLK, ENABLE);

    /* Configure UART4 Rx as input floating */
    GPIO_InitStructure.GPIO_Pin = RF_USART_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(RF_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    /* Configure UART4 Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = RF_USART_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(RF_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RF_USART_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* UART4 configuration ---------------------------------------------------*/
    /* UART4configured as follow:
          - BaudRate = 9600 baud
          - Word Length = 8 Bits
          - One Stop Bit
          - No parity
          - Hardware flow control disabled (RTS and CTS signals)
          - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate = RF_USART_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* Configure UART4 */
    USART_Init(RF_USARTx, &USART_InitStructure);

    /* Enable UART4 Receive and Transmit interrupts */
    USART_ITConfig(RF_USARTx, USART_IT_RXNE, ENABLE);

    /* Enable the UART4 */
    USART_Cmd(RF_USARTx, ENABLE);

}

/**
 * @brief  串口发送一个字节
 * @param  byData 发送数据
 * @retval 无
 */
void rf_usart_send_byte(uint8_t byData)
{

}
#endif
/* -------------------------------------------------------------------------- */

/**
 * @brief  发送一个字节
 * @param  无
 * @retval 无
 */
void usart_send_byte(USART_TypeDef * pUSARTx, uint8_t ch)
{
    /* 发送一个字节数据到USART */
    USART_SendData(pUSARTx, ch);

    /* 等待发送数据寄存器为空 */
    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}

/**
 * @brief  发送8位的数组
 * @param  无
 * @retval 无
 */
void usart_send_array(USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num)
{
    uint8_t i;

    for (i = 0; i < num; i++) {
        /* 发送一个字节数据到USART */
        usart_send_byte(pUSARTx, array[i]);

    }

    /* 等待发送完成 */
    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET);
}

/**
 * @brief  发送字符串
 * @param  无
 * @retval 无
 */
void usart_send_string(USART_TypeDef * pUSARTx, char *str)
{
    unsigned int k = 0;

    do {
        usart_send_byte(pUSARTx, *(str + k));
        k++;
    } while (*(str + k) != '\0');

    /* 等待发送完成 */
    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET)
    {}
}

/**
 * @brief  发送一个16位数
 * @param  无
 * @retval 无
 */
void Usart_SendHalfWord(USART_TypeDef * pUSARTx, uint16_t ch)
{
    uint8_t temp_h, temp_l;

    /* 取出高八位 */
    temp_h = (ch & 0XFF00) >> 8;
    /* 取出低八位 */
    temp_l = ch & 0XFF;

    /* 发送高八位 */
    USART_SendData(pUSARTx, temp_h);

    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);

    /* 发送低八位 */
    USART_SendData(pUSARTx, temp_l);

    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}

/**
 * @brief  重定向c库函数printf到串口，重定向后可使用printf函数
 * @param  无
 * @retval 无
 */
int fputc(int ch, FILE *f)
{
    /* 发送一个字节数据到串口 */
    USART_SendData(DEBUG_USARTx, (uint8_t) ch);

    /* 等待发送完毕 */
    while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);

    return (ch);
}

/**
 * @brief  重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
 * @param  无
 * @retval 无
 */
int fgetc(FILE *f)
{
    /* 等待串口输入数据 */
    while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET);

    return (int)USART_ReceiveData(DEBUG_USARTx);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

