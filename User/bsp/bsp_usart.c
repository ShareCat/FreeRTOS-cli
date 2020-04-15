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
 * @brief  ���ڷ����жϷ���������stm32f0xx_it.c�����ж��е���
 * @param  ��
 * @retval ��
 */
void com2net_usart_tx_handle(void)
{
    /* Ԥ�� */
}

/**
 * @brief  ���ڽ����жϷ���������stm32f0xx_it.c�����ж��е���
 * @param  ��
 * @retval ��
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
        com2net_com_rx_flag = TRUE;     /* ��Ǵ����ڽ������� */
    }
}

/**
 * @brief  ����Ƕ�������жϿ�����NVIC
 * @param  ��
 * @retval ��
 */
static void com2net_usart_nvic_config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Ƕ�������жϿ�������ѡ�� */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* ����USARTΪ�ж�Դ */
    NVIC_InitStructure.NVIC_IRQChannel = COM2NET_USART_IRQ;
    /* �������ȼ�*/
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    /* �����ȼ� */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    /* ʹ���ж� */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* ��ʼ������NVIC */
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  USART GPIO ����,������������
 * @param  ��
 * @retval ��
 */
void com2net_usart_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // �򿪴���GPIO��ʱ��
    COM2NET_USART_GPIO_APBxClkCmd(COM2NET_USART_GPIO_CLK, ENABLE);

    // �򿪴��������ʱ��
    COM2NET_USART_APBxClkCmd(COM2NET_USART_CLK, ENABLE);

    // ��USART Tx��GPIO����Ϊ���츴��ģʽ
    GPIO_InitStructure.GPIO_Pin = COM2NET_USART_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(COM2NET_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    // ��USART Rx��GPIO����Ϊ��������ģʽ
    GPIO_InitStructure.GPIO_Pin = COM2NET_USART_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(COM2NET_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    // ���ô��ڵĹ�������
    // ���ò�����
    USART_InitStructure.USART_BaudRate = COM2NET_USART_BAUDRATE;
    // ���� �������ֳ�
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    // ����ֹͣλ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    // ����У��λ
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    // ����Ӳ��������
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    // ���ù���ģʽ���շ�һ��
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    // ��ɴ��ڵĳ�ʼ������
    USART_Init(COM2NET_USARTx, &USART_InitStructure);

    // �����ж����ȼ�����
    com2net_usart_nvic_config();

    // ʹ�ܴ��ڽ����ж�
    USART_ITConfig(COM2NET_USARTx, USART_IT_RXNE, ENABLE);

    // ʹ�ܴ���
    USART_Cmd(COM2NET_USARTx, ENABLE);

    // ���������ɱ�־
    //USART_ClearFlag(COM2NET_USARTx, USART_FLAG_TC);
}

/* -------------------------------------------------------------------- UART2 */
/**
 * @brief  ���ڷ����жϷ���������stm32f0xx_it.c�����ж��е���
 * @param  ��
 * @retval ��
 */
void gsm_wifi_usart_tx_handle(void)
{
    /* Ԥ�� */
}

/**
 * @brief  ���ڽ����жϷ���������stm32f0xx_it.c�����ж��е���
 * @param  ��
 * @retval ��
 */
void gsm_wifi_usart_rx_handle(void)
{

}

/**
 * @brief  ����Ƕ�������жϿ�����NVIC
 * @param  ��
 * @retval ��
 */
static void gsm_wifi_usart_nvic_config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Ƕ�������жϿ�������ѡ�� */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* ����USARTΪ�ж�Դ */
    NVIC_InitStructure.NVIC_IRQChannel = GSM_WIFI_USART_IRQ;
    /* �������ȼ�*/
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    /* �����ȼ� */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    /* ʹ���ж� */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* ��ʼ������NVIC */
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  USART GPIO ����,������������
 * @param  ��
 * @retval ��
 */
void gsm_wifi_usart_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // �򿪴���GPIO��ʱ��
    DEBUG_USART_GPIO_APBxClkCmd(GSM_WIFI_USART_GPIO_CLK, ENABLE);

    // �򿪴��������ʱ��
    DEBUG_USART_APBxClkCmd(GSM_WIFI_USART_CLK, ENABLE);

    // ��USART Tx��GPIO����Ϊ���츴��ģʽ
    GPIO_InitStructure.GPIO_Pin = GSM_WIFI_USART_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GSM_WIFI_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    // ��USART Rx��GPIO����Ϊ��������ģʽ
    GPIO_InitStructure.GPIO_Pin = GSM_WIFI_USART_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GSM_WIFI_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    // ���ô��ڵĹ�������
    // ���ò�����
    USART_InitStructure.USART_BaudRate = GSM_WIFI_USART_BAUDRATE;
    // ���� �������ֳ�
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    // ����ֹͣλ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    // ����У��λ
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    // ����Ӳ��������
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    // ���ù���ģʽ���շ�һ��
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    // ��ɴ��ڵĳ�ʼ������
    USART_Init(GSM_WIFI_USARTx, &USART_InitStructure);

    // �����ж����ȼ�����
    gsm_wifi_usart_nvic_config();

    // ʹ�ܴ��ڽ����ж�
    USART_ITConfig(GSM_WIFI_USARTx, USART_IT_RXNE, ENABLE);

    // ʹ�ܴ���
    USART_Cmd(GSM_WIFI_USARTx, ENABLE);

    // ���������ɱ�־
    //USART_ClearFlag(GSM_WIFI_USARTx, USART_FLAG_TC);
}

/* -------------------------------------------------------------------- UART3 */
/**
 * @brief  ���ڷ����жϷ���������stm32f0xx_it.c�����ж��е���
 * @param  ��
 * @retval ��
 */
void pstn_wifi_usart_tx_handle(void)
{
    /* Ԥ�� */
}

/**
 * @brief  ���ڽ����жϷ���������stm32f0xx_it.c�����ж��е���
 * @param  ��
 * @retval ��
 */
void pstn_wifi_usart_rx_handle(void)
{

}

/**
 * @brief  ����Ƕ�������жϿ�����NVIC
 * @param  ��
 * @retval ��
 */
static void pstn_wifi_usart_nvic_config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Ƕ�������жϿ�������ѡ�� */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* ����USARTΪ�ж�Դ */
    NVIC_InitStructure.NVIC_IRQChannel = PSTN_WIFI_USART_IRQ;
    /* �������ȼ�*/
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    /* �����ȼ� */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    /* ʹ���ж� */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* ��ʼ������NVIC */
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  USART GPIO ����,������������
 * @param  ��
 * @retval ��
 */
void pstn_wifi_usart_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // �򿪴���GPIO��ʱ��
    DEBUG_USART_GPIO_APBxClkCmd(PSTN_WIFI_USART_GPIO_CLK, ENABLE);

    // �򿪴��������ʱ��
    DEBUG_USART_APBxClkCmd(PSTN_WIFI_USART_CLK, ENABLE);

    // ��USART Tx��GPIO����Ϊ���츴��ģʽ
    GPIO_InitStructure.GPIO_Pin = PSTN_WIFI_USART_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PSTN_WIFI_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    // ��USART Rx��GPIO����Ϊ��������ģʽ
    GPIO_InitStructure.GPIO_Pin = PSTN_WIFI_USART_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(PSTN_WIFI_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    // ���ô��ڵĹ�������
    // ���ò�����
    USART_InitStructure.USART_BaudRate = PSTN_WIFI_USART_BAUDRATE;
    // ���� �������ֳ�
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    // ����ֹͣλ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    // ����У��λ
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    // ����Ӳ��������
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    // ���ù���ģʽ���շ�һ��
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    // ��ɴ��ڵĳ�ʼ������
    USART_Init(PSTN_WIFI_USARTx, &USART_InitStructure);

    // �����ж����ȼ�����
    pstn_wifi_usart_nvic_config();

    // ʹ�ܴ��ڽ����ж�
    USART_ITConfig(PSTN_WIFI_USARTx, USART_IT_RXNE, ENABLE);

    // ʹ�ܴ���
    USART_Cmd(PSTN_WIFI_USARTx, ENABLE);

    // ���������ɱ�־
    //USART_ClearFlag(PSTN_WIFI_USARTx, USART_FLAG_TC);
}
#endif
/* -------------------------------------------------------------------- UART4 */
/**
 * @brief  ���ڷ����жϷ���������stm32f0xx_it.c�����ж��е���
 * @param  ��
 * @retval ��
 */
void debug_usart_tx_handle(void)
{
    /* Ԥ�� */
}

#include "./../../FreeRTOS/include/task.h"

/**
 * @brief  ���ڽ����жϷ���������stm32f0xx_it.c�����ж��е���
 * @param  ��
 * @retval ��
 */
void debug_usart_rx_handle(void)
{
    uint8_t ucTemp;
    uint32_t ulReturn;

    /* �����ٽ�Σ��ٽ�ο���Ƕ�� */
    ulReturn = taskENTER_CRITICAL_FROM_ISR();

    if (USART_GetITStatus(DEBUG_USARTx, USART_IT_RXNE) != RESET) {
        ucTemp = USART_ReceiveData(DEBUG_USARTx);
        xQueueSendFromISR(cli_queue, &ucTemp, 0);
        //printf("%02x", ucTemp);
    }

    /* �˳��ٽ�� */
    taskEXIT_CRITICAL_FROM_ISR(ulReturn);
}

/**
 * @brief  ����Ƕ�������жϿ�����NVIC
 * @param  ��
 * @retval ��
 */
static void debug_usart_nvic_config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Ƕ�������жϿ�������ѡ�� */
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* ����USARTΪ�ж�Դ */
    NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
    /* �������ȼ�*/
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
    /* �����ȼ� */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    /* ʹ���ж� */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* ��ʼ������NVIC */
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  USART GPIO ����,������������
 * @param  ��
 * @retval ��
 */
void debug_usart_init(uint32_t baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // �򿪴���GPIO��ʱ��
    DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);

    // �򿪴��������ʱ��
    DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

    // ��USART Tx��GPIO����Ϊ���츴��ģʽ
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    // ��USART Rx��GPIO����Ϊ��������ģʽ
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    // ���ô��ڵĹ�������
    // ���ò�����
    USART_InitStructure.USART_BaudRate = baud;
    // ���� �������ֳ�
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    // ����ֹͣλ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    // ����У��λ
    USART_InitStructure.USART_Parity = USART_Parity_No;
    // ����Ӳ��������
    USART_InitStructure.USART_HardwareFlowControl =
        USART_HardwareFlowControl_None;
    // ���ù���ģʽ���շ�һ��
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    // ��ɴ��ڵĳ�ʼ������
    USART_Init(DEBUG_USARTx, &USART_InitStructure);

    // �����ж����ȼ�����
    debug_usart_nvic_config();

    // ʹ�ܴ��ڽ����ж�
    USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);

    // ʹ�ܴ���
    USART_Cmd(DEBUG_USARTx, ENABLE);
}

/* -------------------------------------------------------------------- UART5 */
#if 0
/**
 * @brief  ���ڷ����жϷ���������stm32f0xx_it.c�����ж��е���
 * @param  ��
 * @retval ��
 */
void rf_usart_tx_handle(void)
{
}

/**
 * @brief  ���ڽ����жϷ���������stm32f0xx_it.c�����ж��е���
 * @param  ��
 * @retval ��
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
 * @brief  USART GPIO ����,������������
 * @param  ��
 * @retval ��
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
 * @brief  ���ڷ���һ���ֽ�
 * @param  byData ��������
 * @retval ��
 */
void rf_usart_send_byte(uint8_t byData)
{

}
#endif
/* -------------------------------------------------------------------------- */

/**
 * @brief  ����һ���ֽ�
 * @param  ��
 * @retval ��
 */
void usart_send_byte(USART_TypeDef * pUSARTx, uint8_t ch)
{
    /* ����һ���ֽ����ݵ�USART */
    USART_SendData(pUSARTx, ch);

    /* �ȴ��������ݼĴ���Ϊ�� */
    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}

/**
 * @brief  ����8λ������
 * @param  ��
 * @retval ��
 */
void usart_send_array(USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num)
{
    uint8_t i;

    for (i = 0; i < num; i++) {
        /* ����һ���ֽ����ݵ�USART */
        usart_send_byte(pUSARTx, array[i]);

    }

    /* �ȴ�������� */
    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET);
}

/**
 * @brief  �����ַ���
 * @param  ��
 * @retval ��
 */
void usart_send_string(USART_TypeDef * pUSARTx, char *str)
{
    unsigned int k = 0;

    do {
        usart_send_byte(pUSARTx, *(str + k));
        k++;
    } while (*(str + k) != '\0');

    /* �ȴ�������� */
    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET)
    {}
}

/**
 * @brief  ����һ��16λ��
 * @param  ��
 * @retval ��
 */
void Usart_SendHalfWord(USART_TypeDef * pUSARTx, uint16_t ch)
{
    uint8_t temp_h, temp_l;

    /* ȡ���߰�λ */
    temp_h = (ch & 0XFF00) >> 8;
    /* ȡ���Ͱ�λ */
    temp_l = ch & 0XFF;

    /* ���͸߰�λ */
    USART_SendData(pUSARTx, temp_h);

    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);

    /* ���͵Ͱ�λ */
    USART_SendData(pUSARTx, temp_l);

    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}

/**
 * @brief  �ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
 * @param  ��
 * @retval ��
 */
int fputc(int ch, FILE *f)
{
    /* ����һ���ֽ����ݵ����� */
    USART_SendData(DEBUG_USARTx, (uint8_t) ch);

    /* �ȴ�������� */
    while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);

    return (ch);
}

/**
 * @brief  �ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
 * @param  ��
 * @retval ��
 */
int fgetc(FILE *f)
{
    /* �ȴ������������� */
    while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET);

    return (int)USART_ReceiveData(DEBUG_USARTx);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

