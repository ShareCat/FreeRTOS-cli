/**
  ******************************************************************************
  * @file:      command_line.c
  * @author:    Cat(���ƽ)
  * @version:   V1.0
  * @date:      2018-1-18
  * @brief:     ����������
  * @attention:
  ******************************************************************************
  */


#include "command_line.h"
#include "./../bsp/bsp_usart.h"
//#include "../app/app_led.h"


static uint8_t cli_help(void *para, uint8_t len);
static uint8_t cli_clear(void *para, uint8_t len);
static uint8_t cli_echo(void *para, uint8_t len);
static uint8_t cli_reboot(void *para, uint8_t len);


__packed typedef struct {
#define HANDLE_LEN 128

    uint8_t buff[HANDLE_LEN];
    uint8_t len;
} HANDLE_TYPE_S;



uint8_t cli_echo_flag = DISABLE; /* ����Ĭ�Ϲر� */



const char CLI_Cmd_Help[] =
    "\r\n"
    "[help]\r\n"
    " * show commands\r\n"
    "\r\n";

const char CLI_Cmd_Clear[] =
    "[cls]\r\n"
    " * clear the screen\r\n"
    "\r\n";

const char CLI_Cmd_Echo[] =
    "[echo]\r\n"
    " * echo 1: echo on\r\n"
    " * echo 0: echo off\r\n"
    "\r\n";

const char CLI_Cmd_Reboot[] =
    "[reboot]\r\n"
    " * reboot MCU\r\n"
    "\r\n";


/**
  * ����ṹ����������ִ�Сд
  */
const COMMAND_S CLI_Cmd[] = {
    /* ����               �������                ��ʼ������          ������ */
    {"help",            CLI_Cmd_Help,       NULL,           cli_help},
    {"cls",             CLI_Cmd_Clear,      NULL,           cli_clear},
    {"echo",            CLI_Cmd_Echo,       NULL,           cli_echo},
    {"reboot",          CLI_Cmd_Reboot,     NULL,           cli_reboot},

    /* ������ϵͳ������¿�ʼ���й�Ӧ�õ������� */
    //{"led",             CLI_Cmd_LED,        CLI_LED_Init,   CLI_LED},
};


/**
  * @brief          ��ӡÿ������İ�����Ϣ
  * @param  para:   ������ַ
  * @param  len:    ��������
  * @retval         ����True��ʾ��ȷ
  */
static uint8_t cli_help(void *para, uint8_t len)
{
    uint8_t i;

    for (i = 0; i < sizeof(CLI_Cmd) / sizeof(COMMAND_S); i++) {
        if (NULL != CLI_Cmd[i].pHelp) {
            PRINTF(CLI_Cmd[i].pHelp);
        }
    }

    return TRUE;
}


/**
  * @brief          ��������
  * @param  para:   ������ַ
  * @param  len:    ��������
  * @retval         ����True��ʾ��ȷ
  */
static uint8_t cli_clear(void *para, uint8_t len)
{
    TERMINAL_BACK_BLACK(); /* �����ն���ʾ����Ϊ��ɫ */
    TERMINAL_FONT_GREEN(); /* �����ն���ʾ����Ϊ��ɫ */

    /* This prints the clear screen and move cursor to top-left corner control
     * characters for VT100 terminals. This means it will not work on
     * non-VT100 compliant terminals, namely Windows' cmd.exe, but should
     * work on anything unix-y. */
    TERMINAL_RESET_CURSOR();
    TERMINAL_DISPLAY_CLEAR(); /* secureCRT�յ�����ַ��������� */

    return TRUE;
}


/**
  * @brief          ��ӡÿ������İ�����Ϣ
  * @param  para:   ������ַ
  * @param  len:    ��������
  * @retval         ����True��ʾ��ȷ
  */
static uint8_t cli_echo(void *para, uint8_t len)
{
    uint8_t *pTemp;
    pTemp = (uint8_t *)para;

    if ((0 < len) && (NULL != pTemp)) {
        pTemp++; /* ����һ���ո� */

        if ('1' == *pTemp) {
            /* �򿪻��� */
            cli_echo_flag = ENABLE;
            PRINTF("echo on\r\n");
        } else if ('0' == *pTemp) {
            /* �رջ��� */
            cli_echo_flag = DISABLE;
            PRINTF("echo off\r\n");
        } else {
            /* �����д� */
            return FALSE;
        }
    }

    /* ��ȷ���� */
    return TRUE;
}


#if 0
/**
  * @brief  ��ȡMCU��Ϣ
  * @param  ������ַ�Ͳ�������
  * @retval ����True��ʾ��ȷ
  */
static uint8_t cli_mcu_info(void *para, uint8_t len)
{
    uint16_t flash_size;
    uint32_t MAC_buff[3];

    /* ��ȡFLASH��С */
    flash_size = *(__IO uint16_t *)(0x1FFF7A22);
    PRINTF("Flash size: %d KB\r\n", flash_size);

    /* ��ȡID */
    MAC_buff[0] = *(__IO uint32_t*)(0x1FFF7A10);
    MAC_buff[1] = *(__IO uint32_t*)(0x1FFF7A14);
    MAC_buff[2] = *(__IO uint32_t*)(0x1FFF7A18);
    PRINTF("UID(hex): %02X-%02X-%02X\r\n", MAC_buff[0], MAC_buff[1], MAC_buff[2]);

    return TRUE;
}
#endif


/**
  * @brief          ����MCU
  * @param  para:   ������ַ
  * @param  len:    ��������
  * @retval         ����True��ʾ��ȷ
  */
static uint8_t cli_reboot(void *para, uint8_t len)
{
    //extern void Delay(__IO uint32_t nCount);
    PRINTF("\r\n[END]: System Rebooting");
    PRINTF(".");
    //Delay(0xFFFFF);
    PRINTF(".");
    //Delay(0xFFFFF);

    SYSTEM_REBOOT();

    return TRUE;
}


#if CLI_HISTORY

__packed typedef struct {
    char cmd[CLI_HISTORY_MAX][HANDLE_LEN];
    uint8_t count;
    uint8_t latest;
    uint8_t show;
} HISTORY_S;

static HISTORY_S history;


/**
  * @brief          ���һ����ʷ��¼
  * @param  buff:   ��ʷ��¼
  * @retval         null
  */
static void cli_history_add(char* buff)
{
    uint16_t len;
    uint8_t index = history.latest;

    if (NULL == buff) return;

    len = strlen((const char *)buff);

    if (len >= HANDLE_LEN) return;  /* �������� */

    /* ��λ���һ����ʷ���� */
    if (0 != index) {
        index--;
    } else {
        index = CLI_HISTORY_MAX - 1;
    }

    if (0 != memcmp(history.cmd[index], buff, len)) {
        /* �����һ����ʷ���һ�����ű��� */
        memset((void *)history.cmd[history.latest], 0x00, HANDLE_LEN);
        memcpy((void *)history.cmd[history.latest], (const void *)buff, len);

        if (history.count < CLI_HISTORY_MAX) {
            history.count++;
        }

        history.latest++;

        if (history.latest >= CLI_HISTORY_MAX) {
            history.latest = 0;
        }
    }

    history.show = 0;
}


/**
  * @brief              �鿴��ʷ��¼
  * @param  mode:       TRUE�鿴��һ����FALSE�鿴��һ��
  * @param  p_history:  ָ���ѯ������ʷ��¼
  * @retval             TRUE��ʾû����ʷ��¼��FALSE��ʾ��ѯ�ɹ�
  */
static uint8_t cli_history_show(uint8_t mode, char** p_history)
{
    uint8_t err = TRUE;
    uint8_t num;
    uint8_t index;

    if (0 == history.count) return err;

    if (TRUE == mode) {
        /* ��һ����ʷ���� */
        if (history.show < history.count) {
            history.show++;
        }
    } else {
        /* ��һ����ʷ���� */
        if (1 < history.show) {
            history.show--;
        }
    }

    num = history.show;
    index = history.latest;

    while (num) {
        if (0 != index) {
            index--;
        } else {
            index = CLI_HISTORY_MAX - 1;
        }

        num--;
    }

    err = FALSE;
    *p_history = history.cmd[index];
    //PRINTF("history: %s \r\n", history.cmd[index]);

    return err;
}

#endif



//#include "./../FreeRTOS/include/task.h"
QueueHandle_t   cli_queue = NULL;
#define CLI_QUEUE_LEN       128    /* ���еĳ��ȣ����ɰ������ٸ���Ϣ */
#define CLI_QUEUE_SIZE      1      /* ������ÿ����Ϣ��С���ֽڣ� */


/**
  * @brief  �����г�ʼ��
  * @param  ���ڲ�����
  * @retval null
  */
void cli_init(uint32_t baud)
{
    uint8_t i;

    /* ����cli_queue */
    cli_queue = xQueueCreate((UBaseType_t ) CLI_QUEUE_LEN,  /* ��Ϣ���еĳ��� */
                            (UBaseType_t ) CLI_QUEUE_SIZE); /* ��Ϣ�Ĵ�С */
    //if(NULL != cli_queue)
    //    printf("creat cli_queue ok! \r\n");

#if CLI_HISTORY
    memset((uint8_t *)&history, 0, sizeof(history));
#endif
    USART_INIT(baud);

    /* ��ÿ��������г�ʼ�� */
    for (i = 0; i < sizeof(CLI_Cmd) / sizeof(COMMAND_S); i++) {
        /* �������ʼ�������ǿ� */
        if (NULL != CLI_Cmd[i].pInit) {
            if (FALSE == CLI_Cmd[i].pInit()) {
                /* ִ�г�ʼ���������ش���Ҫ��ʾ */
                PRINTF("\r\n-> FUN[%d] INIT WRONG\r\n", i);
            }
        }
    }

    PRINTF(" \r\n");
    TERMINAL_BACK_BLACK(); /* �����ն���ʾ����Ϊ��ɫ */
    TERMINAL_FONT_GREEN(); /* �����ն���ʾ����Ϊ��ɫ */
    TERMINAL_DISPLAY_CLEAR();
    TERMINAL_RESET_CURSOR();

    PRINTF("------------------------------\r\n\r\n");
    TERMINAL_HIGH_LIGHT();
    PRINTF("    CLI version: V0.6         \r\n\r\n");
    PRINTF("    coder: Cat                \r\n\r\n");
    PRINTF("    Email: 843553493@qq.com   \r\n\r\n");
    TERMINAL_UN_HIGH_LIGHT();
    PRINTF("------------------------------\r\n\r\n");
}


static HANDLE_TYPE_S Handle = {.len = 0};
static void cli_char_process(HANDLE_TYPE_S *h)
{
    /* KEY_BACKSPACE����ɾ�������һ���ַ� */
    if (KEY_BACKSPACE == h->buff[h->len]) {
        /* ���л�����ַ���ɾ�����һ�� */
        if (0 < h->len) {
            /* ʵ����secrueCRT��Ҳɾ����������һ���ַ� */
            TERMINAL_MOVE_LEFT(1);
            TERMINAL_CLEAR_END();
            /* ����һ���ַ� */
            h->len -= 1;
        }

    } else if (KEY_HORIZONTAL_TAB == h->buff[h->len]) {
        /* ����ˮƽ�Ʊ��ʵ�������в�ȫ */

    } else {
        /* �������ַ�������ɾ������Ҳ����ˮƽ�Ʊ�� */
        h->len++;
    }
}
static char cli_get_line(void)
{
    while(1) {
        if (Handle.len >= HANDLE_LEN) break; /* �������� */

        /* �����н��յ������ݣ����Ƶ�Handle.buff����� */
        if (TRUE == xQueueReceive(cli_queue, &Handle.buff[Handle.len], 0)) { //portMAX_DELAY
            cli_char_process(&Handle);
        } else {
            break;
        }
    }

    return 0;
}

static void cli_line_process(uint8_t position)
{
    uint8_t i = position;
    /* ��ȫ�����Ƶ�Handle.buff�� */
#if CLI_HISTORY
    uint8_t key = 0;
    uint8_t err = 0xff;
    char *p_hist_cmd = 0;

    if (Handle.len > 2) {
        if (0 != strstr((const char *)Handle.buff, KEY_UP)) {
            //PRINTF("KEY_UP \r\n");
            key = 1;
            TERMINAL_MOVE_LEFT(Handle.len);
            TERMINAL_CLEAR_END();
            err = cli_history_show(TRUE, &p_hist_cmd);
        } else if (0 != strstr((const char *)Handle.buff, KEY_DOWN)) {
            //PRINTF("KEY_DOWN \r\n");
            key = 2;
            TERMINAL_MOVE_LEFT(Handle.len);
            TERMINAL_CLEAR_END();
            err = cli_history_show(FALSE, &p_hist_cmd);
        } else if (0 != strstr((const char *)Handle.buff, KEY_RIGHT)) {
            //PRINTF("KEY_RIGHT \r\n");
            key = 3;
        } else if (0 != strstr((const char *)Handle.buff, KEY_LEFT)) {
            //PRINTF("KEY_LEFT \r\n");
            key = 4;
        }

        if (0 != key) {
            if (FALSE == err) {
                memset(&Handle, 0x00, sizeof(Handle));
                memcpy(Handle.buff, p_hist_cmd, strlen(p_hist_cmd));
                Handle.len = strlen(p_hist_cmd);
                Handle.buff[Handle.len] = '\0';
                PRINTF("%s", Handle.buff);  /* ��ʾ��ѯ������ */
            } else if ((TRUE == err) || (0 != key)) {
                /* ���ϵ磬û���κ���ʷ�����˲�ѯΪ�� */
                TERMINAL_MOVE_LEFT(Handle.len);
                TERMINAL_CLEAR_END();
                memset(&Handle, 0x00, sizeof(Handle));
            }
        }
    }

    if ((0 == key) && (i < Handle.len)) {
#endif

        /* ���յ����ַ����ͳ�ȥ���ն˻��� */
        for (; i < Handle.len; i++) {
            USART_SendData(DEBUG_USARTx, Handle.buff[i]);
        }

#if CLI_HISTORY
    }

#endif
}

static void cli_cmd_process(void)
{
    uint8_t i;
    uint8_t ParaLen;
    uint8_t *ParaAddr;
    uint8_t cmd_match = FALSE;

    if ((1 == Handle.len) && (KEY_ENTER == Handle.buff[Handle.len - 1])) {
        /* ����Ӧ������"\r"������̨���س����Ƿ��͵�"\r"��MCU */
        Handle.len = 0;
    } else if (1 < Handle.len) { /* ��������������������Ϊ�������"\r"����1�ֽ��� */
        /* ���������"\r"��β */
        if (KEY_ENTER == Handle.buff[Handle.len - 1]) {
            Handle.buff[Handle.len - 1] = '\0';

            /* ѭ����Ѱ��ƥ������� */
            for (i = 0; i < sizeof(CLI_Cmd) / sizeof(COMMAND_S); i++) {
                if (0 == strncmp((const char *)Handle.buff,
                                 (void *)CLI_Cmd[i].pCmd,
                                 strlen(CLI_Cmd[i].pCmd))) {
                    cmd_match = TRUE;
                    ParaLen = Handle.len - strlen(CLI_Cmd[i].pCmd);   /* ��������ĳ��� */
                    ParaAddr = &Handle.buff[strlen(CLI_Cmd[i].pCmd)]; /* ��������ĵ�ַ */

                    if (NULL != CLI_Cmd[i].pFun) {
                        /* ִ�������Ӧ�ĺ��� */
                        if (CLI_Cmd[i].pFun(ParaAddr, ParaLen)) {
                            /* ����ִ����ȷ */
                            PRINTF("\r\n-> OK\r\n");
#if CLI_HISTORY
                            cli_history_add((char *)Handle.buff);
#endif

                            /* �����˻��ԣ��ʹ�ӡ�յ������� */
                            if (ENABLE == cli_echo_flag) {
                                Handle.buff[Handle.len] = '\0';
                                PRINTF("[echo]: %s\r\n", (const char*)Handle.buff);
                            }
                        } else {
                            /* ����ִ�г��� */
                            PRINTF("\r\n-> PARA. ERR\r\n");
                            /* ����������ʾ������ʹ�ð��� */
                            PRINTF(CLI_Cmd[i].pHelp);
                        }
                    } else {
                        /* �ǿպ�������ʾ���� */
                        PRINTF("\r\n-> FUNC. ERR\r\n");
                    }
                }
            }

            if (FALSE == cmd_match) {
                /* û��ƥ�䵽��Ч�����ʾ������� */
                PRINTF("\r\n-> CMD ERR, try: help\r\n\r\n");
            }

            Handle.len = 0;

        }

    }
}

/**
  * @brief              ������յ�������
  * @param              null
  * @retval             null
  */
static void cli_parser(void)
{
    uint8_t i = Handle.len;

    /* �����ڽ��յ������ݱ������� */
    cli_get_line();

    /* ����������һ���ַ� */
    cli_line_process(i);

    /* �������� */
    cli_cmd_process();


    if (Handle.len >= HANDLE_LEN) {
        /* ���ˣ���ռ��� */
        Handle.len = 0;
    }
}


/**
  * @brief  �������������ݣ�һ��50ms����һ�ξ͹���
  * @param  null
  * @retval null
  */
void cli_task(void)
{
    while(1) {
        cli_parser();
        vTaskDelay(50);
    }
}


