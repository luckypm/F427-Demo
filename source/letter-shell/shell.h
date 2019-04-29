/**
 * @file shell.h
 * @author Letter (NevermindZZT@gmail.cn)
 * @brief letter shell
 * @version 2.0.0
 * @date 2018-12-29
 * 
 * @Copyright (c) 2018 Letter
 * 
 */

#ifndef     __SHELL_H__
#define     __SHELL_H__
#include "aq.h"

#define SHELL_STACK_SIZE	    200
#define SHELL_PRIORITY	    	9


#define     SHELL_VERSION               "2.0.0"                 /**< �汾�� */

#define     SHELL_USING_OS              1                       /**< �����ڲ���ϵͳ������ */
#define     SHELL_USING_CMD_EXPORT      0                       /**< �Ƿ�ʹ���������ʽ */
#define     SHELL_AUTO_PRASE            1                       /**< �Ƿ�ʹ��shell�����Զ����� */
#define     SHELL_COMMAND_MAX_LENGTH    50                      /**< shell������󳤶� */
#define     SHELL_PARAMETER_MAX_NUMBER  5                       /**< shell�������������� */
#define     SHELL_HISTORY_MAX_NUMBER    5                       /**< ��ʷ�����¼���� */

#define     SHELL_COMMAND               "\r\nletter>>"          /**< shell��ʾ�� */

/**
 * @brief shell�����
 * 
 * @attention �������ʽĿǰ��֧�� keil �����ı�������ʹ��ʱ��Ҫ�� keil ��
 *            Option for Target �� Linker ѡ��� Misc controls ����� --keep shellCommand*
 */
#if SHELL_USING_CMD_EXPORT == 1
#define     SHELL_EXPORT_CMD(cmd, func, desc)                               \
            const SHELL_CommandTypeDef                                      \
            shellCommand##cmd __attribute__((section("shellCommand"))) =    \
            {                                                               \
                #cmd,                                                       \
                (void (*)())func,                                           \
                #desc                                                       \
            }
#else
#define     SHELL_EXPORT_CMD(cmd, func, desc)
#endif

#if SHELL_USING_CMD_EXPORT == 0
/**
 * @brief shell������Ŀ
 * 
 * @note ����shell����ͨ�������ķ�ʽ����
 */
#define     SHELL_CMD_ITEM(cmd, func, desc)                                 \
            {                                                               \
                #cmd,                                                       \
                (void (*)())func,                                           \
                #desc                                                       \
            }
#endif
            
typedef char (*shellRead)(void);                                /**< shell��ȡ���ݺ���ԭ�� */
typedef void (*shellWrite)(const char);                         /**< shellд���ݺ���ԭ�� */
typedef void (*shellFunction)();                                /**< shellָ��ִ�к���ԭ�� */


/**
 * @brief shell����ָ��״̬
 * 
 */
typedef enum
{
    CONTROL_FREE = 0,
    CONTROL_STEP_ONE,
    CONTROL_STEP_TWO,
}CONTROL_Status;


/**
 * @brief shell �����
 * 
 */
typedef struct
{
    const char *name;                                           /**< shell�������� */
    shellFunction function;                                     /**< shell����� */
    const char *desc;                                           /**< shell�������� */
}SHELL_CommandTypeDef;


/**
 * @brief shell������
 * 
 */
typedef struct
{
    char buffer[SHELL_COMMAND_MAX_LENGTH];                      /**< shell����� */
    unsigned short length;                                      /**< shell����� */
    char *param[SHELL_PARAMETER_MAX_NUMBER];                    /**< shell���� */
    unsigned short cursor;                                      /**< shell���λ�� */
    char history[SHELL_HISTORY_MAX_NUMBER][SHELL_COMMAND_MAX_LENGTH];  /**< ��ʷ��¼ */
    unsigned short historyCount;                                /**< ��ʷ��¼���� */
    short historyFlag;                                          /**< ��ǰ��¼λ�� */
    short historyOffset;                                        /**< ��ʷ��¼ƫ�� */
    SHELL_CommandTypeDef *commandBase;                          /**< ������ַ */
    unsigned short commandNumber;                               /**< �������� */
    CONTROL_Status status;                                      /**< ���Ƽ�״̬ */
    shellRead read;                                             /**< shell���ַ� */
    shellWrite write;                                           /**< shellд�ַ� */
}SHELL_TypeDef;

void shellInit(void);
void shellSetCommandList(SHELL_TypeDef *shell, SHELL_CommandTypeDef *base, unsigned short size);
void shellHandler(SHELL_TypeDef *shell, char data);

#if SHELL_USING_OS == 1
void shellTask(void *param);
#endif

#endif

