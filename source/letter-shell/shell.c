/**
 * @file shell.c
 * @author Letter (NevermindZZT@gmail.cn)
 * @brief letter shell
 * @version 2.0.0
 * @date 2018-12-29
 * 
 * @Copyright (c) 2018 Letter
 * 
 */

#include "shell.h"
#include "usb.h"
#include "util.h"
#include "telemetry.h"
#include "string.h"

#if SHELL_AUTO_PRASE == 1
#include "shell_ext.h"
#endif

SHELL_TypeDef shellData __attribute__((section(".ccm")));
TaskHandle_t shellTaskHandle;


static unsigned short shellDisplay(SHELL_TypeDef *shell, const char *string);
static void shellDisplayItem(SHELL_TypeDef *shell, unsigned short index);
static void shellHelp(SHELL_TypeDef *shell);
/**
 * @brief shell
 * 
 * @param shell 
 */

static void shellDelay(unsigned long n)
{
    delay(n);
}


#if SHELL_USING_CMD_EXPORT != 1
/**
 * @brief shellĬ�������
 * 
 * @note ��ʹ�������ʽ���������ʱ�򣬴˱�Ż���Ч
 * @note �������ʱ����ʹ��SHELL_CMD_ITEM�꣬��SHELL_CMD_ITEM(help, shellHelp, command help)
 * @note �ɲ�ʹ��Ĭ���������ʼ�����֮�󣬿ɵ���shellSetCommandList�ӿ����������
 */

const SHELL_CommandTypeDef shellDefaultCommandList[] =
{
    SHELL_CMD_ITEM(help, shellHelp, command help),
	SHELL_CMD_ITEM(delayms, shellDelay, command delayms),
	SHELL_CMD_ITEM(IMU, telemetryEnable, command get IMU data),
};
#endif
#if SHELL_USING_OS == 1
/**
 * @brief shell ����(����ϵͳʹ��)
 * 
 * @param param shell����
 * 
 * @note ʹ�ò���ϵͳʱ�������shell read��������������ʽ��
 */

void shellTaskCode(void *pvParameters)
{
    SHELL_TypeDef *shell = &shellData;
	for( ;; )
	{
	    if (shell->read == NULL)
	    {
	        shellDisplay(shell, "error: shell.read() must be defined\r\n");
	    }
	    else if(usbAvailable())
	    {
	        shellHandler(shell, shell->read());
	    }
		yield(50);
	}
}
#endif


/**
 * @brief shell��ʼ��
 * 
 * @param shell shell����
 */

void shellInit(void)
{
	SHELL_TypeDef *shell = &shellData;
	memset((void *)&shellData, 0, sizeof(shellData));
	shell->write = usbTxOnebyte;
	shell->read = usbRx;
    shellDisplay(shell, "\r\n\r\n");
    shellDisplay(shell, "+=========================================================+\r\n");
    shellDisplay(shell, "|                (C) COPYRIGHT 2018 Letter                |\r\n");
    shellDisplay(shell, "|                   Letter shell v"SHELL_VERSION"                   |\r\n");
    shellDisplay(shell, "|               Build: "__DATE__" "__TIME__"               |\r\n");
    shellDisplay(shell, "+=========================================================+\r\n");
    shellDisplay(shell, SHELL_COMMAND);
    shell->length = 0;
    shell->cursor = 0;
    shell->historyCount = 0;
    shell->historyFlag = 0;
    shell->historyOffset = 0;
    shell->status = CONTROL_FREE;

#if SHELL_USING_CMD_EXPORT == 1
    extern const unsigned int shellCommand$$Base;
    extern const unsigned int shellCommand$$Limit;

    shell->commandBase = (SHELL_CommandTypeDef *)(&shellCommand$$Base);
    shell->commandNumber = ((unsigned int)(&shellCommand$$Limit)
                            - (unsigned int)(&shellCommand$$Base))
                            / sizeof(SHELL_CommandTypeDef);
#else
    shell->commandBase = (SHELL_CommandTypeDef *)shellDefaultCommandList;
    shell->commandNumber = sizeof(shellDefaultCommandList) / sizeof(SHELL_CommandTypeDef);
#endif
	xTaskCreate((TaskFunction_t )shellTaskCode,     	
                (const char*    )"shell_task",   	
                (uint16_t       )SHELL_STACK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )SHELL_PRIORITY,	
                (TaskHandle_t*  )&shellTaskHandle);  
}


/**
 * @brief shell���������
 * 
 * @param shell shell����
 * @param base ������ַ
 * @param size ��������
 * 
 * @note �˽ӿڲ�����shellInit֮ǰ����
 * @note �����ô˽ӿڣ���ʹ��Ĭ��������������γɵ������(ȡ��������巽ʽ)
 */

void shellSetCommandList(SHELL_TypeDef *shell, SHELL_CommandTypeDef *base, unsigned short size)
{
    shell->commandBase = base;
    shell->commandNumber = size;
}

/**
 * @brief shell��ʾ�ַ���
 * 
 * @param shell shell����
 * @param string �ַ���
 * @return unsigned short �ַ�������
 */

static unsigned short shellDisplay(SHELL_TypeDef *shell, const char *string)
{
    unsigned short count = 0;
    if (shell->write == NULL)
    {
        return 0;
    }
    while(*string)
    {
        shell->write(*string++);
        count ++;
    }
    return count;
}


/**
 * @brief shell��ʾ�ַ�
 * 
 * @param data �ַ�
 */

static void shellDisplayByte(SHELL_TypeDef *shell, char data)
{
    if (shell->write == NULL)
    {
        return;
    }
    shell->write(data);
}


/**
 * @brief shell�ַ�������
 * 
 * @param dest Ŀ���ַ���
 * @param src Դ�ַ���
 * @return unsigned short �ַ�������
 */

static unsigned short shellStringCopy(char *dest, char* src)
{
    unsigned short count = 0;
    while (*(src + count))
    {
        *(dest + count) = *(src + count);
        count++;
    }
    *(dest + count) = 0;
    return count;
}


/**
 * @brief shell�ַ����Ƚ�
 * 
 * @param dest Ŀ���ַ���
 * @param src Դ�ַ���
 * @return unsigned short ƥ�䳤��
 */

static unsigned short shellStringCompare(char* dest, char *src)
{
    unsigned short match = 0;
    unsigned short i = 0;

    while (*(dest +i) && *(src + i))
    {
        if (*(dest + i) != *(src +i))
        {
            break;
        }
        match ++;
        i++;
    }
    return match;
}


/**
 * @brief shellɾ��
 * 
 * @param shell shell����
 * @param length ɾ���ĳ���
 */

static void shellDelete(SHELL_TypeDef *shell, unsigned short length)
{
    while (length--)
    {
        shellDisplay(shell, "\b \b");
    }
}


/**
 * @brief shell�������
 * 
 * @param shell shell����
 */

static void shellClearLine(SHELL_TypeDef *shell)
{
    for (short i = shell->length - shell->cursor; i > 0; i--)
    {
        shellDisplayByte(shell, ' ');
    }
    shellDelete(shell, shell->length);
}


/**
 * @brief shell��ʷ��¼���
 * 
 * @param shell shell����
 */

static void shellHistoryAdd(SHELL_TypeDef *shell)
{
    if (strcmp(shell->history[shell->historyFlag - 1], shell->buffer) == 0)
    {
        return;
    }
    if (shellStringCopy(shell->history[shell->historyFlag], shell->buffer) != 0)
    {
        shell->historyFlag++;
    }
    if (++shell->historyCount > SHELL_HISTORY_MAX_NUMBER)
    {
        shell->historyCount = SHELL_HISTORY_MAX_NUMBER;
    }
    if (shell->historyFlag >= SHELL_HISTORY_MAX_NUMBER)
    {
        shell->historyFlag = 0;
    }
    shell->historyOffset = 0;
}


/**
 * @brief shell��ʷ��¼����
 * 
 * @param shell shell����
 * @param dir ���ҷ���
 */

static void shellHistory(SHELL_TypeDef *shell, unsigned char dir)
{
    if (dir == 0)
    {
        if (shell->historyOffset--
            <= -((shell->historyCount > shell->historyFlag)
            ? shell->historyCount : shell->historyFlag))
        {
            shell->historyOffset = -((shell->historyCount > shell->historyFlag)
                                   ? shell->historyCount : shell->historyFlag);
        }
    }
    else if (dir == 1)
    {
        if (++shell->historyOffset > 0)
        {
            shell->historyOffset = 0;
            return;
        }
    }
    else
    {
        return;
    }
    shellClearLine(shell);
    if (shell->historyOffset == 0)
    {
        shell->cursor = shell->length = 0;
    }
    else
    {
        if ((shell->length = shellStringCopy(shell->buffer,
                shell->history[(shell->historyFlag + SHELL_HISTORY_MAX_NUMBER
                + shell->historyOffset) % SHELL_HISTORY_MAX_NUMBER])) == 0)
        {
            return;
        }
        shell->cursor = shell->length;
        shellDisplay(shell, shell->buffer);
    }
}


/**
 * @brief shell�س����봦��
 * 
 * @param shell shell����
 */

static void shellEnter(SHELL_TypeDef *shell)
{
    unsigned char paramCount = 0;
    unsigned char quotes = 0;
    unsigned char record = 1;
    SHELL_CommandTypeDef *base;
    unsigned char runFlag = 0;

    if (shell->length == 0)
    {
        shellDisplay(shell, SHELL_COMMAND);
        return;
    }
    
    *(shell->buffer + shell->length++) = 0;

    shellHistoryAdd(shell);

    for (unsigned short i = 0; i < shell->length; i++)
    {
        if ((quotes != 0 ||
            (*(shell->buffer + i) != ' ' &&
            *(shell->buffer + i) != '\t' &&
            *(shell->buffer + i) != ',')) &&
            *(shell->buffer + i) != 0)
        {
            if (*(shell->buffer + i) == '\"')
            {
                quotes = quotes ? 0 : 1;
            #if SHELL_AUTO_PRASE == 0
                *(shell->buffer + i) = 0;
                continue;
            #endif
            }
            if (record == 1)
            {
                shell->param[paramCount++] = shell->buffer + i;
                record = 0;
            }
            if (*(shell->buffer + i) == '\\' &&
                *(shell->buffer + i) != 0)
            {
                i++;
            }
        }
        else
        {
            *(shell->buffer + i) = 0;
            record = 1;
        }
    }
    shell->length = 0;
    shell->cursor = 0;
    if (paramCount == 0)
    {
        shellDisplay(shell, SHELL_COMMAND);
        return;
    }

    shellDisplay(shell, "\r\n");
    base = shell->commandBase;
    if (strcmp((const char *)shell->param[0], "help") == 0)
    {
        shellHelp(shell);
        shellDisplay(shell, SHELL_COMMAND);
        return;
    }
    for (unsigned char i = 0; i < shell->commandNumber; i++)
    {
        if (strcmp((const char *)shell->param[0], (base + i)->name) == 0)
        {
        #if SHELL_AUTO_PRASE == 0
            runFlag = 1;
            (base + i)->function(paramCount, shell->param);
        #else
            runFlag = shellExtRun((base + i)->function, paramCount, shell->param)
                      + 1;
        #endif
        }
    }
    if (runFlag == 0)
    {
        shellDisplay(shell, "Command not found\r\n");
    }
    shellDisplay(shell, SHELL_COMMAND);
}


/**
 * @brief shell�˸����봦��
 * 
 * @param shell shell����
 */

static void shellBackspace(SHELL_TypeDef *shell)
{
    if (shell->length == 0)
    {
        return;
    }
    if (shell->cursor == shell->length)
    {
        shell->length--;
        shell->cursor--;
        shell->buffer[shell->length] = 0;
        shellDelete(shell, 1);
    }
    else if (shell->cursor > 0)
    {
        for (short i = 0; i < shell->length - shell->cursor; i++)
        {
            shell->buffer[shell->cursor + i - 1] = shell->buffer[shell->cursor + i];
        }
        shell->length--;
        shell->cursor--;
        shell->buffer[shell->length] = 0;
        shellDisplayByte(shell, '\b');
        for (short i = shell->cursor; i < shell->length; i++)
        {
            shellDisplayByte(shell, shell->buffer[i]);
        }
        shellDisplayByte(shell, ' ');
        for (short i = shell->length - shell->cursor + 1; i > 0; i--)
        {
            shellDisplayByte(shell, '\b');
        }
    }
}


/**
 * @brief shell Tab�����봦��
 * 
 * @param shell shell����
 */

static void shellTab(SHELL_TypeDef *shell)
{
    unsigned short maxMatch = SHELL_COMMAND_MAX_LENGTH;
    unsigned short lastMatchIndex = 0;
    unsigned short matchNum = 0;
    unsigned short length;
    SHELL_CommandTypeDef *base = shell->commandBase;

    if (shell->length != 0)
    {
        shell->buffer[shell->length] = 0;
        shellDisplay(shell, "\r\n");
        for (short i = 0; i < shell->commandNumber; i++)
        {
            if (shellStringCompare(shell->buffer, 
                (char *)(base + i)->name)
                == shell->length)
            {
                shellDisplayItem(shell, i);
                if (matchNum != 0)
                {
                    length = shellStringCompare((char *)(base + lastMatchIndex)->name,
                                                (char *)(base +i)->name);
                    maxMatch = (maxMatch > length) ? length : maxMatch;
                }
                lastMatchIndex = i;
                matchNum ++;
            }
        }
        
        shellDisplay(shell, SHELL_COMMAND);
        if (matchNum != 0)
        {
            shell->length = shellStringCopy(shell->buffer,
                                            (char *)(base + lastMatchIndex)->name);
        }
        if (matchNum > 1)
        {
            shell->length = maxMatch;
        }
        shell->buffer[shell->length] = 0;
        shell->cursor = shell->length;
        shellDisplay(shell, shell->buffer);
    }
    else
    {
        shellHelp(shell);
        shellDisplay(shell, SHELL_COMMAND);
    }
}


/**
 * @brief shell������������
 * 
 * @param shell shell����
 * @param data ���������
 */

static void shellNormal(SHELL_TypeDef *shell, char data)
{
    switch ((unsigned char)(shell->status))
    {
    case CONTROL_STEP_TWO:
        switch (data)
        {
        case 0x41:                                              /** �����ϼ� */
            shellHistory(shell, 0);
            break;  
        
        case 0x42:                                              /** �����¼� */
            shellHistory(shell, 1);
            break;

        case 0x43:                                              /** �����Ҽ� */
            if (shell->cursor < shell->length)
            {
                shellDisplayByte(shell, shell->buffer[shell->cursor]);
                shell->cursor++;
            }
            break;

        case 0x44:                                              /** ������� */
            if (shell->cursor > 0)
            {
                shellDisplayByte(shell, '\b');
                shell->cursor--;
            }
            break;

        default:
            break;
        }
        shell->status = CONTROL_FREE;
        break;

    case CONTROL_STEP_ONE:
        if (data == 0x5B)
        {
            shell->status = CONTROL_STEP_TWO;
        }
        else
        {
            shell->status = CONTROL_FREE;
        }
        break;

    case CONTROL_FREE:
        if (shell->length < SHELL_COMMAND_MAX_LENGTH - 1)
        {
            if (shell->length == shell->cursor)
            {
                shell->buffer[shell->length++] = data;
                shell->cursor++;
                shellDisplayByte(shell, data);
            }
            else
            {
                for (short i = shell->length - shell->cursor; i > 0; i--)
                {
                    shell->buffer[shell->cursor + i] = shell->buffer[shell->cursor + i - 1];
                }
                shell->buffer[shell->cursor++] = data;
                shell->buffer[++shell->length] = 0;
                for (short i = shell->cursor - 1; i < shell->length; i++)
                {
                    shellDisplayByte(shell, shell->buffer[i]);
                }
                for (short i = shell->length - shell->cursor; i > 0; i--)
                {
                    shellDisplayByte(shell, '\b');
                }
            }
        }
        else
        {
            shellDisplay(shell, "\r\nWarnig: Command is too long\r\n");
            shellDisplay(shell, SHELL_COMMAND);
            shellDisplay(shell, shell->buffer);
            shell->cursor = shell->length;
        }
        break;

    default:
        break;
    }
}


/**
 * @brief shell����
 * 
 * @param shell shell����
 * @param data ��������
 */

void shellHandler(SHELL_TypeDef *shell, char data)
{
    switch (data)
    {
    case '\r':  
    case '\n':
        shellEnter(shell);
        break;

    case '\b':
        shellBackspace(shell);
        break;

    case '\t':
        shellTab(shell);
        break;

    case 0x1B:
        shell->status = CONTROL_STEP_ONE;
        break;

    default:
        shellNormal(shell, data);
        break;
    }
}






/**
 * @brief shell��ʾһ��������Ϣ
 * 
 * @param shell shell����
 * @param index Ҫ��ʾ����������
 */

static void shellDisplayItem(SHELL_TypeDef *shell, unsigned short index)
{
    unsigned short spaceLength;
    SHELL_CommandTypeDef *base = shell->commandBase;
    
    spaceLength = 22 - shellDisplay(shell, (base + index)->name);
    spaceLength = (spaceLength > 0) ? spaceLength : 4;
    do {
        shellDisplay(shell, " ");
    } while (--spaceLength);
    shellDisplay(shell, "--");
    shellDisplay(shell, (base + index)->desc);
    shellDisplay(shell, "\r\n");

}


/**
 * @brief shell����
 * 
 * @param shell shell����
 */

static void shellHelp(SHELL_TypeDef *shell)
{
    shellDisplay(shell, "\r\nCOMMAND LIST:\r\n\r\n");

    for(unsigned short i = 0; i < shell->commandNumber; i++)
    {
        shellDisplayItem(shell, i);
    }
}
SHELL_EXPORT_CMD(help, shellHelp, command help);
