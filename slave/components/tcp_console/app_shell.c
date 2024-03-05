//#include <APP_SHELL_ASSERT.h>
#include "app_shell.h"
#include "string.h"
#include "stdio.h"

#define APP_SHELL_ASSERT(x)             while (0)
#define KEY_ESC (0x1BU)
#define KET_DEL (0x7FU)

static int32_t help_cmd(p_shell_context_t context, int32_t argc, char **argv); /*!< help command */

static int32_t exit_cmd(p_shell_context_t context, int32_t argc, char **argv); /*!< exit command */

static int32_t parse_line(const char *cmd, uint32_t len, char *argv[SHELL_MAX_ARGS]); /*!< parse line command */

static int32_t custom_str_cmp(const char *str1, const char *str2, int32_t count); /*!< compare string command */

static void process_cmd(p_shell_context_t context, const char *cmd); /*!< process a command */

static void get_history_cmd(p_shell_context_t context, uint8_t hist_pos); /*!< get commands history */

#if SHELL_AUTO_COMPLETE
static void auto_complete(p_shell_context_t context); /*!< auto complete command */
#endif



static char *custom_str_cpy(char *dest, const char *src, int32_t count); /*!< string copy */

/*******************************************************************************
 * Variables
 ******************************************************************************/
static const shell_command_context_t xhelp_cmd = {"help", "\r\n\"help\": Lists all the registered commands\r\n",
                                                     help_cmd, 0};

static const shell_command_context_t xexit_cmd = {"exit", "\r\n\"exit\": Exit program\r\n", exit_cmd, 0};

static shell_command_context_list_t m_registered_cmd;

static char m_param_buffer[SHELL_BUFFER_SIZE];

/*******************************************************************************
 * Code
 ******************************************************************************/
static bool m_loop_back = false;
void app_shell_init(p_shell_context_t context, 
                send_data_cb_t send_cb, 
                printf_data_t shell_printf, 
                char *prompt, 
                bool loop_back)
{
    APP_SHELL_ASSERT(send_cb != NULL);
    APP_SHELL_ASSERT(prompt != NULL);
    APP_SHELL_ASSERT(shell_printf != NULL);

    /* Memset for context */
    memset(context, 0, sizeof(shell_context_struct));
    context->send_data_func = send_cb;
    context->printf_data_func = shell_printf;
    context->prompt = prompt;

    m_loop_back = loop_back;
    app_shell_register_cmd(&xhelp_cmd);
    app_shell_register_cmd(&xexit_cmd);
}


static p_shell_context_t m_context;
void app_shell_set_context(p_shell_context_t context)
{
    m_context = context;
}

static volatile bool welcome_msg_printed = false;
int32_t app_shell_task(uint8_t ch)
{
    int32_t i;

    if (!m_context)
    {
        return -1;
    }
    if (welcome_msg_printed == false)
    {
        m_context->exit = false;
        char date[48];
        sprintf(date, "%s", __DATE__);
        m_context->printf_data_func("\r\nSHELL (build:");
        m_context->printf_data_func(date);
        m_context->printf_data_func("\r\n");

        // m_context->printf_data_func("Copyright (c) BYTECH JSC\r\n");
        m_context->printf_data_func(m_context->prompt);
        welcome_msg_printed = true;
    }

    // while (1)
    {
        if (m_context->exit)
        {
            return -1;
        }

        if (ch == APP_SHELL_INVALID_CHAR) //invalid input
            return 0;

        /* Special key */
        if (ch == KEY_ESC)
        {
            m_context->stat = kSHELL_Special;
            return 0;
        }
        else if (m_context->stat == kSHELL_Special)
        {
            /* Function key */
            if (ch == '[')
            {
                m_context->stat = kSHELL_Function;
                return 0;
            }
            m_context->stat = kSHELL_Normal;
        }
        else if (m_context->stat == kSHELL_Function)
        {
            m_context->stat = kSHELL_Normal;

            switch ((uint8_t)ch)
            {
            /* History operation here */
            case 'A': /* Up key */
                get_history_cmd(m_context, m_context->hist_current);
                if (m_context->hist_current < (m_context->hist_count - 1))
                {
                    m_context->hist_current++;
                }
                break;
            case 'B': /* Down key */
                get_history_cmd(m_context, m_context->hist_current);
                if (m_context->hist_current > 0)
                {
                    m_context->hist_current--;
                }
                break;
            case 'D': /* Left key */
                if (m_context->c_pos)
                {
                    m_context->printf_data_func("\b");
                    m_context->c_pos--;
                }
                break;
            case 'C': /* Right key */
                if (m_context->c_pos < m_context->l_pos)
                {
                    char tmp[4];
                    sprintf(tmp, "%c", m_context->line[m_context->c_pos]);
                    m_context->printf_data_func(tmp);
                    m_context->c_pos++;
                }
                break;
            default:
                break;
            }
            return 0;
        }
        /* Handle tab key */
        else if (ch == '\t')
        {
#if SHELL_AUTO_COMPLETE
            /* Move the cursor to the beginning of line */
            for (i = 0; i < m_context->c_pos; i++)
            {
                m_context->printf_data_func("\b");
            }
            /* Do auto complete */
            auto_complete(m_context);
            /* Move position to end */
            m_context->c_pos = m_context->l_pos = strlen(m_context->line);
#endif
            return 0;
        }
#if SHELL_SEARCH_IN_HIST
        /* Search command in history */
        else if ((ch == '`') && (m_context->l_pos == 0) && (m_context->line[0] == 0x00))
        {
        }
#endif
        /* Handle backspace key */
        else if ((ch == KET_DEL) || (ch == '\b'))
        {
            /* There must be at last one char */
            if (m_context->c_pos == 0)
            {
                return 0;
            }

            m_context->l_pos--;
            m_context->c_pos--;

            if (m_context->l_pos > m_context->c_pos)
            {
                memmove(&m_context->line[m_context->c_pos], &m_context->line[m_context->c_pos + 1],
                        m_context->l_pos - m_context->c_pos);
                m_context->line[m_context->l_pos] = 0;
                m_context->printf_data_func("\b");
                m_context->printf_data_func(&m_context->line[m_context->c_pos]);
                m_context->printf_data_func("  \b");
                /* Reset position */
                for (i = m_context->c_pos; i <= m_context->l_pos; i++)
                {
                    m_context->printf_data_func("\b");
                }
            }
            else /* Normal backspace operation */
            {
                m_context->printf_data_func("\b \b");
                m_context->line[m_context->l_pos] = 0;
            }
            return 0;
        }
        else
        {
        }

        /* Input too long */
        if (m_context->l_pos >= (SHELL_BUFFER_SIZE - 1))
        {
            m_context->l_pos = 0;
        }

        /* Handle end of line, break */
        if ((ch == '\r') || (ch == '\n'))
        {
            m_context->printf_data_func("\r\n");
            process_cmd(m_context, m_context->line);
            /* Reset all params */
            m_context->c_pos = m_context->l_pos = 0;
            m_context->hist_current = 0;
            m_context->printf_data_func(m_context->prompt);
            memset(m_context->line, 0, sizeof(m_context->line));
            return 0;
        }

        /* Normal character */
        if (m_context->c_pos < m_context->l_pos)
        {
            memmove(&m_context->line[m_context->c_pos + 1], &m_context->line[m_context->c_pos],
                    m_context->l_pos - m_context->c_pos);
            m_context->line[m_context->c_pos] = ch;
            m_context->printf_data_func(&m_context->line[m_context->c_pos]);
            /* Move the cursor to new position */
            for (i = m_context->c_pos; i < m_context->l_pos; i++)
            {
                m_context->printf_data_func("\b");
            }
        }
        else
        {
            m_context->line[m_context->l_pos] = ch;
            if (m_loop_back)
            {
                char tmp[4];
                sprintf(tmp, "%c", ch);
                m_context->printf_data_func(tmp);
            }
        }

        ch = 0;
        m_context->l_pos++;
        m_context->c_pos++;
    }

    return 0;
}

static int32_t help_cmd(p_shell_context_t context, int32_t argc, char **argv)
{
    uint8_t i = 0;

    for (i = 0; i < m_registered_cmd.number_of_cmd_in_list; i++)
    {
        context->printf_data_func(m_registered_cmd.cmd_list[i]->pcHelpString);
    }
    return 0;
}

static int32_t exit_cmd(p_shell_context_t context, int32_t argc, char **argv)
{
    /* Skip warning */
    context->printf_data_func("\r\nSHELL exited\r\n");
    context->exit = true;
    return 0;
}

static void process_cmd(p_shell_context_t context, const char *cmd)
{
    static const shell_command_context_t *tmp_cmd = NULL;
    static const char *tmp_cmd_str;
    int32_t argc;
    char *argv[SHELL_BUFFER_SIZE];
    uint8_t flag = 1;
    uint8_t tmp_cmd_len;
    uint8_t tmpLen;
    uint8_t i = 0;

    tmpLen = strlen(cmd);
    argc = parse_line(cmd, tmpLen, argv);

    if ((tmp_cmd == NULL) && (argc > 0))
    {
        for (i = 0; i < m_registered_cmd.number_of_cmd_in_list; i++)
        {
            tmp_cmd = m_registered_cmd.cmd_list[i];
            tmp_cmd_str = tmp_cmd->pcCommand;
            tmp_cmd_len = strlen(tmp_cmd_str);
            /* Compare with space or end of string */
            if ((cmd[tmp_cmd_len] == ' ') || (cmd[tmp_cmd_len] == 0x00))
            {
                if (custom_str_cmp(tmp_cmd_str, argv[0], tmp_cmd_len) == 0)
                {
                    if ((tmp_cmd->expected_number_of_parameters == 0) && (argc == 1))
                    {
                        flag = 0;
                    }
                    else if (tmp_cmd->expected_number_of_parameters > 0)
                    {
                        if ((argc - 1) == tmp_cmd->expected_number_of_parameters)
                        {
                            flag = 0;
                        }
                    }
                    else if (argc >= 0 && tmp_cmd->expected_number_of_parameters == -1)
                    {
                        flag = 0;
                    }
                    else
                    {
                        flag = 1;
                    }
                    break;
                }
            }
        }
    }

    if ((tmp_cmd != NULL) && (flag == 1U))
    {
        context->printf_data_func(
            "\r\nIncorrect command parameter(s).  Enter \"help\" to view a list of available commands.\r\n\r\n");
        tmp_cmd = NULL;
    }
    else if (tmp_cmd != NULL)
    {
        tmpLen = strlen(cmd);
        /* Compare with last command. Push back to history buffer if different */
        if (tmpLen != custom_str_cmp(cmd, context->hist_buf[0], strlen(cmd)))
        {
            for (i = SHELL_HIST_MAX - 1; i > 0; i--)
            {
                memset(context->hist_buf[i], '\0', SHELL_BUFFER_SIZE);
                tmpLen = strlen(context->hist_buf[i - 1]);
                custom_str_cpy(context->hist_buf[i], context->hist_buf[i - 1], tmpLen);
            }
            memset(context->hist_buf[0], '\0', SHELL_BUFFER_SIZE);
            tmpLen = strlen(cmd);
            custom_str_cpy(context->hist_buf[0], cmd, tmpLen);
            if (context->hist_count < SHELL_HIST_MAX)
            {
                context->hist_count++;
            }
        }
        tmp_cmd->pFuncCallBack(context, argc, argv);
        tmp_cmd = NULL;
    }
    else
    {
        // context->printf_data_func(
        //     "\r\nCommand not recognised.  Enter 'help' to view a list of available commands.\r\n\r\n");
        tmp_cmd = NULL;
    }
}

static void get_history_cmd(p_shell_context_t context, uint8_t hist_pos)
{
    uint8_t i;
    uint32_t tmp;

    if (context->hist_buf[0][0] == '\0')
    {
        context->hist_current = 0;
        return;
    }
    if (hist_pos > SHELL_HIST_MAX)
    {
        hist_pos = SHELL_HIST_MAX - 1;
    }
    tmp = strlen(context->line);
    /* Clear current if have */
    if (tmp > 0)
    {
        memset(context->line, '\0', tmp);
        for (i = 0; i < tmp; i++)
        {
            context->printf_data_func("\b \b");
        }
    }

    context->l_pos = strlen(context->hist_buf[hist_pos]);
    context->c_pos = context->l_pos;
    custom_str_cpy(context->line, context->hist_buf[hist_pos], context->l_pos);
    context->printf_data_func(context->hist_buf[hist_pos]);
}

#if SHELL_AUTO_COMPLETE
static void auto_complete(p_shell_context_t context)
{
    int32_t len;
    int32_t minLen;
    uint8_t i = 0;
    const shell_command_context_t *tmp_cmd = NULL;
    const char *namePtr;
    const char *cmdName;

    minLen = 0;
    namePtr = NULL;

    if (!strlen(context->line))
    {
        return;
    }
    context->printf_data_func("\r\n");
    /* Empty tab, list all commands */
    if (context->line[0] == '\0')
    {
        help_cmd(context, 0, NULL);
        return;
    }
    /* Do auto complete */
    for (i = 0; i < m_registered_cmd.number_of_cmd_in_list; i++)
    {
        tmp_cmd = m_registered_cmd.cmd_list[i];
        cmdName = tmp_cmd->pcCommand;
        if (custom_str_cmp(context->line, cmdName, strlen(context->line)) == 0)
        {
            if (minLen == 0)
            {
                namePtr = cmdName;
                minLen = strlen(namePtr);
                /* Show possible matches */
                context->printf_data_func("%s\r\n", cmdName);
                continue;
            }
            len = custom_str_cmp(namePtr, cmdName, strlen(namePtr));
            if (len < 0)
            {
                len = len * (-1);
            }
            if (len < minLen)
            {
                minLen = len;
            }
        }
    }
    /* Auto complete string */
    if (namePtr)
    {
        custom_str_cpy(context->line, namePtr, minLen);
    }
    context->printf_data_func("%s%s", context->prompt, context->line);

    return;
}

#endif /* SHELL_AUTO_COMPLETE */

static char *custom_str_cpy(char *dest, const char *src, int32_t count)
{
    char *ret = dest;
    int32_t i = 0;

    for (i = 0; i < count; i++)
    {
        dest[i] = src[i];
    }

    return ret;
}

static int32_t custom_str_cmp(const char *str1, const char *str2, int32_t count)
{
    while (count--)
    {
        if (*str1++ != *str2++)
        {
            return *(unsigned char *)(str1 - 1) - *(unsigned char *)(str2 - 1);
        }
    }
    return 0;
}

static int32_t parse_line(const char *cmd, uint32_t len, char *argv[SHELL_MAX_ARGS])
{
    uint32_t argc;
    char *p;
    uint32_t position;

    /* Init params */
    memset(m_param_buffer, '\0', len + 1);
    custom_str_cpy(m_param_buffer, cmd, len);

    p = m_param_buffer;
    position = 0;
    argc = 0;

    while (position < len)
    {
        /* Skip all blanks */
        while (((char)(*p) == ' ') && (position < len))
        {
            *p = '\0';
            p++;
            position++;
        }
        /* Process begin of a string */
        if (*p == '"')
        {
            p++;
            position++;
            argv[argc] = p;
            argc++;
            /* Skip this string */
            while ((*p != '"') && (position < len))
            {
                p++;
                position++;
            }
            /* Skip '"' */
            *p = '\0';
            p++;
            position++;
        }
        else /* Normal char */
        {
            argv[argc] = p;
            argc++;
            while (((char)*p != ' ') && ((char)*p != '\t') && (position < len))
            {
                p++;
                position++;
            }
        }
    }
    return argc;
}

int32_t app_shell_register_cmd(const shell_command_context_t *command_context)
{
    int32_t result = 0;

    /* If have room  in command list */
    if (m_registered_cmd.number_of_cmd_in_list < SHELL_MAX_CMD)
    {
        m_registered_cmd.cmd_list[m_registered_cmd.number_of_cmd_in_list++] = command_context;
    }
    else
    {
        result = -1;
    }
    return result;
}
