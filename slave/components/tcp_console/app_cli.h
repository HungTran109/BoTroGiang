#ifndef APP_CLI_H
#define APP_CLI_H

#include <stdint.h>

typedef void (*app_cli_puts_cb_t)(uint8_t *buffer, uint32_t size);
typedef int (*app_cli_print_cb_t)(const char *msg);
typedef void (*app_cli_terminate_console_cb_t)(void);

typedef struct
{
    app_cli_puts_cb_t puts;
    app_cli_print_cb_t printf;
    app_cli_terminate_console_cb_t terminate;
} app_cli_cb_t;

/*!
 * @brief   Start cli task 
 */
void app_cli_start(app_cli_cb_t *callback);

/*!
 * @brief       Polling cli service
 * @param[in]   ch New character
 */
void app_cli_poll(uint8_t ch);


#endif /* APP_CLI_H */
