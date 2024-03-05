#ifndef TCP_CONSOLE_H
#define TCP_CONSOLE_H

#include "stdint.h"
#include "stdbool.h"

/**
 * @brief           Start TCP console
 */
void tcp_console_start(void);


/**
 * @brief           Check if tcp console is started
 */
bool tcp_console_started(void);

/**
 * @brief           Send data to tcp console port link printf style
 */
void tcp_printf(const char *format, ...);

/**
 * @brief           Send a buffer to tcp console port
 */
uint32_t tcp_send_buffer(const void *data, uint32_t len);

#endif /* TCP_CONSOLE_H */
