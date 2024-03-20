/******************************************************************************
 * @file:    app_cli.c
 * @brief:
 * @version: V0.0.0
 * @date:    2019/11/12
 * @author:
 * @email:
 *
 * THE SOURCE CODE AND ITS RELATED DOCUMENTATION IS PROVIDED "AS IS". VINSMART
 * JSC MAKES NO OTHER WARRANTY OF ANY KIND, WHETHER EXPRESS, IMPLIED OR,
 * STATUTORY AND DISCLAIMS ANY AND ALL IMPLIED WARRANTIES OF MERCHANTABILITY,
 * SATISFACTORY QUALITY, NON INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * THE SOURCE CODE AND DOCUMENTATION MAY INCLUDE ERRORS. VINSMART JSC
 * RESERVES THE RIGHT TO INCORPORATE MODIFICATIONS TO THE SOURCE CODE IN LATER
 * REVISIONS OF IT, AND TO MAKE IMPROVEMENTS OR CHANGES IN THE DOCUMENTATION OR
 * THE PRODUCTS OR TECHNOLOGIES DESCRIBED THEREIN AT ANY TIME.
 *
 * VINSMART JSC SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGE OR LIABILITY ARISING FROM YOUR USE OF THE SOURCE CODE OR
 * ANY DOCUMENTATION, INCLUDING BUT NOT LIMITED TO, LOST REVENUES, DATA OR
 * PROFITS, DAMAGES OF ANY SPECIAL, INCIDENTAL OR CONSEQUENTIAL NATURE, PUNITIVE
 * DAMAGES, LOSS OF PROPERTY OR LOSS OF PROFITS ARISING OUT OF OR IN CONNECTION
 * WITH THIS AGREEMENT, OR BEING UNUSABLE, EVEN IF ADVISED OF THE POSSIBILITY OR
 * PROBABILITY OF SUCH DAMAGES AND WHETHER A CLAIM FOR SUCH DAMAGE IS BASED UPON
 * WARRANTY, CONTRACT, TORT, NEGLIGENCE OR OTHERWISE.
 *
 * (C)Copyright VINSMART JSC 2019 All rights reserved
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "app_cli.h"
#include "app_shell.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
// #include "main.h"
#include "app_sntp.h"
#include "app_flash.h"
#include "driver/gpio.h"
#include "app_ota.h"
#include "version_ctrl.h"
#include "app_audio.h"

static const char *TAG = "cli";

static int32_t cli_get_memory(p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_reset(p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_ota_update(p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_factory_reset(p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_get_imei(p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_console_disable(p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_show_fw_version(p_shell_context_t context, int32_t argc, char **argv);
static int32_t restart_blue(p_shell_context_t context, int32_t argc, char **argv);

static const shell_command_context_t cli_command_table[] =
{
    {"mem", "\tmem: Get free memory size\r\n", cli_get_memory, 0},
    {"reset", "\treset: Reset\r\n", cli_reset, 1},
    {"ota", "\tota: Update firmware, ex ota http://asdasd/bin\r\n", cli_ota_update, 1},
    {"imei", "\timei: Get imei\r\n", cli_get_imei, 0},
    {"version", "\tversion: Get firmware version\r\n", cli_show_fw_version, 0},
    {"factory", "\tfactory: Factory reset\r\n", cli_factory_reset, 0},
    {"consoleDisable", "\tconsoleDisable: disable tcp console\r\n", cli_console_disable, 0},
    {"blue", "\trestart bt service\r\n", restart_blue, 0},
};

static shell_context_struct m_user_context;
static app_cli_cb_t *m_cb;

void app_cli_poll(uint8_t ch)
{
    app_shell_task(ch);
}

void app_cli_start(app_cli_cb_t *callback)
{
    m_cb = callback;
    app_shell_set_context(&m_user_context);
    app_shell_init(&m_user_context,
                   m_cb->puts,
                   m_cb->printf,
                   ">",
                   false);

    /* Register CLI commands */
    for (int i = 0; i < sizeof(cli_command_table) / sizeof(shell_command_context_t); i++)
    {
        app_shell_register_cmd(&cli_command_table[i]);
    }

    /* Run CLI task */
    app_shell_task(APP_SHELL_INVALID_CHAR);
}

/* Reset System */
static int32_t cli_get_memory(p_shell_context_t context, int32_t argc, char **argv)
{
    char tmp[256];
    sprintf(tmp, "Free memory size %u\r\n", xPortGetFreeHeapSize());
    // char tmp[512+128];
    // vTaskGetRunTimeStats(tmp);
    // ESP_LOGI(TAG, tmp);
    // ESP_LOGI(TAG, "\r\n");
    m_cb->printf(tmp);
    return 0;
}

static int32_t cli_reset(p_shell_context_t context, int32_t argc, char **argv)
{
    esp_restart();
    return 0;
}

// extern void app_ota_start(char *url);
static int32_t cli_ota_update(p_shell_context_t context, int32_t argc, char **argv)
{
    char *link = strstr(argv[1], "http://");
    if (!link)
    {
        link = strstr(argv[1], "https://");
    }
    if (link)
    {
        /* Không nhận lệnh liên tiếp, hoặc khi đang chạy task OTA rồi */
        if (app_ota_is_running())
        {
            return 0;
        }


        static char m_ota_http_url[256];
        sprintf(m_ota_http_url, "%s", link);
        static app_ota_info_t ota_url;
        ota_url.url = m_ota_http_url;
        ota_url.type = APP_OTA_DEVICE_ESP32;
        xTaskCreate(app_ota_download_task, "ota_tsk", 8192, (void *)&ota_url, 5, NULL);
    }

    return 0;
}


// static int32_t cli_get_config(p_shell_context_t context, int32_t argc, char **argv)
// {
//     if (strstr(argv[1], "dump"))
//     {
//         //		internal_flash_cfg_t *cfg = internal_flash_get_config();
//         //		(void)cfg;
//         //		ESP_LOGI(TAG, "\t\tConfig addr %s:%d\r\n\tPing nterval %ums", cfg->host_addr, cfg->port, cfg->ping_cycle);
//     }

//     return 0;
// }

static int32_t cli_get_imei(p_shell_context_t context, int32_t argc, char **argv)
{
    m_cb->printf("IMEI = ");
    m_cb->printf(app_flash_get_imei());
    m_cb->printf("\r\n");
    return 0;
}

static int32_t cli_console_disable(p_shell_context_t context, int32_t argc, char **argv)
{
    app_flash_tcp_console_disable();
    m_cb->printf("Terminate session\r\n");
    vTaskDelay(2000);
    m_cb->terminate();
    return 0;
}

static int32_t restart_blue(p_shell_context_t context, int32_t argc, char **argv)
{
    if (true)//app_flash_is_btc_enable())
    {
        app_audio_restart_bluetooth();
    }
    m_cb->printf("Terminate session\r\n");
    vTaskDelay(2000);
    m_cb->terminate();
    return 0;
}

static int32_t cli_factory_reset(p_shell_context_t context, int32_t argc, char **argv)
{
    m_cb->printf("Do factory reset\r\n");
    app_flash_do_factory_reset();
    return 0;
}



static int32_t cli_show_fw_version(p_shell_context_t context, int32_t argc, char **argv)
{
    m_cb->printf("Firmware version : ");
    m_cb->printf(__FIRMWARE_VERSION__);
    // m_cb->printf("\r\nHardware version : ");
    // m_cb->printf(HARDWARE_VERSION);
    // m_cb->printf("\r\n");
    char build[48];
    sprintf(build, "Build %s %s\r\n", __DATE__, __TIME__);
    m_cb->printf(build);
    return 0;
}
