// Copyright 2015-2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "ec2x.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "netif/ppp/pppapi.h"
#include "netif/ppp/pppos.h"
#include "lwip/dns.h"
#include "esp_netif.h"
// #include "tcpip_adapter.h"
#include "esp_modem.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "utilities.h"
#include "main.h"
#include "app_io.h"
#include "app_flash.h"
#include "app_debug.h"

#define MODEM_RESULT_CODE_POWERDOWN "POWERED DOWN"

/**
 * @brief Macro defined for error checking
 *
 */
static const char *TAG = "EC2x";
#if 0
#define DCE_CHECK(a, str, goto_tag, ...)                                          \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            DEBUG_ERROR("%s: " str, __FUNCTION__, ##__VA_ARGS__); \
            DEBUG_RAW("\r\n"); \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)
#endif

#define DCE_CHECK(a, str, goto_tag, ...)                                          \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)


/**
 * @brief EC2x Modem
 *
 */
typedef struct
{
    void *priv_resource; /*!< Private resource */
    modem_dce_t parent;  /*!< DCE parent class */
} ec2x_modem_dce_t;

ec2x_modem_dce_t *ec2x_dce = NULL;


/**
 * @brief Handle response from AT+CSQ
 */
static esp_err_t ec2x_handle_csq(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;
    ec2x_modem_dce_t *ec2x_dce = __containerof(dce, ec2x_modem_dce_t, parent);
    if (strstr(line, MODEM_RESULT_CODE_SUCCESS))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    }
    else if (strstr(line, MODEM_RESULT_CODE_ERROR))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    }
    else if (!strncmp(line, "+CSQ", strlen("+CSQ")))
    {
        /* store value of rssi and ber */
        uint32_t **csq = ec2x_dce->priv_resource;
        /* +CSQ: <rssi>,<ber> */
        sscanf(line, "%*s%d,%d", csq[0], csq[1]);
        err = ESP_OK;
    }
    return err;
}

/**
 * @brief Handle response from AT+CBC
 */
static esp_err_t ec2x_handle_cbc(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;
    ec2x_modem_dce_t *ec2x_dce = __containerof(dce, ec2x_modem_dce_t, parent);
    if (strstr(line, MODEM_RESULT_CODE_SUCCESS))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    }
    else if (strstr(line, MODEM_RESULT_CODE_ERROR))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    }
    else if (!strncmp(line, "+CBC", strlen("+CBC")))
    {
        /* store value of bcs, bcl, voltage */
        uint32_t **cbc = ec2x_dce->priv_resource;
        /* +CBC: <bcs>,<bcl>,<voltage> */
        sscanf(line, "%*s%d,%d,%d", cbc[0], cbc[1], cbc[2]);
        err = ESP_OK;
    }
    return err;
}

/**
 * @brief Handle response from +++
 */
static esp_err_t ec2x_handle_exit_data_mode(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;
    if (strstr(line, MODEM_RESULT_CODE_SUCCESS))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    }
    else if (strstr(line, MODEM_RESULT_CODE_NO_CARRIER))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    }
    else if (strstr(line, MODEM_RESULT_CODE_ERROR))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    }
    return err;
}

/**
 * @brief Handle response from ATD*99#
 */
static esp_err_t ec2x_handle_atd_ppp(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;
    if (strstr(line, MODEM_RESULT_CODE_CONNECT))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    }
    else if (strstr(line, MODEM_RESULT_CODE_ERROR))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    }
    return err;
}

// /**
//  * @brief Handle response from AT+CGMM
//  */
// static esp_err_t ec2x_handle_cgmm(modem_dce_t *dce, const char *line)
// {
//     esp_err_t err = ESP_FAIL;
//     if (strstr(line, MODEM_RESULT_CODE_SUCCESS)) {
//         err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
//     } else if (strstr(line, MODEM_RESULT_CODE_ERROR)) {
//         err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
//     } else {
//         int len = snprintf(dce->name, MODEM_MAX_NAME_LENGTH, "%s", line);
//         if (len > 2) {
//             /* Strip "\r\n" */
//             strip_cr_lf_tail(dce->name, len);
//             err = ESP_OK;
//         }
//     }
//     return err;
// }

/**
 * @brief Handle response from AT+CGMR
 * EC20EQAR01A01E2G
 * OK
 */
static esp_err_t ec2x_handle_cgmr(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;
    if (strstr(line, MODEM_RESULT_CODE_SUCCESS))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    }
    else if (strstr(line, MODEM_RESULT_CODE_ERROR))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    }
    else
    {
        int len = snprintf(dce->name, MODEM_MAX_NAME_LENGTH, "%s", line);
        if (len > 2)
        {
            /* Strip "\r\n" */
            strip_cr_lf_tail(dce->name, len);
            err = ESP_OK;
        }
    }
    return err;
}

/**
 * @brief Handle response from AT+CGSN
 */
static esp_err_t ec2x_handle_cgsn(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;
    if (strstr(line, MODEM_RESULT_CODE_SUCCESS))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    }
    else if (strstr(line, MODEM_RESULT_CODE_ERROR))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    }
    else
    {
        int len = snprintf(dce->imei, MODEM_IMEI_LENGTH + 1, "%s", line);
        if (len > 2)
        {
            /* Strip "\r\n" */
            strip_cr_lf_tail(dce->imei, len);
            err = ESP_OK;
        }
    }
    return err;
}

/**
 * @brief Handle response from AT+CIMI
 * AT+CIMI
 * 460023210226023            //Query IMSI number of SIM which is attached to ME
 * OK
 */
static esp_err_t ec2x_handle_cimi(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;
    if (strstr(line, MODEM_RESULT_CODE_SUCCESS))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    }
    else if (strstr(line, MODEM_RESULT_CODE_ERROR))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    }
    else
    {
        int len = snprintf(dce->imsi, MODEM_IMSI_LENGTH + 1, "%s", line);
        if (len > 2)
        {
            /* Strip "\r\n" */
            strip_cr_lf_tail(dce->imsi, len);
            err = ESP_OK;
        }
    }
    return err;
}

/**
 * @brief Handle response from AT+QCCID
 * +QCCID: 89860025128306012474
 * OK
 */
static esp_err_t ec2x_handle_qccid(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;

    if (strstr(line, MODEM_RESULT_CODE_SUCCESS))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    }
    else if (strstr(line, MODEM_RESULT_CODE_ERROR))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    }
    else
    {
        //        int len = snprintf(dce->sim_imei, MODEM_IMSI_LENGTH + 1, "%s", line);
        //        if (len > 2) {
        //            /* Strip "\r\n" */
        //            strip_cr_lf_tail(dce->sim_imei, len);
        //            err = ESP_OK;
        //        }

        //+QCCID: 89860025128306012474
        char *qccid = strstr(line, "+QCCID:");
        if (qccid != NULL)
        {
            // ESP_LOGI(TAG, "%s", qccid);
            int len = snprintf(dce->sim_imei, MODEM_IMSI_LENGTH + 1, "%s", &qccid[8]);
            if (len > 2)
            {
                /* Strip "\r\n" */
                strip_cr_lf_tail(dce->sim_imei, len);
                err = ESP_OK;
            }
            // ESP_LOGI(TAG, "SIM IMEI: %s", dce->sim_imei);
        }
    }
    return err;
}

/**
 * @brief Handle response from AT+QNINFO
 */
static esp_err_t ec2x_handle_network_band(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;

    //+QNWINFO: “FDD LTE”,46011,“LTE BAND 3”,1825
    char *nwInfoMsg = strstr(line, "+QNWINFO:");
    if (nwInfoMsg != NULL)
    {
        ESP_LOGI(TAG, "%s", nwInfoMsg);

        uint8_t commaIndex[6] = {0};
        uint8_t index = 0;

        for (uint8_t i = 0; i < strlen(nwInfoMsg); i++)
        {
            if (nwInfoMsg[i] == ',')
                commaIndex[index++] = i;
        }
        if (index >= 3)
        {
            // Copy fields
            memset(dce->access_tech, 0, sizeof(dce->access_tech));

            //“GSM” “GPRS” “EDGE”/ “WCDMA” “HSDPA” “HSUPA” “HSPA+” “TDSCDMA”/ “TDD LTE” “FDD LTE”
            char accTech[MODEM_ACCESS_TECH_LENGTH] = {0};
            memcpy(accTech, &nwInfoMsg[11], commaIndex[0] - 12);
            snprintf(dce->access_tech, MODEM_ACCESS_TECH_LENGTH, "%s", accTech);

            char operatorCodeName[15] = {0};
            memset(dce->oper, 0, sizeof(dce->oper));
            memcpy(operatorCodeName, &nwInfoMsg[commaIndex[0] + 1], commaIndex[1] - commaIndex[0] - 1);
            uint32_t operatorCode = GetNumberFromString(1, operatorCodeName);
            switch (operatorCode)
            {
            case 45201:
                sprintf(dce->oper, "%s", "MOBIFONE");
                break;
            case 45202:
                sprintf(dce->oper, "%s", "VINAPHONE");
                break;
            case 45204:
                sprintf(dce->oper, "%s", "VIETTEL");
                break;
            case 45205:
                sprintf(dce->oper, "%s", "VNMOBILE");
                break;
            case 45207:
                sprintf(dce->oper, "%s", "BEELINE");
                break;
            case 45208:
                sprintf(dce->oper, "%s", "EVN");
                break;
            default:
                sprintf(dce->oper, "%d", operatorCode);
                break;
            }

            // Network band
            memset(dce->band, 0, sizeof(dce->band));

            char bandName[25] = {0};
            memcpy(bandName, (&nwInfoMsg[commaIndex[1] + 1])+1, commaIndex[2] - commaIndex[1] - 3);
            snprintf(dce->band, MODEM_NETWORK_BAND_LENGTH, "%s", bandName);

            // Network channel
            uint8_t j = 0;
            for (uint8_t i = commaIndex[2] + 1; i < strlen(nwInfoMsg); i++)
            {
                if (nwInfoMsg[i] == '\r' || nwInfoMsg[i] == '\n')
                    break;
                dce->channel[j++] = nwInfoMsg[i];
            }

            ESP_LOGI(TAG, "Access Techology: %s", dce->access_tech);
            ESP_LOGI(TAG, "Operator code: %s, name: %s", operatorCodeName, dce->oper);
            ESP_LOGI(TAG, "Network band: %s", dce->band);
            ESP_LOGI(TAG, "Network channel: %s", dce->channel);

            dce->isGetNetworkInfo = 1;
        }
        else
        {
            if (strstr(nwInfoMsg, "No Service"))
            {
                sprintf(dce->oper, "%s", "NO SERVICE");
            }
        }
    }

    if (strstr(line, MODEM_RESULT_CODE_SUCCESS))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    }
    else if (strstr(line, MODEM_RESULT_CODE_ERROR))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    }
    else
    {
        err = ESP_OK;
    }

    return err;
}

// /**
//  * @brief Handle response from AT+COPS?
//  */
// static esp_err_t ec2x_handle_cops(modem_dce_t *dce, const char *line)
// {
//     esp_err_t err = ESP_FAIL;
//     if (strstr(line, MODEM_RESULT_CODE_SUCCESS)) {
//         err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
//     } else if (strstr(line, MODEM_RESULT_CODE_ERROR)) {
//         err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
//     } else if (!strncmp(line, "+COPS", strlen("+COPS"))) {
//         /* there might be some random spaces in operator's name, we can not use sscanf to parse the result */
//         /* strtok will break the string, we need to create a copy */
//         size_t len = strlen(line);
//         char *line_copy = malloc(len + 1);
//         strcpy(line_copy, line);
//         /* +COPS: <mode>[, <format>[, <oper>]] */
//         char *str_ptr = NULL;
//         char *p[3];
//         uint8_t i = 0;
//         /* strtok will broke string by replacing delimiter with '\0' */
//         p[i] = strtok_r(line_copy, ",", &str_ptr);
//         while (p[i]) {
//             p[++i] = strtok_r(NULL, ",", &str_ptr);
//         }
//         if (i >= 3) {
//             int len = snprintf(dce->oper, MODEM_MAX_OPERATOR_LENGTH, "%s", p[2]);
//             if (len > 2) {
//                 /* Strip "\r\n" */
//                 strip_cr_lf_tail(dce->oper, len);
//                 err = ESP_OK;
//             }
//         }
//         free(line_copy);
//     }
//     return err;
// }

/**
 * @brief Handle response from AT+QPOWD=1
 */
static esp_err_t ec2x_handle_power_down(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;
    if (strstr(line, MODEM_RESULT_CODE_SUCCESS))
    {
        err = ESP_OK;
    }
    else if (strstr(line, MODEM_RESULT_CODE_POWERDOWN))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    }
    return err;
}

// /**
//  * @brief Handle response from any AT command that return result is "OK"
//  */
// static esp_err_t ec2x_handle_return_OK(modem_dce_t *dce, const char *line)
// {
//     esp_err_t err = ESP_FAIL;
//     if (strstr(line, MODEM_RESULT_CODE_SUCCESS)) {
//         err = ESP_OK;
//     }
//     return err;
// }

esp_err_t ec2x_modem_dce_sync(ec2x_modem_dce_t *ec2x_dce)
{
    modem_dte_t *dte = ec2x_dce->parent.dte;
    ec2x_dce->parent.handle_line = esp_modem_dce_handle_response_default;

    if (dte->send_cmd(dte, "ATV1\r", MODEM_COMMAND_TIMEOUT_DEFAULT) == ESP_FAIL)
    {
        DEBUG_WARN("send cmd failed\r\n");
        return ESP_FAIL;
    }
    if (ec2x_dce->parent.state != MODEM_STATE_SUCCESS)
    {
        DEBUG_ERROR("sync failed\r\n");
        return ESP_FAIL;
    }
    DEBUG_VERBOSE("sync ok\r\n");
    return ESP_OK;
}

esp_err_t ec2x_modem_dce_echo(ec2x_modem_dce_t *ec2x_dce, bool on)
{
    modem_dte_t *dte = ec2x_dce->parent.dte;
    ec2x_dce->parent.handle_line = esp_modem_dce_handle_response_default;

    if (on)
    {
        if (dte->send_cmd(dte, "ATE1\r", MODEM_COMMAND_TIMEOUT_DEFAULT) != ESP_OK)
        {
            DEBUG_ERROR("send cmd failed\r\n");
            return ESP_FAIL;
        }
        if (ec2x_dce->parent.state != MODEM_STATE_SUCCESS)
        {
            DEBUG_ERROR("enable echo failed\r\n");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "enable echo ok");
    }
    else
    {
        if (dte->send_cmd(dte, "ATE0\r", MODEM_COMMAND_TIMEOUT_DEFAULT) != ESP_OK)
        {
            DEBUG_ERROR("send cmd failed\r\n");
            return ESP_FAIL;
        }
        if (ec2x_dce->parent.state != MODEM_STATE_SUCCESS)
        {
            DEBUG_ERROR("disable echo failed\r\n");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "disable echo ok");
    }
    return ESP_OK;
}

/**
 * @brief Get signal quality
 *
 * @param dce Modem DCE object
 * @param rssi received signal strength indication
 * @param ber bit error ratio
 * @return esp_err_t
 *      - ESP_OK on success
 *      - ESP_FAIL on error
 */
static esp_err_t ec2x_get_signal_quality(modem_dce_t *dce, uint32_t *rssi, uint32_t *ber)
{
    modem_dte_t *dte = dce->dte;
    ec2x_modem_dce_t *ec2x_dce = __containerof(dce, ec2x_modem_dce_t, parent);
    uint32_t *resource[2] = {rssi, ber};
    ec2x_dce->priv_resource = resource;
    dce->handle_line = ec2x_handle_csq;
    DCE_CHECK(dte->send_cmd(dte, "AT+CSQ\r", MODEM_COMMAND_TIMEOUT_DEFAULT) == ESP_OK, "send cmd failed", err);
    DCE_CHECK(dce->state == MODEM_STATE_SUCCESS, "inquire signal quality failed", err);
    ESP_LOGI(TAG, "inquire signal quality ok");
    return ESP_OK;
err:
    return ESP_FAIL;
}

/**
 * @brief Get battery status
 *
 * @param dce Modem DCE object
 * @param bcs Battery charge status
 * @param bcl Battery connection level
 * @param voltage Battery voltage
 * @return esp_err_t
 *      - ESP_OK on success
 *      - ESP_FAIL on error
 */
static esp_err_t ec2x_get_battery_status(modem_dce_t *dce, uint32_t *bcs, uint32_t *bcl, uint32_t *voltage)
{
    modem_dte_t *dte = dce->dte;
    ec2x_modem_dce_t *ec2x_dce = __containerof(dce, ec2x_modem_dce_t, parent);
    uint32_t *resource[3] = {bcs, bcl, voltage};
    ec2x_dce->priv_resource = resource;
    dce->handle_line = ec2x_handle_cbc;
    DCE_CHECK(dte->send_cmd(dte, "AT+CBC\r", MODEM_COMMAND_TIMEOUT_DEFAULT) == ESP_OK, "send cmd failed", err);
    DCE_CHECK(dce->state == MODEM_STATE_SUCCESS, "Inquire battery status failed", err);
    DEBUG_VERBOSE("Inquire battery status ok");
    return ESP_OK;
err:
    return ESP_FAIL;
}

/**
 * @brief Set Working Mode
 *
 * @param dce Modem DCE object
 * @param mode woking mode
 * @return esp_err_t
 *      - ESP_OK on success
 *      - ESP_FAIL on error
 */
static esp_err_t ec2x_set_working_mode(modem_dce_t *dce, modem_mode_t mode)
{
    modem_dte_t *dte = dce->dte;
    switch (mode)
    {
    case MODEM_COMMAND_MODE:
        dce->handle_line = ec2x_handle_exit_data_mode;
        DCE_CHECK(dte->send_cmd(dte, "+++", MODEM_COMMAND_TIMEOUT_MODE_CHANGE) == ESP_OK, "send cmd failed", err);
        DCE_CHECK(dce->state == MODEM_STATE_SUCCESS, "enter command mode failed", err);
        ESP_LOGI(TAG, "enter command mode ok");
        dce->mode = MODEM_COMMAND_MODE;
        break;
    case MODEM_PPP_MODE:
        dce->handle_line = ec2x_handle_atd_ppp;
        DCE_CHECK(dte->send_cmd(dte, "ATD*99***1#\r", MODEM_COMMAND_TIMEOUT_MODE_CHANGE) == ESP_OK, "send cmd failed", err);
        DCE_CHECK(dce->state == MODEM_STATE_SUCCESS, "enter ppp mode failed", err);
        ESP_LOGI(TAG, "enter ppp mode ok");
        dce->mode = MODEM_PPP_MODE;
        break;
    default:
        ESP_LOGW(TAG, "unsupported working mode: %d", mode);
        goto err;
        break;
    }
    return ESP_OK;
err:
    return ESP_FAIL;
}

/**
 * @brief Power down
 *
 * @param ec2x_dce ec2x object
 * @return esp_err_t
 *      - ESP_OK on success
 *      - ESP_FAIL on error
 */
static esp_err_t ec2x_power_down(modem_dce_t *dce)
{
    modem_dte_t *dte = dce->dte;
    dce->handle_line = ec2x_handle_power_down;
    DCE_CHECK(dte->send_cmd(dte, "AT+QPOWD=1\r", MODEM_COMMAND_TIMEOUT_POWEROFF) == ESP_OK, "send cmd failed", err);
    DCE_CHECK(dce->state == MODEM_STATE_SUCCESS, "power down failed", err);
    ESP_LOGI(TAG, "power down ok");
    return ESP_OK;
err:
    return ESP_FAIL;
}

// /**
//  * @brief Get DCE module name
//  *
//  * @param ec2x_dce ec2x object
//  * @return esp_err_t
//  *      - ESP_OK on success
//  *      - ESP_FAIL on error
//  */
// static esp_err_t ec2x_get_module_name(ec2x_modem_dce_t *ec2x_dce)
// {
//     modem_dte_t *dte = ec2x_dce->parent.dte;
//     ec2x_dce->parent.handle_line = ec2x_handle_cgmm;
//     if (dte->send_cmd(dte, "AT+CGMM\r", MODEM_COMMAND_TIMEOUT_DEFAULT) != ESP_OK) {
// 		DEBUG_ERROR("send cmd failed");
// 		return ESP_FAIL;
//     }
//     if (ec2x_dce->parent.state != MODEM_STATE_SUCCESS) {
// 		DEBUG_ERROR("get module name failed");
// 		return ESP_FAIL;
//     }
//     ESP_LOGI(TAG, "get module name ok");
//     return ESP_OK;
// }

/**
 * @brief Request TA Revision Identification of Software Release
 *
 * @param ec2x_dce ec2x object
 * @return esp_err_t
 *      - ESP_OK on success
 *      - ESP_FAIL on error
 */
static esp_err_t ec2x_get_module_identify(ec2x_modem_dce_t *ec2x_dce)
{
    modem_dte_t *dte = ec2x_dce->parent.dte;
    ec2x_dce->parent.handle_line = ec2x_handle_cgmr;
    if (dte->send_cmd(dte, "AT+CGMR\r", MODEM_COMMAND_TIMEOUT_DEFAULT) != ESP_OK)
    {
        DEBUG_ERROR("send cmd failed\r\n");
        return ESP_FAIL;
    }
    if (ec2x_dce->parent.state != MODEM_STATE_SUCCESS)
    {
        DEBUG_ERROR("get module identify failed");
        return ESP_FAIL;
    }
    DEBUG_VERBOSE("get module identify OK");
    return ESP_OK;
}

/**
 * @brief Get DCE module IMEI number
 *
 * @param ec2x_dce ec2x object
 * @return esp_err_t
 *      - ESP_OK on success
 *      - ESP_FAIL on error
 */
static esp_err_t ec2x_get_imei_number(ec2x_modem_dce_t *ec2x_dce)
{
    modem_dte_t *dte = ec2x_dce->parent.dte;
    ec2x_dce->parent.handle_line = ec2x_handle_cgsn;
    DCE_CHECK(dte->send_cmd(dte, "AT+CGSN\r", MODEM_COMMAND_TIMEOUT_DEFAULT) == ESP_OK, "send cmd failed", err);
    DCE_CHECK(ec2x_dce->parent.state == MODEM_STATE_SUCCESS, "get imei number failed", err);
    DEBUG_VERBOSE("get GSM IMEI OK");
    return ESP_OK;
err:
    return ESP_FAIL;
}

// /**
//  * @brief Get DCE module IMEI number (SIM IMEI)
//  *
//  * @param ec2x_dce ec2x object
//  * @return esp_err_t
//  *      - ESP_OK on success
//  *      - ESP_FAIL on error
//  */
// static esp_err_t ec2x_get_imei(ec2x_modem_dce_t *ec2x_dce)
// {
//     modem_dte_t *dte = ec2x_dce->parent.dte;
//     ec2x_dce->parent.handle_line = ec2x_handle_qccid;
//     DCE_CHECK(dte->send_cmd(dte, "AT+QCCID\r", MODEM_COMMAND_TIMEOUT_DEFAULT) == ESP_OK, "send cmd failed", err);
//     DCE_CHECK(ec2x_dce->parent.state == MODEM_STATE_SUCCESS, "get sim imei failed", err);
//     ESP_LOGI(TAG, "get SIM IMEI OK");
//     return ESP_OK;
// err:
//     return ESP_FAIL;
// }

#if CONFIG_EC200_UNLOCK_BAND
static esp_err_t ec2x_handle_unlock_band_step(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;

    if (strstr(line, MODEM_RESULT_CODE_SUCCESS))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    }
    else
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    }
    return err;
}
#endif

static esp_err_t ec2x_do_unlock_band(ec2x_modem_dce_t *ec2x_dce)
{
#if CONFIG_EC200_UNLOCK_BAND
    modem_dte_t *dte = ec2x_dce->parent.dte;
    ec2x_dce->parent.handle_line = ec2x_handle_unlock_band_step;
    DCE_CHECK(dte->send_cmd(dte, "AT+QCFG=\"nwscanseq\",3,1\r", 5000) == ESP_OK, "send cmd failed", err);
    DCE_CHECK(ec2x_dce->parent.state == MODEM_STATE_SUCCESS, "Unlock band step0 failed", err);

    ec2x_dce->parent.handle_line = ec2x_handle_unlock_band_step;
    DCE_CHECK(dte->send_cmd(dte, "AT+QCFG=\"nwscanmode\",3,1\r", 5000) == ESP_OK, "send cmd failed", err);
    DCE_CHECK(ec2x_dce->parent.state == MODEM_STATE_SUCCESS, "Unlock band step1 failed", err);

    ec2x_dce->parent.handle_line = ec2x_handle_unlock_band_step;
    DCE_CHECK(dte->send_cmd(dte, "AT+QCFG=\"band\",00,45\r", 5000) == ESP_OK, "send cmd failed", err);
    DCE_CHECK(ec2x_dce->parent.state == MODEM_STATE_SUCCESS, "Unlock band step2 failed", err);
    ESP_LOGI(TAG, "Unlock band finished");
err:
    return ESP_FAIL;
#else
    return ESP_OK;
#endif
}

/**
 * @brief Get DCE module IMSI number (SIM IMSI)
 *
 * @param ec2x_dce ec2x object
 * @return esp_err_t
 *      - ESP_OK on success
 *      - ESP_FAIL on error
 */
static esp_err_t ec2x_get_imsi_number(ec2x_modem_dce_t *ec2x_dce)
{
    modem_dte_t *dte = ec2x_dce->parent.dte;
    ec2x_dce->parent.handle_line = ec2x_handle_cimi;
    DCE_CHECK(dte->send_cmd(dte, "AT+CIMI\r", MODEM_COMMAND_TIMEOUT_DEFAULT) == ESP_OK, "send cmd failed\r\n", err);
    DCE_CHECK(ec2x_dce->parent.state == MODEM_STATE_SUCCESS, "", err);
    ESP_LOGI(TAG, "get SIM IMSI OK");
    return ESP_OK;
err:
    return ESP_FAIL;
}

/**
 * @brief Get DCE module IMEI number (SIM IMEI)
 *
 * @param ec2x_dce ec2x object
 * @return esp_err_t
 *      - ESP_OK on success
 *      - ESP_FAIL on error
 */
static esp_err_t ec2x_get_sim_imei(ec2x_modem_dce_t *ec2x_dce)
{
    modem_dte_t *dte = ec2x_dce->parent.dte;
    ec2x_dce->parent.handle_line = ec2x_handle_qccid;
    DCE_CHECK(dte->send_cmd(dte, "AT+QCCID\r", MODEM_COMMAND_TIMEOUT_DEFAULT) == ESP_OK, "send cmd failed", err);
    DCE_CHECK(ec2x_dce->parent.state == MODEM_STATE_SUCCESS, "get sim imei failed", err);
    ESP_LOGI(TAG, "get SIM IMEI OK");
    return ESP_OK;
err:
    return ESP_FAIL;
}

// /**
//  * @brief Thiết lập tốc độ baudrate cho module
//  *
//  * @param ec2x_dce ec2x object
//  * @return esp_err_t
//  *      - ESP_OK on success
//  *      - ESP_FAIL on error
//  */
static esp_err_t ec2x_set_IPR(ec2x_modem_dce_t *ec2x_dce, uint32_t baudrate)
{
    modem_dte_t *dte = ec2x_dce->parent.dte;
    ec2x_dce->parent.handle_line = esp_modem_dce_handle_response_default;

	char atcmd[20] = {0};
	sprintf(atcmd, "AT+IPR=%d\r", baudrate);
    DCE_CHECK(dte->send_cmd(dte, atcmd, MODEM_COMMAND_TIMEOUT_DEFAULT) == ESP_OK, "send cmd failed", err);
    DCE_CHECK(ec2x_dce->parent.state == MODEM_STATE_SUCCESS, "set IPR failed", err);
    DEBUG_VERBOSE("Thiet lap toc do truyen du lieu OK : %d", baudrate);
    return ESP_OK;
err:
    return ESP_FAIL;
}

// /**
//  * @brief Thiết lập chế độ nhận tin nhắn SMS
//  *
//  * @param ec2x_dce ec2x object
//  * @return esp_err_t
//  *      - ESP_OK on success
//  *      - ESP_FAIL on error
//  */
// static esp_err_t ec2x_set_CNMI(ec2x_modem_dce_t *ec2x_dce)
// {
//     modem_dte_t *dte = ec2x_dce->parent.dte;
//     ec2x_dce->parent.handle_line = esp_modem_dce_handle_response_default;	//ec2x_handle_return_OK;
//     DCE_CHECK(dte->send_cmd(dte, "AT+CNMI=2,1,0,0,0\r", MODEM_COMMAND_TIMEOUT_DEFAULT) == ESP_OK, "send cmd failed", err);
//     DCE_CHECK(ec2x_dce->parent.state == MODEM_STATE_SUCCESS, "set CNMI mode failed", err);
//     ESP_LOGI(TAG, "Thiet lap che do tin nhan OK");
//     return ESP_OK;
// err:
//     return ESP_FAIL;
// }

// /**
//  * @brief Thiết lập SMS ở chế độ Text
//  *
//  * @param ec2x_dce ec2x object
//  * @return esp_err_t
//  *      - ESP_OK on success
//  *      - ESP_FAIL on error
//  */
// static esp_err_t ec2x_set_CMGF(ec2x_modem_dce_t *ec2x_dce)
// {
//     modem_dte_t *dte = ec2x_dce->parent.dte;
//     ec2x_dce->parent.handle_line = esp_modem_dce_handle_response_default; //ec2x_handle_return_OK;
//     DCE_CHECK(dte->send_cmd(dte, "AT+CMGF=1\r", MODEM_COMMAND_TIMEOUT_DEFAULT) == ESP_OK, "send cmd failed", err);
//     DCE_CHECK(ec2x_dce->parent.state == MODEM_STATE_SUCCESS, "set CMGF failed", err);
//     ESP_LOGI(TAG, "Thiet lap SMS o che do Text OK");
//     return ESP_OK;
// err:
//     return ESP_FAIL;
// }

/**
 * @brief Gửi lệnh AT+QIDEACT=1
 *
 * @param ec2x_dce ec2x object
 * @return esp_err_t
 *		- ESP_OK on success
 *		- ESP_FAIL on error
 */
static esp_err_t ec2x_set_QIDEACT(ec2x_modem_dce_t *ec2x_dce)
{
    modem_dte_t *dte = ec2x_dce->parent.dte;
    ec2x_dce->parent.handle_line = esp_modem_dce_handle_response_default; // ec2x_handle_return_OK;
    DCE_CHECK(dte->send_cmd(dte, "AT+QIDEACT=1\r", MODEM_COMMAND_TIMEOUT_DEFAULT) == ESP_OK, "send cmd failed", err);
    DCE_CHECK(ec2x_dce->parent.state == MODEM_STATE_SUCCESS, "send QIDEACT failed", err);
    ESP_LOGI(TAG, "De-active PDP OK");
    return ESP_OK;
err:
    return ESP_FAIL;
}

// /**
//  * @brief Gửi lệnh AT+QIACT=1
//  *
//  * @param ec2x_dce ec2x object
//  * @return esp_err_t
//  *		- ESP_OK on success
//  *		- ESP_FAIL on error
//  */
// static esp_err_t ec2x_set_QIACT(ec2x_modem_dce_t *ec2x_dce)
// {
// 	modem_dte_t *dte = ec2x_dce->parent.dte;
// 	ec2x_dce->parent.handle_line = esp_modem_dce_handle_response_default; //ec2x_handle_return_OK;
// 	DCE_CHECK(dte->send_cmd(dte, "AT+QIACT=1\r", MODEM_COMMAND_TIMEOUT_DEFAULT) == ESP_OK, "send cmd failed", err);
// 	DCE_CHECK(ec2x_dce->parent.state == MODEM_STATE_SUCCESS, "send QIACT failed", err);
// 	ESP_LOGI(TAG, "Active PDP OK");
// 	return ESP_OK;
// err:
// 	return ESP_FAIL;
// }

/**
 * @brief Gửi lệnh AT+CGREG=1
 *
 * @param ec2x_dce ec2x object
 * @return esp_err_t
 *		- ESP_OK on success
 *		- ESP_FAIL on error
 */
static esp_err_t ec2x_set_CGREG(ec2x_modem_dce_t *ec2x_dce)
{
    modem_dte_t *dte = ec2x_dce->parent.dte;
    ec2x_dce->parent.handle_line = esp_modem_dce_handle_response_default; // ec2x_handle_return_OK;
    DCE_CHECK(dte->send_cmd(dte, "AT+CGREG=2\r", MODEM_COMMAND_TIMEOUT_DEFAULT) == ESP_OK, "send cmd failed", err);
    DCE_CHECK(ec2x_dce->parent.state == MODEM_STATE_SUCCESS, "send CGREG failed", err);
    ESP_LOGI(TAG, "Register GPRS network OK");
    return ESP_OK;
err:
    return ESP_FAIL;
}

/**
 * @brief Gửi lệnh thiết lập APN
 *
 * @param ec2x_dce ec2x object
 * @return esp_err_t
 *		- ESP_OK on success
 *		- ESP_FAIL on error
 */
static esp_err_t ec2x_set_APN(ec2x_modem_dce_t *ec2x_dce)
{
    modem_dte_t *dte = ec2x_dce->parent.dte;
    ec2x_dce->parent.handle_line = esp_modem_dce_handle_response_default; // ec2x_handle_return_OK;
    DCE_CHECK(dte->send_cmd(dte, "AT+CGDCONT=1,\"IP\",\"v-internet\"\r", MODEM_COMMAND_TIMEOUT_DEFAULT) == ESP_OK, "set APN failed", err);
    DCE_CHECK(ec2x_dce->parent.state == MODEM_STATE_SUCCESS, "send CGREG failed", err);
    ESP_LOGI(TAG, "Thiet lap APN OK");
    return ESP_OK;
err:
    return ESP_FAIL;
}

// /**
//  * @brief Get Operator's name
//  *
//  * @param ec2x_dce ec2x object
//  * @return esp_err_t
//  *      - ESP_OK on success
//  *      - ESP_FAIL on error
//  */
// static esp_err_t ec2x_get_operator_name(ec2x_modem_dce_t *ec2x_dce)
// {
//     modem_dte_t *dte = ec2x_dce->parent.dte;
//     ec2x_dce->parent.handle_line = ec2x_handle_cops;
//     DCE_CHECK(dte->send_cmd(dte, "AT+COPS?\r", MODEM_COMMAND_TIMEOUT_OPERATOR) == ESP_OK, "send cmd failed", err);
//     DCE_CHECK(ec2x_dce->parent.state == MODEM_STATE_SUCCESS, "get network operator failed", err);
//     ESP_LOGI(TAG, "get network operator ok");
//     return ESP_OK;
// err:
//     return ESP_FAIL;
// }

/**
 * @brief Get network Band
 *
 * @param ec2x_dce ec2x object
 * @return esp_err_t
 *      - ESP_OK on success
 *      - ESP_FAIL on error
 */
static esp_err_t ec2x_get_network_band(ec2x_modem_dce_t *ec2x_dce)
{
    modem_dte_t *dte = ec2x_dce->parent.dte;
    ec2x_dce->parent.handle_line = ec2x_handle_network_band;
    DCE_CHECK(dte->send_cmd(dte, "AT+QNWINFO\r", MODEM_COMMAND_TIMEOUT_OPERATOR) == ESP_OK, "send cmd failed", err);
    DCE_CHECK(ec2x_dce->parent.state == MODEM_STATE_SUCCESS, "get network band failed", err);
    ESP_LOGI(TAG, "get network band ok");
    return ESP_OK;
err:
    return ESP_FAIL;
}

static void gsm_reset_module(void)
{
    //	//Power off module by PowerKey
    //	app_io_control_gsm_pwr_key(1);
    //	vTaskDelay(1000 / portTICK_PERIOD_MS);
    //	app_io_control_gsm_pwr_key(0);

    // Turn off Vbat +4V2
    app_io_control_gsm_power_en(0);
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    // Turn ON Vbat +4V2
    app_io_control_gsm_power_en(1);
    vTaskDelay(2000 / portTICK_PERIOD_MS); // Waiting for Vbat stable 500ms

    /* Turn On module by PowerKey */
    app_io_control_gsm_pwr_key(1);
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    app_io_control_gsm_pwr_key(0);
}

/******************************************************************************************/
/**
 * @brief   : task quản lý hoạt động của module gsm
 * @param   :
 * @retval  :
 * @author  :
 * @created :
 */
static void gsm_manager_task(void *arg)
{
    ESP_LOGI("EC2x", "\t\t--- gsm_manager_task is running ---");

    esp_err_t err;
    uint8_t gsm_init_step = 0;
    bool isInitDone = false;
    uint8_t send_at_retries_num = 0;
    uint8_t retries_get_rssi = 30;
    uint32_t m_gsm_interval = 1000;
    for (;;)
    {
        if (!isInitDone && ec2x_dce != NULL)
        {
            switch (gsm_init_step)
            {
            case 0:
                DEBUG_VERBOSE("Send cmd ATV1");
                m_gsm_interval = 500;
                uart_set_baudrate(UART_NUM_1, 115200);
                err = esp_modem_dce_sync(&(ec2x_dce->parent));
                if (err == ESP_OK)
                {
                    send_at_retries_num = 0;
                    gsm_init_step++;
                }
                else
                {
                    send_at_retries_num++;
                    if (send_at_retries_num > 15)
                    {
                        DEBUG_ERROR("Modem not response AT cmd. Reset module...\r\n");
                        send_at_retries_num = 0;
                        app_flash_node_nvs_write_u32(APP_FLASH_KEY_GSM_ERR, 1 << 1);
                        gsm_reset_module();
                    }
                }
                break;
            case 1:
                DEBUG_VERBOSE("Send cmd ATE0");
                m_gsm_interval = 100;
                err = esp_modem_dce_echo(&(ec2x_dce->parent), false);
                if (err == ESP_OK)
                {
                    send_at_retries_num = 0;
                    gsm_init_step++;
                }
                else
                {
                    send_at_retries_num++;
                    if (send_at_retries_num > 10)
                    {
                        DEBUG_ERROR("Send cmd ATE0: FAILED! Reset module...");
                        send_at_retries_num = 0;
                        gsm_init_step = 0;
                        gsm_reset_module();
                    }
                }
                break;
            case 2:
                /* Thay đổi tốc độ truyền dữ liệu UART baudrate
                * Baudrate 115200 chạy ổn định khi stream opus
                * Baudrate 230400 stream được mp3 128kbps
                */
                ESP_LOGI(TAG, "Send cmd IPR");
                m_gsm_interval = 100;
                err = ec2x_set_IPR(ec2x_dce, 230400);
                if (err == ESP_OK) 
                {
                    /* Thay đổi UART baudrate của ESP */
                    if (uart_set_baudrate(UART_NUM_1, 230400) == ESP_OK) 
                    {
                        send_at_retries_num = 0;
                        gsm_init_step++;
                    }
                }
                break;

            case 3:
                DEBUG_VERBOSE("Send cmd CGMR: Get module name");
                memset(ec2x_dce->parent.name, 0, sizeof(ec2x_dce->parent.name));
                err = ec2x_get_module_identify(ec2x_dce);
                if (err == ESP_OK)
                {
                    DEBUG_INFO("GSM FW : %s\r\n", ec2x_dce->parent.name);
                    send_at_retries_num = 0;
                    gsm_init_step++;

                    uint32_t voltage = 0, bcs = 0, bcl = 0;
                    ec2x_dce->parent.get_battery_status(&ec2x_dce->parent, &bcs, &bcl, &voltage);
                    DEBUG_INFO("Battery voltage: %d mV\r\n", voltage);
                }
                else
                {
                    send_at_retries_num++;
                    if (send_at_retries_num > 10)
                    {
                        DEBUG_ERROR("Modem not response AT command. Reset module...\r\n");
                        send_at_retries_num = 0;
                        gsm_init_step = 0;
                        gsm_reset_module();
                    }
                }
                break;
            case 4:
                DEBUG_VERBOSE("Send cmd CGSN: Get module IMEI\r\n");
                retries_get_rssi = 30;
                memset(ec2x_dce->parent.imei, 0, sizeof(ec2x_dce->parent.imei));
                err = ec2x_get_imei_number(ec2x_dce);
                if (err == ESP_OK)
                {
                    ESP_LOGI(TAG, "GSM IMEI: %s\r\n", ec2x_dce->parent.imei);
                    send_at_retries_num = 0;
                    gsm_init_step++;
                    if (strcmp(app_flash_get_imei(), ec2x_dce->parent.imei))
                        app_flash_set_imei(ec2x_dce->parent.imei);
                }
                else
                {
                    send_at_retries_num++;
                    if (send_at_retries_num > 10)
                    {
                        DEBUG_ERROR("Modem not response AT cmd. Reset module...\r\n");
                        send_at_retries_num = 0;
                        gsm_init_step = 0;
                        gsm_reset_module();
                    }
                }
                break;
            case 5:
                m_gsm_interval = 500; 
                DEBUG_VERBOSE("Send cmd CIMI: Get SIM IMSI\r\n");
                memset(ec2x_dce->parent.imsi, 0, sizeof(ec2x_dce->parent.imsi));
                err = ec2x_get_imsi_number(ec2x_dce);
                if (err == ESP_OK)
                {
                    DEBUG_VERBOSE("Get SIM IMSI: %s\r\n", ec2x_dce->parent.imsi);
                    send_at_retries_num = 0;
                    gsm_init_step++;
                }
                else
                {
                    send_at_retries_num++;
                    if (send_at_retries_num > 15)
                    {
                        DEBUG_ERROR("Get SIM IMSI: FAILED!\r\n");
                        send_at_retries_num = 0;
                        gsm_init_step = 0;
                        app_flash_node_nvs_write_u32(APP_FLASH_KEY_GSM_ERR, 1 << 0);
                        gsm_reset_module();
                    }
                }
                break;
            case 6:
                m_gsm_interval = 100;
                DEBUG_VERBOSE("Send cmd QCCID: Get SIM IMEI\r\n");
                memset(ec2x_dce->parent.sim_imei, 0, sizeof(ec2x_dce->parent.sim_imei));
                err = ec2x_get_sim_imei(ec2x_dce);
                if (err == ESP_OK)
                {
                    DEBUG_VERBOSE("Get SIM IMEI: %s\r\n", ec2x_dce->parent.sim_imei);
                    send_at_retries_num = 0;
                    gsm_init_step++;
                }
                else
                {
                    send_at_retries_num++;
                    if (send_at_retries_num > 24)
                    {
                        DEBUG_ERROR("Get SIM IMEI: FAILED!\r\n");
                        send_at_retries_num = 0;
                        send_at_retries_num = 0;
                        gsm_init_step = 0;
                        app_flash_node_nvs_write_u32(APP_FLASH_KEY_GSM_ERR, 1 << 0);
                        gsm_reset_module();
                    }
                }
                break;

                //				case 6:
                //					ESP_LOGI(TAG, "Send cmd CNMI: Thiet lap che do SMS");
                //					err = ec2x_set_CNMI(ec2x_dce);
                //					if (err == ESP_OK) {
                //						send_at_retries_num = 0;
                //						gsm_init_step++;
                //					}
                //					break;
                //				case 7:
                //					ESP_LOGI(TAG, "Send cmd CMGF: Thiet lap SMS o che do text");
                //					err = ec2x_set_CMGF(ec2x_dce);
                //					if (err == ESP_OK) {
                //						send_at_retries_num = 0;
                //						gsm_init_step++;
                //					}
                //					break;

            case 7:
                DEBUG_VERBOSE("Send cmd QIDEACT: De-active PDP context\r\n");
                err = ec2x_set_QIDEACT(ec2x_dce);
                if (err == ESP_OK)
                {
                    send_at_retries_num = 0;
                    gsm_init_step++;
                }
                else
                {
                    send_at_retries_num++;
                    if (send_at_retries_num > 15)
                    {
                        DEBUG_ERROR("De-active PDP context: FAILED!\r\n");
                        send_at_retries_num = 0;
                        gsm_init_step++;
                        app_flash_node_nvs_write_u32(APP_FLASH_KEY_GSM_ERR, 1 << 3);
                    }
                }
                break;
            case 8:
                ec2x_do_unlock_band(ec2x_dce);
                gsm_init_step++;
                break;
            case 9:
                DEBUG_VERBOSE("Send cmd CGDCONT: Thiet lap APN\r\n");
                err = ec2x_set_APN(ec2x_dce);
                if (err == ESP_OK)
                {
                    send_at_retries_num = 0;
                    gsm_init_step++;
                }
                else
                {
                    send_at_retries_num++;
                    if (send_at_retries_num > 15)
                    {
                        DEBUG_ERROR("Thiet lap APN: FAILED!\r\n");
                        send_at_retries_num = 0;
                        gsm_init_step++;
                        app_flash_node_nvs_write_u32(APP_FLASH_KEY_GSM_ERR, 1 << 4);
                    }
                }
                break;
            case 10:
                /*
                ESP_LOGI(TAG, "Send cmd QIACT: Active PDP context");
                err = ec2x_set_QIACT(ec2x_dce);
                if (err == ESP_OK) {
                    send_at_retries_num = 0;
                    gsm_init_step++;
                } else {
                    send_at_retries_num++;
                    if (send_at_retries_num > 15) {
                        DEBUG_ERROR("Active PDP context: FAILED!");
                        send_at_retries_num = 0;
                        gsm_init_step++;
                    }
                } */

                // Test: Không dùng lệnh QIACT nữa để chạy 2G/3G
                DEBUG_VERBOSE("Send cmd QNWINFO\r\n");
                ec2x_dce->parent.isGetNetworkInfo = 0;
                err = ec2x_get_network_band(ec2x_dce);
                if (ec2x_dce->parent.isGetNetworkInfo == 1)
                {
                    DEBUG_VERBOSE("Get network info: OK\r\n");
                    send_at_retries_num = 0;
                    gsm_init_step++;
                }
                else
                {
                    ESP_LOGW(TAG, "Get network info failed, retry...\r\n");
                    send_at_retries_num++;
                    if (send_at_retries_num > 25)
                    {
                        DEBUG_ERROR("Get Network band: FAILED!\r\n");
                        send_at_retries_num = 0;
                        gsm_init_step++;
                        app_flash_node_nvs_write_u32(APP_FLASH_KEY_GSM_ERR, 1 << 5);
                    }
                }
                break;
            case 11:
                DEBUG_VERBOSE("Send cmd CGREG: Register GPRS network\r\n");
                err = ec2x_set_CGREG(ec2x_dce);
                if (err == ESP_OK)
                {
                    send_at_retries_num = 0;
                    gsm_init_step++;
                }
                else
                {
                    send_at_retries_num++;
                    if (send_at_retries_num > 20)
                    {
                        DEBUG_ERROR("Register GPRS network: FAILED!\r\n");
                        send_at_retries_num = 0;
                        gsm_init_step++;
                        app_flash_node_nvs_write_u32(APP_FLASH_KEY_GSM_ERR, 1 << 6);
                    }
                }
                break;
            case 12:
                send_at_retries_num = 0;
                if (ec2x_dce != NULL)
                {
                    /* Print Module ID, Operator, IMEI, IMSI */
                    DEBUG_VERBOSE("Module name: %s, IMEI: %s\r\n", ec2x_dce->parent.name, ec2x_dce->parent.imei);
                    DEBUG_VERBOSE("SIM IMEI: %s, IMSI: %s\r\n", ec2x_dce->parent.sim_imei, ec2x_dce->parent.imsi);
                    DEBUG_VERBOSE("Network: %s,%s,%s,%s\r\n", ec2x_dce->parent.oper, ec2x_dce->parent.access_tech,
                             ec2x_dce->parent.band, ec2x_dce->parent.channel);

                    ec2x_dce->parent.set_flow_ctrl(&ec2x_dce->parent, MODEM_FLOW_CONTROL_NONE);
                    //						ec2x_dce->parent.store_profile(&ec2x_dce->parent);	//sẽ lưu cấu hình baudrate -> không dùng!

                    /* Get signal quality */
                    uint32_t rssi = 0, ber = 0;
                    ec2x_dce->parent.get_signal_quality(&ec2x_dce->parent, &rssi, &ber);
                    ESP_LOGI(TAG, "RSSI: %d, Ber: %d", rssi, ber);
                    ec2x_dce->parent.rssi = rssi;
                    if (retries_get_rssi && ec2x_dce->parent.rssi == 99)
                    {
                        m_gsm_interval = 300;
                        retries_get_rssi--;
                    }
                    else
                    {
                        m_gsm_interval = 1000;
                        retries_get_rssi = 30;
                        /* Get battery voltage */
                        uint32_t voltage = 0, bcs = 0, bcl = 0;
                        ec2x_dce->parent.get_battery_status(&ec2x_dce->parent, &bcs, &bcl, &voltage);
                        DEBUG_VERBOSE("Battery voltage: %d mV\r\n", voltage);
                        ec2x_dce->parent.gsm_vbat = voltage;
                        /* Post event init done to main task to store GSM IMEI */
                        esp_modem_dte_t *esp_dte = __containerof(ec2x_dce->parent.dte, esp_modem_dte_t, parent);
                        esp_event_post_to(esp_dte->event_loop_hdl, ESP_MODEM_EVENT, MODEM_EVENT_INIT_DONE, NULL, 0, 0);

                        /* Nếu có gắn SIM thì mới thực hiện mở ppp */
                        if (strlen(ec2x_dce->parent.imsi) >= 15)
                        {
                            isInitDone = true;

                            /* Setup PPP environment */
                            esp_modem_setup_ppp(ec2x_dce->parent.dte);
                        }
                        else
                        {
                            DEBUG_ERROR("Khong doc duoc SIM IMEI, khoi dong lai module...\r\n");
                            // Khởi động lại module GSM luôn cho chắc
                            send_at_retries_num = 0;
                            gsm_init_step = 0;
                            isInitDone = false;
                            gsm_reset_module();
                        }
                    }
                }
                else
                {
                    DEBUG_VERBOSE("ec2x_dce is NULL!\r\n");
                    system_software_reset(SW_RESET_REASON_DCE_NULL);
                    //						esp_restart();
                }
                break;

            default:
                break;
            }
        }

        vTaskDelay(m_gsm_interval / portTICK_RATE_MS);
    }

    ESP_LOGI("EC2x", "\t\t--- gsm_manager_task is exit ---\r\n");
    vTaskDelete(NULL);
}

// static void gsm_manager_task(void *arg)
//{
//     ESP_LOGI("EC2x", "\t\t--- gsm_manager_task is running ---");
//
//	esp_err_t err;
//	/* Sync between DTE and DCE : ATV1 */
//	uint8_t gsm_init_step = 0;
//	bool isInitDone = false;
//	uint8_t send_at_retries_num = 0;
//
//     for (;;) {
//         if (!isInitDone && ec2x_dce != NULL) {
//
//			send_at_retries_num++;
//			if (send_at_retries_num > 15) {
//				DEBUG_ERROR("Modem not response AT command. Reset module...");
//
//				//Power off module by PowerKey
//				app_io_control_gsm_pwr_key(1);
//				vTaskDelay(1000 / portTICK_PERIOD_MS);
//				app_io_control_gsm_pwr_key(0);
//
//				//Turn off Vbat +4V2
//				app_io_control_gsm_power_en(0);
//				vTaskDelay(3000 / portTICK_PERIOD_MS);
//
//				//Turn ON Vbat +4V2
//				app_io_control_gsm_power_en(1);
//				vTaskDelay(500 / portTICK_PERIOD_MS);	//Waiting for Vbat stable 500ms
//
//				/* Turn On module by PowerKey */
//				app_io_control_gsm_pwr_key(1);
//				vTaskDelay(1000 / portTICK_PERIOD_MS);
//				app_io_control_gsm_pwr_key(0);
//
//				send_at_retries_num = 0;
//				gsm_init_step = 0;
//
////				esp_restart();
//			}
//
//			switch(gsm_init_step) {
//				case 0:
//					ESP_LOGI(TAG, "Send cmd ATV1");
//					err = esp_modem_dce_sync(&(ec2x_dce->parent));
//					if (err == ESP_OK) {
//						send_at_retries_num = 0;
//						gsm_init_step++;
//					}
//					break;
//				case 1:
//					ESP_LOGI(TAG, "Send cmd ATE0");
//					err = esp_modem_dce_echo(&(ec2x_dce->parent), false);
//					if (err == ESP_OK) {
//						send_at_retries_num = 0;
//						gsm_init_step++;
//					}
//					break;
////				case 2:
////					/* Thay đổi tốc độ truyền dữ liệu UART baudrate
////					* Baudrate 115200 chạy ổn định khi stream opus
////					* Baudrate 230400 stream được mp3 128kbps
////					*/
////					ESP_LOGI(TAG, "Send cmd IPR");
////					err = ec2x_set_IPR(ec2x_dce, 115200);
////					if (err == ESP_OK) {
////						/* Thay đổi UART baudrate của ESP */
////						if (uart_set_baudrate(UART_NUM_1, 115200) == ESP_OK) {
////							send_at_retries_num = 0;
////							gsm_init_step++;
////						}
////					}
////					break;
////				case 3:
////					ESP_LOGI(TAG, "Send cmd CGMM: Get module name");
////					err = ec2x_get_module_name(ec2x_dce);
////					if (err == ESP_OK) {
////						send_at_retries_num = 0;
////						gsm_init_step++;
////					}
////					break;
//				case 2:
//					ESP_LOGI(TAG, "Send cmd CGSN: Get module IMEI");
//					memset(ec2x_dce->parent.imei, 0, sizeof(ec2x_dce->parent.imei));
//					err = ec2x_get_imei_number(ec2x_dce);
//					if (err == ESP_OK) {
//						send_at_retries_num = 0;
//						gsm_init_step++;
//					}
//					break;
//				case 3:
//					ESP_LOGI(TAG, "Send cmd CIMI: Get SIM IMEI");
//					memset(ec2x_dce->parent.imsi, 0, sizeof(ec2x_dce->parent.imsi));
//					err = ec2x_get_imsi_number(ec2x_dce);
//					if (err == ESP_OK) {
//						send_at_retries_num = 0;
//						gsm_init_step++;
//					}
//					break;
////				case 6:
////					ESP_LOGI(TAG, "Send cmd CNMI: Thiet lap che do SMS");
////					err = ec2x_set_CNMI(ec2x_dce);
////					if (err == ESP_OK) {
////						send_at_retries_num = 0;
////						gsm_init_step++;
////					}
////					break;
////				case 7:
////					ESP_LOGI(TAG, "Send cmd CMGF: Thiet lap SMS o che do text");
////					err = ec2x_set_CMGF(ec2x_dce);
////					if (err == ESP_OK) {
////						send_at_retries_num = 0;
////						gsm_init_step++;
////					}
////					break;
//
//				case 4:
//					ESP_LOGI(TAG, "Send cmd QIDEACT: De-active PDP context");
//					err = ec2x_set_QIDEACT(ec2x_dce);
//					if (err == ESP_OK) {
//						send_at_retries_num = 0;
//						gsm_init_step++;
//					}
//					break;
//				case 5:
//					ESP_LOGI(TAG, "Send cmd CGDCONT: Thiet lap APN");
//					err = ec2x_set_APN(ec2x_dce);
//					if (err == ESP_OK) {
//						send_at_retries_num = 0;
//						gsm_init_step++;
//					}
//					break;
//				case 6:
//					ESP_LOGI(TAG, "Send cmd QIACT: Active PDP context");
//					err = ec2x_set_QIACT(ec2x_dce);
//					if (err == ESP_OK) {
//						send_at_retries_num = 0;
//						gsm_init_step++;
//					}
//					break;
//				case 7:
//					ESP_LOGI(TAG, "Send cmd CGREG: Register GPRS network");
//					err = ec2x_set_CGREG(ec2x_dce);
//					if (err == ESP_OK) {
//						send_at_retries_num = 0;
//						gsm_init_step++;
//					}
//					break;
//				case 8:
//					isInitDone = true;
//					send_at_retries_num = 0;
//
//					if (ec2x_dce != NULL) {
//						/* Print Module ID, Operator, IMEI, IMSI */
//						ESP_LOGI(TAG, "Module: %s", ec2x_dce->parent.name);
//						ESP_LOGI(TAG, "Operator: %s", ec2x_dce->parent.oper);
//						ESP_LOGI(TAG, "IMEI: %s", ec2x_dce->parent.imei);
//						ESP_LOGI(TAG, "IMSI: %s", ec2x_dce->parent.imsi);
//
//
//						ec2x_dce->parent.set_flow_ctrl(&ec2x_dce->parent, MODEM_FLOW_CONTROL_NONE);
////						ec2x_dce->parent.store_profile(&ec2x_dce->parent);	//sẽ lưu cấu hình baudrate -> không dùng!
//
//						/* Get signal quality */
//						uint32_t rssi = 0, ber = 0;
//						ec2x_dce->parent.get_signal_quality(&ec2x_dce->parent, &rssi, &ber);
//						ESP_LOGI(TAG, "rssi: %d, ber: %d", rssi, ber);
//
//						/* Get battery voltage */
//						uint32_t voltage = 0, bcs = 0, bcl = 0;
//						ec2x_dce->parent.get_battery_status(&ec2x_dce->parent, &bcs, &bcl, &voltage);
//						ESP_LOGI(TAG, "Battery voltage: %d mV", voltage);
//
//						/* post event init done to main task */
//						esp_modem_dte_t *esp_dte = __containerof(ec2x_dce->parent.dte, esp_modem_dte_t, parent);
//						esp_event_post_to(esp_dte->event_loop_hdl, ESP_MODEM_EVENT, MODEM_EVENT_INIT_DONE, NULL, 0, 0);
//
//						/* Setup PPP environment */
//						esp_modem_setup_ppp(ec2x_dce->parent.dte);
//					}else {
//						DEBUG_ERROR("ec2x_dce is NULL!");
//						esp_restart();
//					}
//					break;
//				default:
//					break;
//			}
//    	}
//
//        vTaskDelay(1000 / portTICK_RATE_MS);
//    }
//
//    ESP_LOGI("EC2x", "\t\t--- gsm_manager_task is exit ---");
//    vTaskDelete(NULL);
//}

/**
 * @brief Deinitialize ec2x object
 *
 * @param dce Modem DCE object
 * @return esp_err_t
 *      - ESP_OK on success
 *      - ESP_FAIL on fail
 */
static esp_err_t ec2x_deinit(modem_dce_t *dce)
{
    ec2x_modem_dce_t *ec2x_dce = __containerof(dce, ec2x_modem_dce_t, parent);
    if (dce->dte)
    {
        dce->dte->dce = NULL;
    }
    free(ec2x_dce);
    return ESP_OK;
}

modem_dce_t *ec2x_init(modem_dte_t *dte)
{
    //	esp_err_t err;
    DCE_CHECK(dte, "DCE should bind with a DTE", err);

    // Deactive PowerKey
    app_io_control_gsm_pwr_key(0);

    // Turn off Vbat +4V2
    app_io_control_gsm_power_en(0);
    for (int i = 0 ; i < 4; i++)
    {
        app_io_control_gsm_power_en(0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Turn ON Vbat +4V2
    app_io_control_gsm_power_en(1);
    vTaskDelay(2000 / portTICK_PERIOD_MS); // Waiting for Vbat stable 500ms

    /* Turn On module by PowerKey */
    app_io_control_gsm_pwr_key(1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    app_io_control_gsm_pwr_key(0);

    /* malloc memory for ec2x_dce object */
    //    ec2x_modem_dce_t *ec2x_dce = calloc(1, sizeof(ec2x_modem_dce_t));
    ec2x_dce = calloc(1, sizeof(ec2x_modem_dce_t));
    DCE_CHECK(ec2x_dce, "calloc ec2x_dce failed", err);

    /* Bind DTE with DCE */
    ec2x_dce->parent.dte = dte;
    dte->dce = &(ec2x_dce->parent);

    /* Bind methods */
    ec2x_dce->parent.handle_line = NULL;
    ec2x_dce->parent.sync = esp_modem_dce_sync;
    ec2x_dce->parent.echo_mode = esp_modem_dce_echo;
    ec2x_dce->parent.store_profile = esp_modem_dce_store_profile;
    ec2x_dce->parent.set_flow_ctrl = esp_modem_dce_set_flow_ctrl;
    ec2x_dce->parent.define_pdp_context = esp_modem_dce_define_pdp_context;
    ec2x_dce->parent.hang_up = esp_modem_dce_hang_up;
    ec2x_dce->parent.get_signal_quality = ec2x_get_signal_quality;
    ec2x_dce->parent.get_battery_status = ec2x_get_battery_status;
    ec2x_dce->parent.set_working_mode = ec2x_set_working_mode;
    ec2x_dce->parent.power_down = ec2x_power_down;
    ec2x_dce->parent.deinit = ec2x_deinit;

    /**
     * @brief Tạo task quản lý hoạt động của module GSM
     */
    xTaskCreate(gsm_manager_task, "gsm_manager_task", 5 * 1024, NULL, 5, NULL);

    ESP_LOGI(TAG, "ec2x_init exit!");

    return &(ec2x_dce->parent);
// err_io:
//     free(ec2x_dce);
err:
    return NULL;
}
