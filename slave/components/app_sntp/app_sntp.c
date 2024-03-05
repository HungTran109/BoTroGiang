/*
 * app_sntp.c
 *
 *  Created on: Apr 27, 2021
 *      Author: huybk
 */

#include "app_sntp.h"
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sntp.h"
#include "esp_log.h"

static const char *TAG = "app_sntp";

static void initialize_sntp(void);
static bool m_timstamp_found = false;
void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
    struct timespec tp;
#if 0
    time_t now = 0;
    struct tm timeinfo = { 0 };
    time(&now);
    setenv("TZ", "CST-7", 1);
	tzset();
	localtime_r(&now, &timeinfo);
    tp.tv_sec = now;
#else
    tp.tv_sec = tv->tv_sec;
#endif
   	clock_settime(CLOCK_REALTIME, &tp);

    m_timstamp_found = true;
}

void app_sntp_start(void)
{
	initialize_sntp();
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    time_t now = 0;
    struct tm timeinfo = { 0 };
    time(&now);
    char strftime_buf[64];
    setenv("TZ", "CST-7", 1);
	tzset();
	localtime_r(&now, &timeinfo);
	strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
	ESP_LOGI(TAG, "The current date/time in Vietnam is: [%u] %s", (uint32_t)time(NULL), strftime_buf);

    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    // Set NTP server
    sntp_setservername(0, "pool.ntp.org");
    sntp_setservername(1, "time.google.com");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();
}


void app_sntp_debug_timenow(void)
{
    time_t now = 0;
    struct tm timeinfo = { 0 };
    time(&now);
    localtime_r(&now, &timeinfo);

    char strftime_buf[64];
    setenv("TZ", "CST-7", 1);
	tzset();
	localtime_r(&now, &timeinfo);
	strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
	ESP_LOGI(TAG, "The current date/time in Vietnam is: [%u] %s", (uint32_t)time(NULL), strftime_buf);
}

uint32_t app_sntp_timestamp(void)
{
	return time(NULL);
}

