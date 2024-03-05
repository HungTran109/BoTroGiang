#ifndef LPF_H
#define LPF_H

#include <stdint.h>

typedef  struct
{
    int32_t estimate_value;
    uint8_t gain;               // 0 to 100%
} lpf_data_t;

/*!
 * @brief           Low pass filter function
 * @param[in]       current Current data
 * @param[in]       measure New data
 * @note            Old = Old - Gain*(Old - new)     Gain : percent
 */
void lpf_update_estimate(lpf_data_t *current, int32_t *measure);

#endif /* LPF_H */
