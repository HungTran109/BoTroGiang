#include "lpf.h"

void lpf_update_estimate(lpf_data_t *current, int32_t *measure)
{
    // Old = Old - Gain*(Old - new)     Gain : percent
    int32_t diff = current->estimate_value - *measure;
    if (diff)
    {
        current->estimate_value = current->estimate_value - current->gain*(diff)/100;
    }
}
