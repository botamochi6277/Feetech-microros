#ifndef _FEETECH_UTILS_HPP
#define _FEETECH_UTILS_HPP
#include "SCServo.h"

static const float PI_F = 3.141f;

template <typename T>
T remap(T x, T in_min, T in_max, T out_min, T out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void feetechWrite(SMS_STS &st, unsigned char id,
                  float angle, float speed = PI_F, float acc = 1.0f)
{
    short pos_cnt = (short)remap(angle, -PI_F, PI_F, 0.0f, 4095.0f);
    unsigned short speed_cnt = (unsigned short)remap(speed, 0.0f, 2.0f * PI_F, 0.0f, 4095.0f);
    unsigned char acc_cnt = (unsigned char)remap(acc, 0.0f, 2.0f * PI_F, 0.0f, 4095.0f);
    st.RegWritePosEx(id, pos_cnt, speed_cnt, acc_cnt);
}

void wakeup_sweep(SMS_STS &st, float angles[], int length_angle, int ids[], int length_id, int interval = 1000)
{
    for (size_t j = 0; j < length_angle; j++)
    {
        for (size_t i = 0; i < length_id; i++)
        {
            feetechWrite(st, ids[i], angles[j]);
        }
        st.RegWriteAction();
        delay(interval);
    }
}

#endif