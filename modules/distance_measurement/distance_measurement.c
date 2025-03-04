#include "distance_measurement.h"
#include "bsp_iic.h"

static IICInstance *distance;

void DistanceInit()
{
    IIC_Init_Config_s distance_config=
    {
        .dev_address=VL53L0X_Addr,
        .handle=&hi2c2,
        .work_mode=IIC_IT_MODE,
    };
    distance =IICRegister(&distance_config);
}

DistanceInstance *DistanceRegister(distance_Init_config_s *config)
{
    if (config->alarm_level > BUZZER_DEVICE_CNT) // 超过最大实例数,考虑增加或查看是否有内存泄漏
        while (1)
            ;
    DistanceInstance *distance = (DistanceInstance *)malloc(sizeof(DistanceInstance));
    memset(distance, 0, sizeof(DistanceInstance));
    distance->finalRangeVcselPeriod=
    distance->preRangeVcselPeriod=
    distance->sigmaLimit=
    distance->signalLimit=
    distance->timingBudget=
    return distance;
}

void DistanceTask()
{
    BuzzzerInstance *buzz;
    for (size_t i = 0; i < BUZZER_DEVICE_CNT; ++i)
    {
        buzz = buzzer_list[i];
        if (buzz->alarm_level > ALARM_LEVEL_LOW)
        {
            continue;
        }
        if (buzz->alarm_state == ALARM_OFF)
        {
            PWMSetDutyRatio(buzzer, 0);
        }
        else
        {
            PWMSetDutyRatio(buzzer, buzz->loudness);
            switch (buzz->octave)
            {
            case OCTAVE_1:
                PWMSetPeriod(buzzer, (float)1 / DoFreq);
                break;
            case OCTAVE_2:
                PWMSetPeriod(buzzer, (float)1 / ReFreq);
                break;
            case OCTAVE_3:
                PWMSetPeriod(buzzer, (float)1 / MiFreq);
                break;
            case OCTAVE_4:
                PWMSetPeriod(buzzer, (float)1 / FaFreq);
                break;
            case OCTAVE_5:
                PWMSetPeriod(buzzer, (float)1 / SoFreq);
                break;
            case OCTAVE_6:
                PWMSetPeriod(buzzer, (float)1 / LaFreq);
                break;
            case OCTAVE_7:
                PWMSetPeriod(buzzer, (float)1 / SiFreq);
                break;
            default:
                break;
            }
            break;
        }
    }
}
