#ifndef BSP_INIT_h
#define BSP_INIT_h

#include "bsp_init.h"
#include "bsp_dwt.h"
#include "buzzer.h"
/**
 * @brief bsp层初始化统一入口,这里仅初始化必须的bsp组件,其他组件的初始化在各自的模块中进行
 *        需在实时系统启动前调用,目前由RobotoInit()调用
 * 
 * @note 其他实例型的外设如CAN和串口会在注册实例的时候自动初始化,不注册不初始化
  */
// 
void BSPInit()
{
    DWT_Init(168);
    BuzzerInit();
}


#endif // !BSP_INIT_h

