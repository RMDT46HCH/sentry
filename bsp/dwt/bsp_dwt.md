# dwt
## 获取当前时间
```c
float t;
t=DWT_DetTimeline_ms();
```
## 计算时间间隔
如卡弹检测：
//随便乱写的，看具体测试
```c
typedef enum
{
    LOAD_NORMAL = 0,  // 停止发射
    LOAD_WARN,
    LOAD_ERROR,
    LOAD_RECOVER, 
} loader_state_e;

static loader_state_e loader_state;
float tstart = DWT_GetTimeline_s();
float dt;
if(abs(loader->measure.real_current)>9.5)
{
    dt= DWT_GetTimeline_s() - tstart;
}
else 
{
    dt=0;
}

if(loader_state==LOAD_WARN&&dt>0.3)
{
    loader_state=LOAD_ERROR;
}
else if(loader_state==LOAD_ERROR)
{
    loader_state=LOAD_RECOVER;
    shoot_cmd_send.load_mode==LOAD_REVERSE;
}
else if(loader_state==LOAD_RECOVER&&abs(loader->measure.real_current)<9.5)
{
    loader_state=LOAD_NORMAL;
    shoot_cmd_send.load_mode==LOAD_BURSTFIRE;
}

```


