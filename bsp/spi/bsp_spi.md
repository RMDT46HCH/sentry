# spi 即Serial Peripheral Interface（串行外设接口）
## 小知识：
### 特点：
- 主从模式通信，支持一主多从或多主多从。
- 全双工通信，支持同时发送和接收数据。
- 高速、同步通信，适合短距离数据传输。
### 信号线：
- SCLK：时钟信号，由主机产生。
- MOSI：Master Out Slave In，主机发送数据到从机。
- MISO：Master In Slave Out，从机发送数据到主机。
- CS/SS：片选信号，用于选择从设备。

- 每定义一个`spi_instance`，就代表一个spi的**实例**（对象）。

## .c中的函数：
- `SPIInstance *SPIRegister(SPI_Init_Config_s *conf);`通过调用此函数 将一个SPI设备配置信息注册到系统中。
- 使用实例：
```c
    SPIInstance *spi_gyro; // 注意,SPIInstnace内部也有一个GPIOInstance,用于控制片选CS
    SPIInstance *spi_acc;  // 注意,SPIInstnace内部也有一个GPIOInstance,用于控制片选CS
    config->spi_gyro_config.spi_work_mode = SPI_DMA_MODE; // 如果DMA资源不够,可以用SPI_IT_MODE
    config->spi_gyro_config.spi_work_mode = SPI_DMA_MODE;
    config->spi_acc_config.callback = BMI088AccSPIFinishCallback;
    config->spi_gyro_config.callback = BMI088GyroSPIFinishCallback;
    bmi088_instance->spi_acc = SPIRegister(&config->spi_acc_config);
    bmi088_instance->spi_gyro = SPIRegister(&config->spi_gyro_config);
```

`void SPITransmit(SPIInstance *spi_ins, uint8_t *ptr_data, uint8_t len);`
- 拉低片选信号（HAL_GPIO_WritePin），选中从设备。
- 选择工作模式（DMA、中断、阻塞）。
- 阻塞模式下，发送完成后自动拉高片选信号。
- 使用示例
```c
SPITransmit(bmi088->spi_acc, tx, 2);
uint8_t tx[2] = {reg, data};
```

`void SPITransRecv(SPIInstance *spi_ins, uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint8_t len);`
- 使用实例
```c
    SPIInstance *spi_gyro; // 注意,SPIInstnace内部也有一个GPIOInstance,用于控制片选CS
    SPIInstance *spi_acc;  // 注意,SPIInstnace内部也有一个GPIOInstance,用于控制片选CS
    static uint8_t len;
    static uint8_t tx[8]; 
    static uint8_t rx[8]; 
    tx[0] = 0x80 | reg;
    SPITransRecv(bmi088->spi_acc, rx, tx, len + 2);
```
### 对HAL（spi.h）的回调函数的重定义(你可能发现以下函数并不在bsp_spi.h里。其实是对HAL（spi.h）的回调函数的重定义)
- 接收：
`void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);`
调用模块层里设置的回调函数