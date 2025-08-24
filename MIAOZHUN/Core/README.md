# 这是一个说明文档

## 项目概述

这是一个基于 STM32F4 的双轴云台控制系统，主要用于瞄准和跟踪应用。系统采用步进电机驱动，支持串口通信协议，实现了精确的位置控制和数据传输功能。

## 目录结构

```
Core/
├── Inc/                    # 头文件目录
│   ├── main.h             # 主程序头文件
│   ├── DATOU.h            # 云台控制头文件
│   ├── frame.h            # 数据帧处理头文件
│   ├── frame_parser.h     # 帧解析器头文件（空文件）
│   ├── Key.h              # 按键处理头文件
│   ├── usart.h            # 串口通信头文件
│   ├── dma.h              # DMA控制头文件
│   ├── gpio.h             # GPIO控制头文件
│   ├── stm32f4xx_hal_conf.h   # HAL库配置文件
│   └── stm32f4xx_it.h     # 中断处理头文件
└── Src/                    # 源文件目录
    ├── main.c             # 主程序源文件
    ├── DATOU.c            # 云台控制源文件
    ├── frame.c            # 数据帧处理源文件
    ├── frame_parser.c     # 帧解析器源文件
    ├── Key.c              # 按键处理源文件
    ├── usart.c            # 串口通信源文件
    ├── dma.c              # DMA控制源文件
    ├── gpio.c             # GPIO控制源文件
    ├── stm32f4xx_hal_msp.c    # HAL MSP配置文件
    ├── stm32f4xx_it.c     # 中断处理源文件
    └── system_stm32f4xx.c # 系统配置文件
```

## 模块功能说明

### 1. 云台控制模块 (DATOU.h/c)

负责双轴云台的精确位置控制，支持步进电机驱动。

**主要功能：**
- X轴和Y轴的位置控制
- 零点设置和回零功能
- 参数配置和停止控制

**关键函数：**
```c
void set_xzero(void);                    // 设置X轴零点
void set_yzero(void);                    // 设置Y轴零点
void zero_param_x(void);                 // X轴零点参数配置
void zero_param_y(void);                 // Y轴零点参数配置
void pos_control_x(float angle_deg);     // X轴位置控制（角度输入）
void pos_control_y(float angle_deg);     // Y轴位置控制（角度输入）
void stop(void);                         // 停止运动
void Remove(void);                       // 移除功能
```

**技术参数：**
- 步距角：1.8°
- 微步数：16
- 减速比：1.0
- 控制精度：基于脉冲数精确定位
- 通信接口：UART3（X轴）、UART6（Y轴）

### 2. 数据帧处理模块 (frame.h/c)

实现串口通信协议的数据封装和解析功能。

**数据结构：**

**发送数据结构 (DataTransmit)：**
```c
typedef struct {
    uint8_t head1;                    // 帧头1
    uint8_t head2;                    // 帧头2
    uint8_t length;                   // 有效数据长度
    uint8_t cnt;                      // 总数据长度
    uint8_t data[40];                 // 有效数据数组
    uint8_t transmit_data[50];        // 实际发送数组
} DataTransmit;
```

**接收数据结构 (DataReceive)：**
```c
typedef struct {
    uint8_t head1, head2;             // 帧头
    uint8_t length;                   // 有效数据长度
    uint8_t cnt;                      // 总数据长度
    uint8_t state;                    // 接收状态
    uint8_t i;                        // 数据下标
    uint8_t receive_data[50];         // 接收数组
    uint8_t data;                     
    uint8_t complete;                 // 接收完成标志
} DataReceive;
```

**目标属性结构 (TargetProperty)：**
```c
typedef struct {
    uint16_t x;                       // 目标X轴坐标
    uint16_t y;                       // 目标Y轴坐标
    uint8_t flag;                     // 目标标志位
} TargetProperty;
```

**关键函数：**
```c
void Data_Transmit_Init(DataTransmit *data, uint8_t head1, uint8_t head2, uint8_t length);
void Data_Pack(DataTransmit *data);
void Data_Receive_Init(DataReceive *data, uint8_t head1, uint8_t head2);
void Data_Receive(DataReceive *data, uint8_t buf);
void Data_Receive_Custom(DataReceive *data, uint8_t buf);
```

### 3. 按键处理模块 (Key.h/c)

提供多按键输入检测功能，支持5个按键。

**功能特点：**
- 支持5个独立按键（key1-key5）
- 防抖处理（10ms延时）
- 全局按键状态变量

**关键函数：**
```c
uint8_t Key_getnum(void);             // 获取按键编号（1-5）
```

**使用说明：**
- 返回值：1-5表示对应按键被按下，0表示无按键按下
- 按键检测包含简单的软件防抖处理

### 4. 串口通信模块 (usart.h/c)

提供多路串口通信功能，支持DMA传输。

**通信配置：**
- UART1：用于接收MaixCam数据
- UART2：用于发送调试信息
- UART3：X轴步进电机控制
- UART6：Y轴步进电机控制

### 5. DMA模块 (dma.h/c)

提供高效的数据传输功能，减少CPU占用。

### 6. GPIO模块 (gpio.h/c)

管理所有GPIO引脚的配置和控制。

## 通信协议

### 步进电机控制协议

**位置控制命令格式：**
```
地址 + 0xFD + 方向 + 速度 + 加速度 + 脉冲数 + 模式标志 + 同步标志 + 校验字节
```

**示例：**
```
01 FD 01 01 2C 00 xx xx xx xx 01 00 6B
```

**参数说明：**
- 方向：0x00=CW（顺时针），0x01=CCW（逆时针）
- 速度：0x012C = 300 RPM
- 模式：0x00=相对位置，0x01=绝对位置
- 脉冲数计算：`脉冲数 = 角度(°) ÷ 360° × PPR`

### 零点设置协议

**回零命令：**
```
01 9A 00 00 6B
```

**零点参数配置：**
```
01 4C AE 01 00 00 00 1E 00 00 27 10 01 2C 03 20 00 3C 00 6B
```

## 编译和使用

### 环境要求

- STM32CubeMX（项目配置）
- Keil MDK-ARM 或 STM32CubeIDE
- STM32F4xx HAL库
- 目标芯片：STM32F407VETx

### 编译步骤

1. 打开项目文件（.uvprojx 或 STM32CubeIDE工作空间）
2. 确保所有依赖库已正确配置
3. 编译项目生成可执行文件
4. 通过调试器下载到目标板

### 使用说明

1. **系统初始化**：上电后自动初始化所有外设
2. **零点设置**：首次使用需要设置X、Y轴零点
3. **位置控制**：通过串口发送目标坐标实现精确定位
4. **状态监控**：通过调试串口输出系统状态信息

## 注意事项

1. **电源要求**：确保步进电机驱动器有足够的电源供应
2. **通信波特率**：确保所有串口波特率配置正确
3. **安全限位**：使用前确认机械限位开关工作正常
4. **调试模式**：开发阶段建议开启调试串口输出

## 开发者信息

- 项目类型：STM32嵌入式项目
- 开发环境：STM32CubeMX + Keil MDK-ARM
- 版权：STMicroelectronics (2025)
- 许可证：参见根目录LICENSE文件

## 更新日志

- 2025年：初始版本完成，实现基本的双轴云台控制功能
- 支持步进电机精确位置控制
- 实现串口通信协议和数据帧处理
- 添加按键输入和DMA传输功能

---

如需技术支持或有任何问题，请查看项目文档或联系开发团队。
