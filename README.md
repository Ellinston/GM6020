# GM6020

使用 STM32 控制大疆 RoboMaster GM6020 电机的实验工程。

## 开发板版本

### 达妙 STM32H723 开发板

仓库根目录保留原始达妙开发板工程：

- MCU：STM32H723VGT6
- CAN：FDCAN2，PB5/RX、PB6/TX，1 Mbps
- 串口：USART1，PA9/TX、PA10/RX，115200

### STM32F407 最小系统板

工程位于 [`boards/DM6020_F407`](boards/DM6020_F407)：

- MCU：STM32F407VET6
- CAN：CAN1，PD0/RX、PD1/TX，1 Mbps
- 串口：USART1，PA9/TX、PA10/RX，115200
- 外部晶振：8 MHz，系统时钟 168 MHz
- 包含上电 CAN 内部回环自检及 CAN 错误诊断变量

两个目录都是独立的 STM32CubeIDE 工程。请根据实际开发板导入对应目录，避免用错目标芯片和烧录文件。
