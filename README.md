# 基于CAN总线的永磁同步电机控制系统设计与实现


## 项目简介
本设计硬件部分MCU使用的STM32F405，通讯方式采用CAN总线协议，集成了MOS驱动和电流运算放大器的驱动芯片DRV8323S。核心控制算法为磁场定向控制FOC算法，可通过三个PID控制环精确的控制电机的电流、速度及位置。

---



---

## 📂 项目结构
```bash
/project
 ├── hardware/     # 硬件设计文件（原理图/PCB/Gerber）
 ├── firmware/     # STM32 固件代码（C/FreeRTOS/HAL）
 ├── docs/         # 文档、说明书、设计报告
 ├── examples/     # 示例程序
 └── LICENSE       # 开源协议
