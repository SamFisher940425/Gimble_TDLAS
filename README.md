# Gimble_TDLAS
STM32F103C8T6 board code in "Gimble_TDLAS" project

此工程采用STM32 CubeMX工具生成，使用FreeRTOS V2.0系统

主要实现功能：

其中1路485接收主站发送的查询或控制命令，另外1路485以及232、CAN用来控制两个电机和多种传感器

PWM输出控制雨刮器舵机
