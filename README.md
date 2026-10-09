# TN20 Test

TN20 控制台按键测试相关结构与固件工程。

本仓库主要包含两部分：

- 结构设计文件（`3d/`）
- 控制台主板 ESP32 固件工程（`code/servo_driver/`）

## 30 秒快速开始

- 想快速了解仓库：先看本文档下面的“仓库结构”和“固件工程要点”
- 想烧录主板：直接看 [FLASH.md](C:\tn20_test\FLASH.md)
- 想看核心代码：优先读 [servo_driver.c](C:\tn20_test\code\servo_driver\main\servo_driver.c) 和 [servo_task.c](C:\tn20_test\code\servo_driver\main\servo_task.c)
- 想让 Agent 高效接手：先让它读 [README.md](C:\tn20_test\README.md)、[FLASH.md](C:\tn20_test\FLASH.md)，再聚焦 `code/servo_driver/`

## 1. 先看什么

如果你是第一次接手，建议按这个顺序阅读：

1. 本文档 `README.md`
2. [FLASH.md](C:\tn20_test\FLASH.md)
3. `code/servo_driver/` 固件工程

如果你的目标是烧录主板，直接重点看：

- [FLASH.md](C:\tn20_test\FLASH.md)
- [CMakeLists.txt](C:\tn20_test\code\servo_driver\CMakeLists.txt)
- [sdkconfig](C:\tn20_test\code\servo_driver\sdkconfig)

## 2. 仓库结构

```text
C:\tn20_test
├─ 3d/                          结构设计文件
├─ code/
│  └─ servo_driver/             ESP-IDF 固件工程
├─ FLASH.md                     烧录与环境说明
├─ README.md                    项目总入口
└─ Servo_Driver_with_ESP32.pdf  相关参考资料
```

`code/servo_driver/` 目录结构：

```text
code/servo_driver
├─ CMakeLists.txt
├─ sdkconfig
├─ main/
│  ├─ servo_driver.c
│  ├─ servo_task.c
│  ├─ uart_receiver.c
│  ├─ at_parser.c
│  ├─ system_commands.c
│  └─ drv_oled.c
├─ build/                       当前构建输出（生成目录）
└─ build_stale_20260918_1/      历史构建缓存备份（生成目录）
```

## 3. 项目用途

该固件运行在控制台主板的 `ESP32` 上，主要功能包括：

- OLED 显示启动内容
- 串口接收 AT 命令
- 控制舵机执行按键动作
- 提供部分系统命令与状态读取能力

当前已验证通过的现象：

- 固件烧录成功后，OLED 会显示 `HELLO`
- 串口可看到 ESP32 启动日志

## 4. 固件工程要点

### 4.1 工程类型

- 框架：`ESP-IDF`
- 目标芯片：`ESP32`

关键文件：

- 工程入口：[CMakeLists.txt](C:\tn20_test\code\servo_driver\CMakeLists.txt)
- 工程配置：[sdkconfig](C:\tn20_test\code\servo_driver\sdkconfig)
- 应用入口：[servo_driver.c](C:\tn20_test\code\servo_driver\main\servo_driver.c)

### 4.2 关键模块

- [servo_driver.c](C:\tn20_test\code\servo_driver\main\servo_driver.c)
  - `app_main()` 入口
  - 初始化 OLED、舵机、串口命令系统

- [servo_task.c](C:\tn20_test\code\servo_driver\main\servo_task.c)
  - 舵机通信与动作控制
  - 包含按钮 GPIO 与串口 1 配置

- [uart_receiver.c](C:\tn20_test\code\servo_driver\main\uart_receiver.c)
  - `UART0` 命令接收

- [at_parser.c](C:\tn20_test\code\servo_driver\main\at_parser.c)
  - 解析 `AT+...` 指令

- [system_commands.c](C:\tn20_test\code\servo_driver\main\system_commands.c)
  - 版本、SN、睡眠等系统命令

- [drv_oled.c](C:\tn20_test\code\servo_driver\main\drv_oled.c)
  - OLED 驱动

## 5. 如何烧录

烧录说明已单独整理到：

- [FLASH.md](C:\tn20_test\FLASH.md)

其中包含：

- ESP-IDF 安装
- CP2102 驱动安装
- 串口识别方法
- `build / flash / monitor` 命令
- Windows 下构建失败的规避方法

## 6. 当前已知经验

### 6.1 正确串口

本机本次验证时，控制台主板真实串口为：

- `COM7`

但这个值依赖电脑、USB 口和驱动环境，换机器后应重新确认。

### 6.2 Windows 构建注意事项

在本机环境中，曾出现 `ranlib` 处理静态库失败的问题。已验证的稳定构建方式见 [FLASH.md](C:\tn20_test\FLASH.md)。

### 6.3 生成目录不要手改

以下目录/文件属于构建输出或历史缓存：

- `code/servo_driver/build/`
- `code/servo_driver/build_stale_20260918_1/`

除非是在排查构建问题，否则不要把它们当源码阅读重点。

## 7. 给 Agent 的快速入口

如果是 Agent/自动化工具接手本仓库，建议遵循以下阅读顺序。

### 7.1 优先阅读

1. [README.md](C:\tn20_test\README.md)
2. [FLASH.md](C:\tn20_test\FLASH.md)
3. [CMakeLists.txt](C:\tn20_test\code\servo_driver\CMakeLists.txt)
4. [sdkconfig](C:\tn20_test\code\servo_driver\sdkconfig)
5. [servo_driver.c](C:\tn20_test\code\servo_driver\main\servo_driver.c)
6. [servo_task.c](C:\tn20_test\code\servo_driver\main\servo_task.c)
7. [uart_receiver.c](C:\tn20_test\code\servo_driver\main\uart_receiver.c)
8. [at_parser.c](C:\tn20_test\code\servo_driver\main\at_parser.c)
9. [system_commands.c](C:\tn20_test\code\servo_driver\main\system_commands.c)

### 7.2 默认忽略

除非任务明确要求，否则默认不要优先深入这些目录：

- `3d/`
- 根目录 `build/`
- `code/servo_driver/build/`
- `code/servo_driver/build_stale_20260918_1/`

原因：

- 这些目录大多是结构文件、构建产物或历史缓存
- 对理解主逻辑帮助不大
- 容易把注意力带偏到生成文件

### 7.3 面向任务的入口建议

- 如果任务是“烧录/连板/串口”：
  - 先读 [FLASH.md](C:\tn20_test\FLASH.md)

- 如果任务是“理解固件行为”：
  - 先读 [servo_driver.c](C:\tn20_test\code\servo_driver\main\servo_driver.c)
  - 再读 [servo_task.c](C:\tn20_test\code\servo_driver\main\servo_task.c)
  - 再读 [at_parser.c](C:\tn20_test\code\servo_driver\main\at_parser.c)

- 如果任务是“修改 AT 指令”：
  - 先读 [at_parser.c](C:\tn20_test\code\servo_driver\main\at_parser.c)
  - 再读 [system_commands.c](C:\tn20_test\code\servo_driver\main\system_commands.c)

- 如果任务是“修改显示行为”：
  - 先读 [drv_oled.c](C:\tn20_test\code\servo_driver\main\drv_oled.c)
  - 再读 [servo_driver.c](C:\tn20_test\code\servo_driver\main\servo_driver.c)

## 8. 建议的提问方式

如果要让 Agent 更高效处理，不建议只说“读取整个项目树”。

更推荐这样描述：

```text
请先阅读 README.md 和 FLASH.md，
然后重点分析 code/servo_driver 下的 ESP-IDF 工程。
如果要看烧录逻辑，优先关注：
- code/servo_driver/CMakeLists.txt
- code/servo_driver/sdkconfig
- code/servo_driver/main/servo_driver.c
```

这种方式能减少无关扫描，提高理解速度。
