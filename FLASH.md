# TN20 控制台主板固件烧录说明

本文记录 `C:\tn20_test\code\servo_driver` 工程的实际烧录流程，以及本次已验证通过的环境与排错方法。

## 1. 工程位置

- 固件工程目录：`C:\tn20_test\code\servo_driver`
- 目标芯片：`ESP32`
- 工程类型：`ESP-IDF`

## 2. 已验证通过的现象

烧录成功后：

- 串口可看到 ESP32 启动日志
- 控制台主板 OLED 会显示 `HELLO`

这说明当前仓库里的固件已经成功运行在主板上。

## 3. 必备环境

### 3.1 ESP-IDF

本项目已验证通过的版本：

- `ESP-IDF v5.5.5`

建议使用 ESP-IDF Installation Manager 安装：

- Target：`esp32`
- IDF Version：`v5.5.5`

### 3.2 USB 转串口驱动

本板实际识别到的下载芯片为：

- `CP2102 USB to UART Bridge Controller`

如果设备管理器里该设备带黄色感叹号，需要先安装 `Silicon Labs CP210x Universal Windows Driver`。

驱动安装完成后，设备应出现在：

- `端口 (COM 和 LPT)`

而不是：

- `其他设备`

## 4. 如何找对串口

本次排查结果：

- `COM4` / `COM5`：蓝牙串口，不是主板下载口
- `COM7`：主板真实串口

建议用下面方法确认：

1. 打开设备管理器
2. 展开 `端口 (COM 和 LPT)`
3. 插拔主板 USB
4. 观察哪个 `COM` 口随插拔出现/消失

找到真实串口后，用它替换下面示例中的 `COM7`。

## 5. 推荐烧录方式

### 5.1 推荐方式：IDF PowerShell + 命令行

先打开 `ESP-IDF PowerShell`，进入工程目录：

```powershell
cd C:\tn20_test\code\servo_driver
```

### 5.2 先验证串口

```powershell
idf.py -p COM7 monitor
```

如果串口正确，通常会看到类似日志：

```text
ets Jul 29 2019 12:21:46
rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
```

按一下板子的 `EN/RESET`，应再次打印启动日志。

退出监视器：

```text
Ctrl+]
```

### 5.3 构建

本机 Windows 环境下，直接 `idf.py build` 曾遇到静态库打包失败，表现为：

```text
xtensa-esp-elf-ranlib.exe: unable to copy file '...libxxx.a'; reason: Invalid argument
```

已验证有效的稳定构建方式：

```powershell
$env:IDF_CCACHE_ENABLE="0"
$env:CMAKE_BUILD_PARALLEL_LEVEL="1"
Remove-Item -LiteralPath .\build -Recurse -Force
idf.py build
```

说明：

- 关闭 `ccache`
- 限制并行度为 `1`
- 删除旧 `build`
- 再重新构建

### 5.4 烧录

```powershell
idf.py -p COM7 -b 115200 flash
```

### 5.5 监视运行日志

```powershell
idf.py -p COM7 monitor
```

## 6. VS Code 烧录

本项目后来也已验证可通过 VS Code 的 ESP-IDF 扩展烧录。

当前工作区建议串口配置：

```json
{
  "idf.currentSetup": "C:\\esp\\v5.5.5\\esp-idf",
  "idf.flashType": "UART",
  "idf.port": "COM7",
  "idf.monitorPort": "COM7",
  "idf.portWin": "COM7",
  "idf.flashBaudRate": 115200
}
```

说明：

- `COM7` 是本机这次验证时的真实串口
- 换电脑或换 USB 口后，端口号可能变化

VS Code 成功烧录时，扩展输出会出现：

```text
[Flash]
Flash Done
```

## 7. 手动进入下载模式

如果执行 `flash` 时卡在：

```text
Connecting...
```

可以手动让 ESP32 进入下载模式：

1. 按住 `BOOT`
2. 轻按一下 `EN`
3. 松开 `EN`
4. 约 1 秒后松开 `BOOT`
5. 再执行烧录命令

## 8. 常见问题

### 8.1 `Failed to connect to ESP32: No serial data received`

通常表示：

- 串口选错
- 不是主板真实下载口
- 没进入下载模式
- USB 转串口驱动没装好

### 8.2 `COM4` / `COM5` 能看到，但不能用

本次确认这两个口是蓝牙串口，不是主板串口。

### 8.3 `CP2102 USB to UART Bridge Controller` 带黄叹号

说明驱动没装好，先装 `CP210x Universal Windows Driver`。

### 8.4 `fullclean` 是否失败

如果输出只有：

```text
Executing action: fullclean
Executing action: remove_managed_components
Done
```

则表示：

- `fullclean` 成功

若后面紧接着 `idf.py build` 失败，那是 `build` 失败，不是 `fullclean` 失败。

### 8.5 VS Code 提示 Flash Encryption

本项目实际未启用 flash encryption。

如果扩展弹出类似提示，但命令行构建与烧录正常，可优先以命令行为准。

## 9. 最稳的完整流程

```powershell
cd C:\tn20_test\code\servo_driver
$env:IDF_CCACHE_ENABLE="0"
$env:CMAKE_BUILD_PARALLEL_LEVEL="1"
Remove-Item -LiteralPath .\build -Recurse -Force
idf.py build
idf.py -p COM7 -b 115200 flash
idf.py -p COM7 monitor
```

如果只是已经编译好、只想快速更新固件，可以直接：

```powershell
idf.py -p COM7 -b 115200 flash
```

