# RTT Manager

一个基于 Python Tkinter 的 RTT（Real-Time Transfer）启动器应用，用于管理和启动不同芯片的 RTT 调试服务，通过TCP转发到127.0.0.1 60001端口让其他应用访问openocd。可实现vofa作为上位机进行调试。

**vofa基本配置**

![alt text](image.png)

## 功能特性

- 🎯 **多调试器支持** - 支持 J-Link、CMSIS-DAP 等多种调试接口
- 🔌 **多芯片配置** - 预配置 STM32F4xx、STM32F1xx 等芯片参数
- 🖥️ **图形化界面** - 使用 Tkinter 提供简洁易用的用户界面
- 📝 **实时日志输出** - 显示服务启动状态和运行日志
- 📦 **独立可执行程序** - 支持打包成单个 EXE 文件，无需 Python 环境

## 系统要求

- Python 3.7 或更高版本
- Windows 操作系统
- 依赖包：
  - `tkinter`（通常随 Python 一起安装）

## 文件结构

```
rtt_manager/
├── rtt_manager.py      # 主应用程序
├── config.json         # 配置文件（调试器和芯片信息）
├── build_exe.py        # EXE 打包脚本
├── RTT_Manager.spec    # PyInstaller 规范文件
└── readme.md           # 本文件
```

## 安装与使用

### 1. 直接运行 Python 版本

```bash
python rtt_manager.py
```

### 2. 打包成 EXE 可执行文件

首先安装 PyInstaller：

```bash
pip install pyinstaller
```

然后运行打包脚本：

```bash
python build_exe.py
```

生成的 `RTT_Manager.exe` 将位于 `dist` 目录中。

## 配置说明

### config.json

配置文件包含两个主要部分：

#### 调试器配置 (interface)

```json
{
    "interface": [
        { "name": "Jlink", "config": "interface/jlink.cfg" },
        { "name": "CMSIS-DAP", "config": "interface/cmsis-dap.cfg" }
    ]
}
```

#### 芯片配置 (chips)

```json
{
    "chips": [
        { 
            "name": "STM32F4xx", 
            "target": "target/stm32f4x.cfg",
            "ram_start": "0x20000000",
            "ram_end": "0x20020000",
            "clock": "2000",
            "rtt_ram_size": "1024"
        }
    ]
}
```

**配置字段说明：**

| 字段 | 说明 |
|------|------|
| `name` | 芯片的显示名称 |
| `target` | 芯片目标配置文件路径 |
| `ram_start` | RTT控制块 RAM 起始地址 |
| `ram_end` | RTT控制块 RAM 结束地址 |
| `clock` | 时钟频率（单位：kHz） |
| `rtt_ram_size` | RTT 缓冲区大小 |

## 使用流程

1. 启动应用程序
2. 从下拉菜单中选择调试器（如 DAP-Link）
3. 选择目标芯片（如 STM32F4xx）
4. 点击"启动服务"按钮
5. 查看日志窗口了解服务状态

## 开发说明

### 主要类：RTTManager

- **初始化** - 加载配置文件，初始化 GUI 界面
- **load_config()** - 从 config.json 读取配置
- **update_ui_from_config()** - 根据配置更新下拉菜单
- **toggle_service()** - 启动或停止 RTT 服务

### 环境检测

**注意使用前确保系统已经安装openocd并将openocd添加到了系统的环境变量**

程序自动检测运行环境：
- **开发环境** - 从脚本所在目录读取 config.json
- **打包环境** - 从 EXE 所在目录读取 config.json

## 支持

如有问题或建议，欢迎提出反馈。