# bsp_log

## 使用说明

bsp_log是基于segger RTT实现的日志打印模块。

`bsp_log.h`中提供了三级日志：

```c
#define LOGINFO(format,...)
#define LOGWARNING(format,...)
#define LOGERROR(format,...)
```

分别用于输出不同等级的日志，用于系统级别的日志输出。注意RTT不支持直接使用`%f`进行浮点格式化,要使用`void Float2Str(char *str, float va);`转化成字符串之后再发送。

**若想用vofa调试，必须通过配套的rtt启动器启动调试（不论使用什么调试器，当前rtt只支持daplink和jlink）。** 如果发现vofa数据不刷新并不是vofa卡死的问题，请查看当前给rtt的ram大小是否过小，导致数据溢出。关于调试的配置文件查看rtt启动器的config.json，关于config.json的配置方法查看rtt启动器的readme.md。
CCMRAM是cortexM4的专用的高速内存，f4芯片可以开启定义用于加速，但f1或其他芯片要关闭使用普通的ram，CCMRAM的大小在bsp_log.h中定义，默认为1024，普通ram的大小在SEGGER_RTT_Conf.h的BUFFER_SIZE_UP中定义，默认为1024。

## 自定义输出

你也可以自定义输出格式，详见Segger RTT的文档。

```c
int printf_log(const char *fmt, ...);
void Float2Str(char *str, float va); // 输出浮点需要先用此函数进行转换
```

调用printf_log函数，可以通过jlink或dap-link向调试器连接的上位机发送信息，格式和printf相同，示例如下：

```c
printf_log("Hello World!\n");
printf_log("Motor %d met some problem, error code %d!\n",3,1);
```

Float2Str函数可以将浮点类型转换成字符串以方便发送：

```c
    float a = -45.121;
    float b = 3.1415926f;
    char printf_buf[10];//这个变量已经在bsp_log.c中定义是全局变量不需要每次使用的时候在函数中声明本质上是储存字符串的内存缓存区
    Float2Str(printf_buf,a);
    PrintLog("a=%s\n",printf_buf);//默认显示在终端0上
```

## 波形输出

想向vofa输出波形必须使用RTT_PrintWave_vofa函数：

```c
float a = -45.121;
float b = 3.1415926f;
void RTT_PrintWave_vofa(2, a, b);
```

## 注意
在任何一个文件使用关于rtt的函数都需要包含`bsp_log.h`头文件。