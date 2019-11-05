# speech 示例工程

> speech示例工程展示了XRadio SDK中进行打断唤醒算法对接的代码实现方法。
>

---

## 适用平台

> 本工程适用以下芯片类型：
>
> 1. XR872系列芯片：XR872AT

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

## 工程配置

> localconfig.mk：
> * __CONFIG_XPLAYER: 必选项，配置使用音频功能
> 
>Makefile：
> * IMAGE_CFG: 选择工程的flash布局文件。如无，则使用默认配置
> * LINKER_SCRIPT: 选择工程的ld文件。如无，则使用默认ld文件
> * PRJ_BOARD：必选项，选择板子的板级配置路径
>
> board_config.h
> * N/A
>
> board_config.c
> * N/A
>
> prj_config.h
>
> * PRJCONF_INTERNAL_SOUNDCARD_EN: 必选项，配置使用内部声卡
> * PRJCONF_AC107_SOUNDCARD_EN: 可选项，配置使用AC107

## 模块依赖

> N/A

---

## 工程说明

> 本工程为模版工程，开发人员需要根据提示添加代码，实现打断唤醒功能。

### 操作说明

> N/A

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

### 控制命令

> N/A

### 代码结构
```
#本工程
.
├── gcc
│   ├── localconfig.mk          # 本工程的配置选项，主要用于覆盖默认全局配置
│   └── Makefile                # 本工程的编译规则，如ld文件、image.cfg、board_config.h等文件指定，可覆盖默认配置
│   ├── template.ld             # 本工程的使用的ld链接文件
├── image
│   └── xr872
│       └── image.cfg           # 本工程的镜像布局配置
├── main.c                      # 本工程的入口，完成平台初始化
├── AecAsr.c                    # 本工程的打断唤醒代码对接实现
├── AecAsr.h
├── command.c                   # 本工程的控制台命令入口
├── command.h
├── debug.c                     # 本工程的一些调试代码实现
├── debug.h
├── output.c                    # 本工程语音识别结果处理代码实现
├── output.h
├── prj_config.h                # 本工程的配置选项，主要用于功能的选择。
└── readme.md                   # 本工程的说明文档

#本工程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xradio_audio           #本工程在Makefile中指定使用xradio_audio的板级配置
                ├── board_config.h     #本工程的板级配置，
                └── board_config.c     #本工程的板级pin mux的配置。
```
### 代码流程

> 1. main()入口： 调用aec_asr_start()创建打断唤醒线程。
> 2. 播放线程函数入口：aec_asr_task()
> 3. aec_asr_task()函数流程：
>   A）完成ASR算法模块初始化
>   B）完成AEC算法模块初始化
>   C）完成录音模块初始化，开启录音
>   D）完成结果处理模块初始化
>   E）循环语句里：
>           1）通过录音模块，获取外部声音信号micsig，和喇叭回拉的参考信号aecref
>           2）将micsig和aecref输入给AEC模块，AEC模块输出纯净的pcm信号LOut
>           3）将纯净的pcm信号LOut输入给ASR模块，ASR模块进行语音识别，并输出识别结果
>           4）调wakeup_result_process，处理识别结果
> 

> 注：请根据注释，特别是TODO来完成算法对接

---


## 常见问题

> 问：程序无法正常识别关键词

答：1.首先添加打印，确保代码有按照上述代码流程正常运行，而不是异常退出；

​		2.如果程序正常运行，请先判断录音数据是否正常。可将录音数据保存到sd卡或flash，借助语音软件，如Adobe Audition进行分析

​		3.如果录音数据正常，可查看micsig和aecref是否正常

​		4.如micsig和aecref都正常，则判断语音算法是否工作正常

> 问：无法正确识别AC107，打印"AC107 init fail."

答：1.确认i2cId是否正确。在该工程里，使用检测的i2cId是I2C0_ID，这里需要根据实际电路修改board_config.c里i2c的gpio口；

​		2.检查board_config配置。电路原理图上ac107的pdmclk和pdmdata的gpio需要和board_config的g_pinmux_dmic的gpio一致


> 问：AC107能被识别，但录音数据异常，导致识别失败

答：1.检查board_config配置。电路原理图上ac107的pdmclk和pdmdata的gpio需要和board_config的g_pinmux_dmic的gpio一致；

​		2.检查AC107的供电是否正常。检测DVCC电压值是否是1.8V，VCC_DIO电压是否是3.3V

​		3.检查AC107的PDMCLK和PDMDAT。检查PDMCLK是否有时钟，其中对于采样率16k来说，时钟频率是2.048MHz；检查PDMDAT是否有数据翻转

​		4.检查XR872的PDMCLK和PDMDAT。XR872的PDMCLK和PDMDAT需要和AC107的PDMCLK和PDMDAT一致。如不一致，则检查硬件连接是否异常

​		5.检查mic电路是否正确

> 问：算法模型过大，内存不足

答：算法模型可以放到psram里。比如算法库名为xxx.a，则将该算法库全部放到psram中的代码如下：

```
    .psram_data :
    {
        . = ALIGN(4);
        __psram_data_start__ = .;

        *project/common/cmd/cmd_psram.o (.data* vtable)
        *(.psram_data*)
        *xxx.a: (*)
        . = ALIGN(4);
        __psram_data_end__ = .;
    } > PSRAM
```

> 问：出现“Rx overrun”等打印？

答：这是cpu资源不足，导致录音丢帧。尝试提高cpu频率，将运行频率高的代码放到sram中
