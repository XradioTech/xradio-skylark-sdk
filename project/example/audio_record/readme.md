# audio record 示例工程

> audio record示例工程展示了XRadio SDK中进行音频录制的代码实现方法。
>
> 本工程中提供以下录音方式的示例：
> 1. 录制amr音频到sd卡
> 2. 录制pcm音频到sd卡
> 3. 以自定义保存地址的方式录制amr音频
> 4. 使用音频驱动录制pcm音频到sd卡

---

## 适用平台

> 本工程适用以下芯片类型：
>
> 1. XR872系列芯片：XR872AT、XR872ET

> 本工程适用以下评估板类型：
> 1. 底板：XR872MD_EVB
> 2. 模组：XR872AT_MD01

> 本工程在基于XR872ET的“XR872MD_EVB+XR872AT_MD01”评估板上测试通过。
> 若需要在其他适用芯片和评估板上运行本工程，请根据快速指南《XRadio_Quick_Start_Guide-CN》的提示进行相关配置修改。

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

## 工程配置

> localconfig.mk：
> * __CONFIG_XPLAYER: 必选项，配置使用音频功能
>
> Makefile：
> * PRJ_BOARD：必选项，选择板子的板级配置路径
>
> board_config.h
> * N/A
>
> board_config.c
> * N/A
>
> prj_config.h
> * PRJCONF_INTERNAL_SOUNDCARD_EN: 必选项，配置使用内置声卡
> * PRJCONF_MMC_EN:可选项，配置使用sd卡

## 模块依赖

> 必选项
> 1. libcedarx.a： 音频核心模块
> 2. libreverb.a： 音频混响核心模块

> 可选项
>
> 1. libamren.a： 录制amr音频需要的编码库

> 音频的数据流、编码格式可根据需求选择，选择说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki/wiki/dev-about-cedarx-encoder-config

---

## 工程说明

> 本工程的功能为录制amr音频到sd卡、录制pcm音频到sd卡、录制amr音频到自定义地址。

### 操作说明

> 1. 在sd/tf卡上创建一个“record”的文件夹
> 2. 编译工程，烧录镜像，启动即可
> 3. 系统启动后，即会自动进行录音

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki


### 代码结构
```
#本工程
.
├── gcc
│   ├── localconfig.mk          # 本工程的配置选项，主要用于覆盖默认全局配置
│   └── Makefile                # 本工程的编译规则，如ld文件、image.cfg、board_config.h等文件指定，可覆盖默认配置
├── main.c                      # 本工程的入口，完成平台初始化和音频录制
├── ExampleCustomerWriter.c     # 本工程的自定义保存地址实现方式
├── ExampleCustomerWriter.h
├── prj_config.h                # 本工程的配置选项，主要用于功能的选择。
└── readme.md                   # 本工程的说明文档

#本程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xradio_audio           #本工程在Makefile中指定使用xradio_audio的板级配置
                ├── board_config.h     #本工程的板级配置，
                └── board_config.c     #本工程的板级pin mux的配置。
```
### 代码流程

> 1. main()入口： 平台初始化，调用录音函数。
> 2. cedarx_record()函数：
> A）完成录音准备工作，如sd卡初始化
> B）完成录音器创建，即recorder_create()
> C）录制amr音频，保存到sd卡
> D）录制pcm音频，保存到sd卡
> E）使用自定义的音频数据保存方式，录制amr音频
> 2. audio_driver_record()函数：
> A）创建待保存的文件
> B）打开音频驱动接口
> C）循环调用snd_pcm_read和f_write函数，将录到的pcm音频保存到相应的文件
---


## 常见问题

> 问：使用cedarx无法录制两声道音频

答：暂不支持两声道音频录制。请直接使用音频驱动进行录音

> 问：使用cedarx无法录制高于32000采样率的pcm音频

答：暂不支持较高采样率的pcm音频录制。请直接使用音频驱动进行录音

## 参考文档

> 文档资源

无

> WiKi资源

1. https://github.com/XradioTech/xradiotech-wiki/wiki/dev-about-cedarx