# audio play and record 示例工程

> audio play and record示例工程展示了XRadio SDK中进行同时录播的代码实现方法。
>
> 本工程中提供以下播放方式的示例：
> 1. 使用cedarx播放音频文件
> 2. 使用音频驱动进行录音

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
> * __CONFIG_XPLAYER: 必选项，配置使用音频播放功能
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
> * PRJCONF_NET_EN: 可选项，配置使用网络功能
> * PRJCONF_MMC_EN:可选项，配置使用sd卡

## 模块依赖

> 必选项
> 1. libcedarx.a： 音频播放核心模块
> 2. libreverb.a： 音频混响核心模块

> 可选项
> 1. libmp3.a： 播放mp3歌曲需要的解码库
> 2. libamr.a： 播放amr歌曲需要的解码库
> 3. libaac.a： 播放aac/m4a歌曲需要的解码库
> 4. libwav.a： 播放wav歌曲需要的解码库
> 5. liblwip.a： 播放网络歌曲需要依赖的库
> 6. libmbedtls.a： 播放https歌曲需要依赖的库
> 7. wlan模块： 播放网络歌曲需要依赖的库

> 音频的数据流、解码格式可根据需求选择，选择说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki/wiki/dev-about-cedarx-decoder-config

---

## 工程说明

> 本工程的实现为循环进行音频播放、录音。

### 操作说明

> 1. 在sd/tf卡上创建一个“music”的文件夹，并在里面添加音频文件“1.mp3”
> 2. 在sd/tf卡上创建一个“record”的文件夹
> 3. 系统启动后，即会自动播放“1.mp3”，以及录音保存到“record/1.pcm”文件

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
├── main.c                      # 本工程的入口，启动播放和启动录音
├── audio_play.c                # 本工程的播放示例代码实现
├── audio_play.h
├── audio_record.c              # 本工程的录音示例代码实现
├── audio_record.h
├── record_output.c             # 录音数据的处理
├── record_output.h
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

> 1. main()入口： 调用check_sample_rate()检查录音采样率和播放采样率是否合理，调用audio_play_start()创建播放线程，调用audio_record_start创建录音线程。
> 2. 同时录播时，播放的采样率和录音的采样率有一定的关联。两者必须都属于[8000, 12000, 16000, 24000, 32000, 48000]或都属于[11025, 22050, 44100]。示例这里播放采用48k采样率，录音采样16k采样率，表示播放的音频会重采样为48k再进行播放，录音就是直接使用16k进行录音
> 3. 播放线程函数入口：play_task()
> 4. play_task()函数流程：
>   A）完成音频播放准备工作，如sd卡初始化
>   B）完成播放器创建，即player_create()
>   C）设置播放器输出的采样率为48khz
>   D）循环播放音频“1.mp3”
> 5. 录音线程函数入口：record_task()
> 6. record_task()函数流程：
>   A）初始化输出模块
>   B）启动录音
>   C）循环获取录音数据，并将录音数据输出到输出模块

---



## 常见问题

> 问：录音录取4个声道时，有些声道没有声音？

答：请确认硬件连接是否正确，板级配置是否正确

> 问：出现“Rx overrun”等打印？

答：这是cpu资源不足，导致播放或录音丢帧。在该example里，主要是因为写sd卡耗时较多，导致丢帧

## 参考文档

> 文档资源

1. 无

> WiKi资源

1. 无