# audio reverb 示例工程

> audio reverb 示例工程展示了XRadio SDK中进行混响播放的代码实现方法。
>

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
> 8. fatfs、mmc模块：播放sd/tf卡歌曲需要的模块

> 音频的数据流、解码格式可根据需求选择，选择说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki/wiki/dev-about-cedarx-decoder-config

---

## 工程说明

> 本工程的实现为播放背景音乐，以及根据话筒的插入与否决定是否使能混响功能。

### 操作说明

> 1. 在sd/tf卡上创建一个“music”的文件夹，并在里面添加音频文件“1.mp3”
> 2. 硬件上，以一个耳机座替换mic。即当插入话筒时，外部声音的来源源于该话筒
> 3. 以GPIOA8作为话筒插入/拔出检测管脚
> 4. 编译工程，烧录镜像，启动即可
> 5. 系统启动后，即会自动播放“1.mp3”。当插入话筒时，系统会启动混响模式，此时喇叭播放声音为话筒声加音乐声。当拔出话筒时，系统退出混响模式，正常播放“1.mp3”

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
├── main.c                      # 本工程的入口，完成平台初始化，启动背景音乐播放，启动话筒检测模块
├── bgm.c                       # 本工程背景音乐播放的调用接口封装
├── bgm.h
├── detect.c                       # 本工程话筒插入/拔出检测模块
├── detect.h
├── karaok.c                       # 本工程karaok播放模式的调用接口封装
├── karaok.h
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

> 1. main()入口： 平台初始化，启动背景音乐播放，启动话筒插入/拔出检测模块。
> 2. 混响模块函数入口：karaok_task()
> 3. karaok_task()函数流程：
>   1）初始化录音模块
>   2）初始化混响模块
>   3）初始化背景音乐模块，这里主要是控制背景音乐播放的pcm数据不在传入音频驱动，而是传入混响模块的buffer
>   4）初始化mixer模块，该模块用于合并两声道数据为单声道数据
>   5）初始化输出模块，该模块会将pcm数据传入音频驱动
>   6）循环执行下述步骤
>            A）获取话筒录音的pcm数据
>            B）处理话筒pcm数据，对其进行混响处理，得到有混响效果的pcm数据
>            C）获取背景音乐pcm数据
>            D）混合背景音乐pcm数据和有混响效果的pcm数据
>            E）输出最终处理得到的pcm数据到喇叭
>


## 常见问题

> 问：插入话筒，但喇叭没有话筒的声音

答：1.首先确认程序是否检测到话筒插入。如没能检测到话筒已插入，请检查硬件电路的GPIO管脚与配置的GPIO管脚是否相符；如能检测到话筒已插入，请检查获取到的话筒pcm数据是否正常

## 参考文档

> 文档资源

1. 《CedarX_Developer_Guide-CN.doc》

> WiKi资源

1. https://github.com/XradioTech/xradiotech-wiki/wiki/dev-about-cedarx