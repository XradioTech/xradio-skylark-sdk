# audio demo 演示工程

> audio_demo演示工程的功能为循环播放sd/tf卡music目录的音频资源
>

---

## 适用平台

> 本工程适用以下芯片类型：
>
> 1. XR872系列芯片：XR872AT、XR872ET

> 本工程适用以下评估板类型：
> 1. 底板：XR872_EVB_AI
> 2. 模组：XR872AT_MD01

> 本工程在基于"XR872AT_MD01"的“XR872_EVB_AI”板上测试通过。
> 若需要在其他适用芯片和评估板上运行本工程，请根据快速指南《XRadio_Quick_Start_Guide-CN》的提示进行相关配置修改。

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://docs.xradiotech.com

## 工程配置

> localconfig.mk：
> * __CONFIG_XPLAYER：必选项，配置使用音频播放功能
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
> * PRJCONF_INTERNAL_SOUNDCARD_EN：必选项，配置使用内置声卡
> * PRJCONF_NET_EN：可选项，配置使用网络功能
> * PRJCONF_MMC_EN：必选项，配置使用sd卡

## 模块依赖

> 必选项
> 1. libcedarx.a：音频播放核心模块
> 2. libreverb.a：音频混响核心模块

> 可选项
> 1. libmp3.a：播放mp3歌曲需要的解码库
> 2. libamr.a：播放amr歌曲需要的解码库
> 3. libaac.a：播放aac/m4a歌曲需要的解码库
> 4. libwav.a：播放wav歌曲需要的解码库
> 5. liblwip.a：播放网络歌曲需要依赖的库
> 6. libmbedtls.a：播放https歌曲需要依赖的库
> 7. wlan模块：播放网络歌曲需要依赖的库
> 8. fatfs、mmc模块：播放sd/tf卡歌曲需要的模块

> 音频的数据流、解码格式可根据需求选择，选择说明可在以下地址获取：
> https://docs.xradiotech.com

---

## 工程说明

> 本工程的实现为循环播放sd/tf卡里music目录的音频。

### 操作说明

> 1. 在sd/tf卡上创建一个“music”的文件夹，并在里面添加一些音频资源
> 3. 系统启动后，即会自动播放该目录下的音频资源

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://docs.xradiotech.com

### 控制命令

> N/A

### 代码结构
```
#本工程
.
├── gcc
│   ├── localconfig.mk          # 本工程的配置选项，主要用于覆盖默认全局配置
│   └── Makefile                # 本工程的编译规则，如ld文件、image.cfg、board_config.h等文件指定，可覆盖默认配置
├── main.c                      # 本工程的入口，启动播放
├── audio_play.c                # 本工程的播放代码实现
├── audio_play.h
├── audio_buttons.c             # 本工程的按键代码实现
├── audio_buttons.h
├── play_list.c                 # 本工程的播放列表处理代码
├── play_list.h
├── command.c                   # 本工程的控制台命令入口
├── command.h
├── prj_config.h                # 本工程的配置选项，主要用于功能的选择。
└── readme.md                   # 本工程的说明文档

#本程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xr872_evb_ai           #本工程在Makefile中指定使用xr872_evb_ai的板级配置
                ├── board_config.h     #本工程的板级配置，
                └── board_config.c     #本工程的板级pin mux的配置。
```
### 代码流程

> 1. main()入口：调用player_task_init()创建播放线程，初始化按键。
> 3. 播放线程函数入口：play_task()
> 3. play_task()函数流程：
>   A）完成音频播放准备工作，如sd卡初始化，播放列表初始化，创建播放器
>    B）设置播放音量
>    C）循环语句里：
>          1）检测是否有按键操作，以及判断歌曲是否播放结束
>          2）如果有按键操作，则执行相应的按键操作；如果播放结束，则启动播放下一首歌
---



## 常见问题

> 问：无法播放m4a，wav或m3u8歌曲

答：1.首先确认函数platform_cedarx_init()里是否已经选择支持相应的音频类型；

​        2.如果是m3u8歌曲，请下载m3u8歌曲文件下来，确认一下里面的格式是否正确，以及里面的子url音频是否支持

> 问：无法播放某个http或https歌曲

答：1.首先确认函数platform_cedarx_init()里是否已经选择支持http/tcp/https/ssl音频来源；

​        2.确认该url是否有效，可以在电脑上下载该url的音频下来，放到sd卡进行播放，如果存放到sd卡也无法播放，请确认该音频的类型是否是支持播放的音频类型；

​        3.如果是https url，请将include/net/mbedtls-2.16.0/mbedlts/configs/config-xr-mini-cliserv.h或include/net/mbedtls-2.2.0/mbedlts/configs/config-xr-mini-cliserv.h里的MBEDTLS_SSL_MAX_CONTENT_LEN增大到16k

## 参考文档

> 文档资源

1. 《XRADIO_Audio_Developer_Guide-CN.doc》

> WiKi资源

1. https://docs.xradiotech.com 
