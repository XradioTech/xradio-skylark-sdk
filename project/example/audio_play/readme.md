# audio play 示例工程

> audio play 示例工程展示了XRadio SDK中进行音频播放的代码实现方法。
>
> 本工程中提供以下播放方式的示例：
> 1. 播放本地SD卡的音频文件
> 2. 播放存于Flash的音频文件
> 3. 以数据流的形式播放本地SD卡的音频文件

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

> 本工程的实现为循环进行file、flash和fifo三种数据来源的音频播放。

### 操作说明

> 1. 在sd/tf卡上创建一个“music”的文件夹，并在里面添加音频文件“1.mp3”
> 2. 编译工程，烧录镜像，启动即可
> 3. 系统启动后，即会自动播放“1.mp3”，以及该工程自带的一个flash音频“image/xr872/1.amr”

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
├── image
│   └── xr872
│       ├── 1.amr               # 本工程用于flash播放示例的amr音频文件
│       └── image.cfg           # 本工程的镜像布局配置
├── main.c                      # 本工程的入口，完成平台初始化和播放器线程初始化
├── audio_play.c                # 本工程的示例代码实现
├── audiofifo.c                 # 本工程对音频数据流播放的调用接口封装
├── audiofifo.h
├── kfifoqueue.c                # 本工程音频数据流播放的队列实现
├── kfifoqueue.h
├── kfifo.c                     # 本工程使用kfifo实现队列
├── kfifo.h
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
#音频播放使用到的相关文件
.
└── project
    └── common
        └── apps
            ├── player_app.h     #对cedarx接口的调用接口封装
            └── player_app.c     #对cedarx接口的调用接口封装实现
```
### 代码流程

> 1. main()入口： 调用audio_play_start()创建播放线程。
> 2. 播放线程函数入口：play_task()
> 3. play_task()函数流程：
>   A）完成音频播放准备工作，如sd卡初始化
>   B）完成播放器创建，即player_create()
>   C）循环调用play_file_music()、play_flash_music()、play_fifo_music()
>

> 更详细的开发指南请参考《CedarX_Developer_Guide-CN.doc》

---

## 性能资源

> 同一歌曲，其播放地址不同时，需要占用的cpu资源差别不大，其中一个测试数据如下：

| 音频地址  | 音频类型 | cpu占用率 | 备注说明 |
| --------- | -------- | --------- | -------- |
| 本地file  | mp3      | 4%        |-         |
| 本地flash | mp3      | 4%        |-         |
| 网络http  | mp3      | 7%        |-         |

> 不同类型、不同质量的音频，播放时需要占用的cpu资源差别较大，具体数据如下：

| 音频地址 | 音频类型 | cpu占用率 | 备注说明 |
| -------- | -------- | --------- | -------- |
| 本地file | mp3      | 4%~10%    |-         |
| 本地file | amr      | 4%        |-         |
| 本地file | wav      | 1%        |-         |
| 本地file | aac/m4a  | 6%~20%    |-         |

> 不同类型的音频，播放时需要的内存资源差别较大，具体数据如下：

| 音频地址 | 音频类型 | 内存占用 | 备注说明 |
| -------- | -------- | -------- | -------- |
| 本地file | aac      | 128k     |-         |
| 本地file | mp3      | 65k      |-         |
| 本地file | mp2      | 67k      |-         |
| 本地file | amr(nb)  | 37k      |-         |
| 本地file | m4a      | 132k     |-         |
| 网络http | m4a      | 142k     |-         |
| 网络http | mp3      | 75k      |-         |

## 常见问题

> 问：无法播放m4a，wav或m3u8歌曲

答：1.首先确认函数platform_cedarx_init()里是否已经选择支持相应的音频类型；

​        2.如果是m3u8歌曲，请下载m3u8歌曲文件下来，确认一下里面的格式是否正确，以及里面的子url音频是否支持

> 问：无法播放某个http或https歌曲

答：1.首先确认函数platform_cedarx_init()里是否已经选择支持http/tcp/https/ssl音频来源；

​        2.确认该url是否有效，可以在电脑上下载该url的音频下来，放到sd卡进行播放，如果存放到sd卡也无法播放，请确认该音频的类型是否是支持播放的音频类型；

​        3.如果是https url，请将include/net/mbedtls-2.16.0/mbedlts/configs/config-xr-mini-cliserv.h或include/net/mbedtls-2.2.0/mbedlts/configs/config-xr-mini-cliserv.h里的MBEDTLS_SSL_MAX_CONTENT_LEN增大到16k

## 音频说明
> platform_cedarx_init()用于选择支持的音频来源/音频类型，其相关函数的说明，请在以下地址获取
> https://github.com/XradioTech/xradiotech-wiki/wiki/dev-about-cedarx-decoder-config

```
__weak void platform_cedarx_init(void)
{
	/* for media player */
	CedarxStreamListInit();
#if PRJCONF_NET_EN
	CedarxStreamRegisterHttps();
	CedarxStreamRegisterSsl();
	CedarxThreadStackSizeSet(DEMUX_THREAD, 8 * 1024);
	CedarxStreamRegisterHttp();
	CedarxStreamRegisterTcp();
#endif
	CedarxStreamRegisterFlash();
	CedarxStreamRegisterFile();
	CedarxStreamRegisterFifo();
	CedarxStreamRegisterCustomer();

	CedarxParserListInit();
	CedarxParserRegisterM3U();
	CedarxParserRegisterM4A();
	CedarxParserRegisterAAC();
	CedarxParserRegisterAMR();
	CedarxParserRegisterMP3();
	CedarxParserRegisterWAV();

	CedarxDecoderListInit();
	CedarxDecoderRegisterAAC();
	CedarxDecoderRegisterAMR();
	CedarxDecoderRegisterMP3();
	CedarxDecoderRegisterWAV();

	SoundStreamListInit();
	SoundStreamRegisterCard();
	SoundStreamRegisterReverb();

	/* for media recorder */
	CedarxWriterListInit();
	CedarxWriterRegisterFile();
	CedarxWriterRegisterCallback();
	CedarxWriterRegisterCustomer();

	CedarxMuxerListInit();
	CedarxMuxerRegisterAmr();
	CedarxMuxerRegisterPcm();

	CedarxEncoderListInit();
	CedarxEncoderRegisterAmr();
	CedarxEncoderRegisterPcm();
}
```



## 参考文档

> 文档资源

1. 《CedarX_Developer_Guide-CN.doc》

> WiKi资源

1. https://github.com/XradioTech/xradiotech-wiki/wiki/dev-about-cedarx