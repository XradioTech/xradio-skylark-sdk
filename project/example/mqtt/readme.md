# mqtt示例工程

> mqtt示例工程展示了XRadio SDK中mqtt模块作为客户端向阿里云mqtt服务器订阅topic和发布消息到的示例。
>

---

## 适用平台

> 本工程适用以下芯片类型：
>
> 1. XR808系列芯片：XR808CT
> 2. XR872系列芯片：XR872AT、XR872ET

> 本工程适用以下评估板类型：
> 1. 底板：XR808_EVB_IO、XR872_EVB_IO、XR872_EVB_AI
> 2. 模组：XR808CT0_MD01、XR808CT0_MD02、XR872AT_MD01

> 本工程在基于"XR872AT_MD01"的“XR872_EVB_AI”板上测试通过。
> 若需要在其他适用芯片和评估板上运行本工程，请根据快速指南《XRadio_Quick_Start_Guide-CN》的提示进行相关配置修改。

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://docs.xradiotech.com

## 工程配置

> localconfig.mk：
> * N/A
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
>
> * PRJCONF_NET_EN：必选项，配置使能网络功能

## 模块依赖

> 必选项
>
> 1. wlan模块：网络连接需要依赖的库

---

## 工程说明

> 本工程启动后，通过配网命令连接AP后，会自动连接阿里云mqtt服务器 ，订阅topic，并每隔一定时间向该topic发布消息。
>
> 由于登陆阿里云需要账号，示例工程中的账号是免费共用的账号，多人使用时会经常出现无法连接或突然掉线的现象。工程中mqtt的client id、host、port、username、password、topic参数，都是经过转换的，请勿修改，具体的转换规则可参考阿里云mqtt文档：https://help.aliyun.com/document_detail/73742.html?spm=a2c4g.11186623.6.587.5d6d6eebYEIUrM。
>
> 本工程只是为了展示XRadio SDK中mqtt模块的使用方式，由于阿里云账号问题，可能运行结果并不理想，可自行搭建mqtt服务器，修改client id、host、port、username、password、topic参数，编译再运行本工程。
>
> 本工程采用的是MQTT-TCP连接方式，并非MQTT-WebSocket。

### 操作说明

> 1. 编译工程，烧录镜像，启动
> 2. 往串口输入配网命令进行网络连接
> 3. 成功联网后，等待自动链接阿里云并发布消息。

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://docs.xradiotech.com

### 控制命令

> 1. 网络连接
>
>    $ net sta config <ssid> <passphrase>
>
>    $ net sta enable

### 代码结构
```
#本工程
.
├── gcc
│   ├── localconfig.mk          # 本工程的配置选项，主要用于覆盖默认全局配置
│   └── Makefile                # 本工程的编译规则，如ld文件、image.cfg、等文件指定，可覆盖默认配置
├── main.c                      # 本工程的代码实现
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

> 1. main()入口：创建并注册网络状态监视器，网络一旦连接成功后，便会调用注册的回调函数net_cb
>
> 2. mqtt_demo_fun()函数流程：
>   A）初始化mqtt
>
>   B）连接mqtt服务器，如果是ssl连接，则需要设置CA证书，且使用TLSConnectNetwork接口连接tcp层
>
>   C）订阅topic
>
>   D）循环向topic发布消息，并调用MQTTYield接收服务消息。
>

---

## 常见问题

> 问：有时会连接服务器失败

   答：示例工程中使用的阿里云账号是免费共用账号，多人使用时会经常出现无法连接或突然掉线的现象，多尝试几次即可。也可自行搭建mqtt服务器，修改client id、host、port、username、password、topic参数，编译再运行本工程。

## 注意问题

1. mqtt模块所有的内存使用都需要外部申请，并通过指针的形式传递到mqtt模块，在mqtt反初始化时，mqtt模块不会释放内存，需要自行释放，否则容易造成内存泄漏。
2. MQTTYield函数是阻塞式的，在timeout前，会阻塞同优先级的其他线程，导致其他线程不能正常运行，可根据要求自行调整mqtt线程优先级、减小timeout时间、提高调用MQTTYield的频率（可用系统延时），保证在每次调用的时间间隔内有时间执行其他操作，从而做到尽量减少对线程的阻塞。 

## 参考文档

无