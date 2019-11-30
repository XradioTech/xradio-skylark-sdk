# websocket示例工程

> websocket示例工程展示了XRadio SDK中nopoll模块作为客户端发送数据到websocket服务器，并从服务器接收数据的示例。
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

> 本工程启动后，通过配网命令连接AP后，会依次自动连接“ws://123.207.167.163:9010/ajaxchattest”和“wss://echo.websocket.org”，连接成功后发送数据到服务器，并接收服务器返回的数据。以上两个链接都是用于websocket测试的，向服务器发送数据后，服务器就会返回同样的数据。

### 操作说明

> 1. 编译工程，烧录镜像，启动
> 2. 往串口输入配网命令进行网络连接
> 3. 成功联网后，等待自动循环连接服务器、发送数据、接收数据、关闭连接。

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
> 2. nopoll_client_demo_test()函数流程：
>   A）初始化nopoll
>
>   B）连接服务器，如果是wss连接，则需要设置CA证书（不设置的话，就需要设置ssl验证模式为nopoll_false，可调用nopoll_conn_opts_ssl_peer_verify函数）
>
>   C）连接成功后发送数据
>
>   D）接收服务器返回的数据
>

---

## 常见问题

> 问：有时会连接服务器失败

   答：能否连接成功、发送、接收数据，取决于网络状态，服务器状态，由于示例代码中的链接都是外网链接，容易造成链接失败，多尝试几次即可。

## 参考文档

无