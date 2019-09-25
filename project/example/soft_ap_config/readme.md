# soft ap config 示例工程

> soft ap config示例工程展示了XRadio SDK在AP模式下，通过创建web server来获取ssid和psk，达到配网的目的。
>
> 本工程中提供以下示例：
> 1. soft ap config代码流程实现示例。
> 2. soft ap config配网接口示例。

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
> * __CONFIG_WLAN_STA：必选项，配置支持station模式
> * __CONFIG_WLAN_AP：必选项，配置支持ap模式
>
> Makefile：
> * N/A
>
> board_config.h
> * N/A
>
> board_config.c
> * N/A
>
> prj_config.h
> * PRJCONF_NET_EN: 必选项，配置使用网络功能
> * PRJCONF_MAIN_THREAD_STACK_SIZE：可选项，修改main线程的栈大小，若出现main线程栈溢出，则可增加该选项大小
> * PRJCONF_CONSOLE_EN：可选项，配置使用控制台功能

## 模块依赖

> 必选项
> 1. liblwip.a： TCP/IP协议栈需要依赖的库
> 2. wlan模块： wlan需要依赖的库

---

## 工程说明

> 本工程演示的是通过创建web server的方式，用手机或PC的浏览器访问web  server，让芯片获取到要连接的wifi名和wifi密码，从而达到配网的目的。

### 操作说明

> 1. 编译工程，烧录镜像，启动即可
> 2. 打开串口调试工具，并连接串口
> 3. 系统启动后，会自动创建一个AP，名称叫“XRADIO_SOFT_AP_CONFIG_TEST”，密码为空
> 4. 用手机或PC连接该AP
> 5. 成功连接到AP后，用浏览器访问“192.168.51.1”
> 6. 弹出登陆界面，用户名为“admin”，密码为“admin”
> 7. 正常登陆并进入配网页面后，在SSID框输入要配网的wifi名称，在Key框中输入要配网的wifi密码，点击save按钮

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

### 控制命令

> N/A


### 代码结构
```
#本工程
.
├── gcc
│   ├── localconfig.mk   # 本工程的配置选项，主要用于覆盖默认全局配置
│   └── Makefile         # 本工程的编译规则，如ld文件、image.cfg、board_config.h等文件指定，可覆盖默认配置
├── main.c               # 本工程的入口，工程示例
├── prj_config.h         # 本工程的配置选项，主要用于功能的选择。
├── soft_ap_config.c     # 本工程实现soft ap config示例的c代码
├── soft_ap_config.h     # 本工程实现soft ap config示例的头文件
├── command.c            # 本工程的测试命令，暂无测试命令
└── readme.md            # 本工程的说明文档

#本程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xradio_audio           #本工程在Makefile中指定使用xradio_audio的板级配置
                ├── board_config.h     #本工程的板级配置
                └── board_config.c     #本工程的板级pin mux的配置
```
### 代码流程

> 1. 创建一个observer，并添加到sys_ctrl中，用于监控网络连接状态
> 2. 设置芯片为AP模式
> 3. 设置soft ap config的回调函数
> 4. 等待网络状态为NET_CTRL_MSG_NETWORK_UP，并启动soft ap config
> 5. 等待配网
>

## 常见问题

   > 问：在浏览器中点击save按钮后，芯片并未连接设置好的AP

   答：本工程只是演示soft ap config获取配网的wifi名和密码，连接AP不属于soft ap config的范畴，用户在获取到wifi名和密码时，可自行将芯片设置为STA模式并连接AP。

## 参考文档

   > 无