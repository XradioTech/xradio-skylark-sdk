# SmartConfig示例工程

> SmartConfig示例工程展示了XRadio SDK中SmartConfig配网接口的使用方法。
>
> 本工程中提供以下示例：
>
> 1. SmartConfig配网接口示例
> 2. XRadio SDK配网助手使用示例

------

## 适用平台

> 本工程适用以下芯片类型：
>
> 1. XR872系列芯片：XR872AT、XR872ET

> 本工程适用以下评估板类型：
>
> 1. 底板：XR872MD_EVB
> 2. 模组：XR872AT_MD01

> 本工程在基于XR872ET的“XR872MD_EVB+XR872AT_MD01”评估板上测试通过。
>
> 若需要在其他适用芯片和评估板上运行本工程，请根据快速指南《XRadio_Quick_Start_Guide-CN》的提示进行相关配置修改。

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

## 工程配置

> localconfig.mk：
>
> - N/A
>
> Makefile：
>
> - N/A
>
> board_config.h：
>
> - N/A
>
> board_config.c：
>
> - N/A
>
> prj_config.h：
>
> - N/A

## 模块依赖

> 必选项
>
> 1. libsmartlink.a：SmartConfig库
> 2. libsc_assistant.a：XRadio SDK配网助手库，该库可以提高配网的成功率。

> 可选项
>
> 无

## 工程说明

> 本工程主要目的是为了展示XRadio SDK中SmartConfig配网接口的使用方法及使用流程，SmartConfig配网接口中有较多阻塞接口，故在实际开发过程中，建议单独建立配网线程来运行SmartConfig配网接口。

### 操作说明

> 1. 编译工程，烧录镜像，启动即可
> 2. 打开串口调试工具，并连接串口
> 3. 系统启动后，会自动进入SmartConfig配网
> 4. 安装Xconfig.apk软件，并打开软件（该软件可咨询FAE获取）
> 5. 手机连接到某个用于SmartConfig配网测试的wifi
> 6. 在Xconfig.apk软件中，填入要配网的wifi名、wifi密码及AESKey，点击开始
> 7. 等待串口打印配网结果

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

### 代码结构

```
#本工程
.
├── gcc
│   ├── localconfig.mk     # 本工程的配置选项，主要用于覆盖默认全局配置
│   └── Makefile           # 本工程的编译规则，如ld文件、image.cfg、board_config.h等文件指定，可覆盖默认配置
├── main.c                 # 本工程的入口，工程示例
├── prj_config.h           # 本工程的配置选项，主要用于功能的选择。
└── readme.md              # 本工程的说明文档

#本程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xradio_evb             #本工程在localconfig.mk中指定使用xradio_evb的板级配置
                ├── board_config.h     #本工程的板级配置
                └── board_config.c     #本工程的板级pin mux的配置。
```

### 代码流程

> 1. 初始化配网助手
> 2. 启动SmartConfig配网（XRadio SDK封装的SmartConfig配网接口内部会使用配网助手）
> 3. 等待配网结果（阻塞等待）
> 4. 配网完成后，获取配网结果
> 5. 用配网结果连接wifi并发送connect ack信号（告诉手机设备已经连接上wifi了）

## 常见问题

> 问：SmartConfig连接不上或连接超时

答：1.检查aes_key是否设置正确；2.检查路由器是否是2.4G；3.检查路由器是否设置为支持20M或20M/40M；4.检查路由器是否支持转发组播包。

## 参考文档

> 无