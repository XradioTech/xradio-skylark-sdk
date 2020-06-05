# Secure boot 示例工程

> Secure boot 示例工程展示了固件安全启动的使用方法。
>
> 本工程中提供安全启动使用的示例：
>
> 1. 安全启动固件配置、烧写EFUSE secure boot区域

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

---

## 工程配置

> localconfig.mk：
>
> - __CONFIG_XIP：可选项，配置XIP的使能。
> - __CONFIG_SECURE_BOOT：必选项，配置固件安全启动的使能。使能后，会修改工程对应的image config文件，即向“boot.bin”和“app.bin”添加证书名字，并将“attr”属性修改为“0x5”。
> 
> Makefile：
>
> - N/A
>
> board_config.h
>
> - N/A
>
> board_config.c
>
> - N/A
>
> prj_config.h
>
> - N/A
>

## 模块依赖

> N/A

## 工程说明

> 实现安全启动的功能需要编译支持安全启动的Bootloader，步骤如下：
>
> 1）在“project\bootloader\gcc\localconfig.mk”中设置“export __CONFIG_SECURE_BOOT := y”。
>
> 2）在“project\bootloader\gcc\”目录下执行“make build”重新编译生产“boot.bin”。
>
> 编译工程时需要先修改工程对应的image config文件，向“boot.bin”和“app.bin”添加证书名字，并将“attr”属性修改为“0x5”。

### 操作说明：

> 1. 重新编译带安全启动功能的“boot.bin”。
>
> 2. 编译工程，烧录镜像，启动。
>
> 3. 将“cakey_pub_dcr_hash.txt”中的SHA256哈希值通过efuse_tool工具烧写到EFUSE Secure Boot区域。
>
>    ![img](file:///C:\Users\SHAOGE~1\AppData\Local\Temp\ksohtml\wpsA351.tmp.jpg)

> 4. 系统启动后能正常运行。
> 5. 重新烧写非安全启动的固件，若系统无法正常启动则说明secure boot机制运行正常。
>
> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://docs.xradiotech.com

### 代码结构
```
.
├── command.c          			# 本工程的控制台命令入口和命令定义
├── command.h
├── gcc
│   ├── localconfig.mk          # 本工程的配置规则，用于覆盖默认配置
│   └── Makefile                # 本工程的编译选项，可覆盖默认配置
├── image
│   └── xr872
│       └── image.cfg           # 本工程的镜像分区配置
├── main.c                      # 本工程的入口
├── prj_config.h                # 本工程的配置规则
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

> 1. main()入口：执行平台初始化。

>		更详细的开发指南请参考《XRADIO_Security_Boot_Developer_Guide-CN.doc》

---

## 常见问题

> N/A

## 参考文档

> 文档资源
>
1. 《XRADIO_Security_Boot_Developer_Guide-CN.doc》
