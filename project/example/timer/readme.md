# timer示例工程

> timer示例工程展示了timer模块接口使用方法。
>
> 本工程中提供以下模块接口使用的示例：
> 1. 开始 计时/定时 功能使用
> 2. 暂停 计时/定时 功能使用
> 3. 继续 计时/定时 功能使用
> 4. 停止 计时/定时 功能使用
---

## 适用平台

> 本工程适用以下芯片类型：
> 1. XR808系列芯片：XR872CT
> 2. XR872系列芯片：XR872AT、XR872ET

> 本工程适用以下评估板类型：
> 1. 底板：XR872_EVB_AI、XR872_EVB_IO、XR808_EVB_IO
> 2. 模组：XR872AT_MD01、XR808CT0_MD01、XR808CT0_MD02

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
> * N/A

## 模块依赖

> * N/A

---

## 工程说明

> 本工程通过串口UART0打印信息演示timer模块的开始、暂停、继续、停止 计时/定时功能的使用。

### 操作说明：

> 1. 使用串口线连接UART0接口
> 2. 编译工程，烧录镜像，复位启动
> 3. 系统启动后，可以通过串口软件看到示例的打印信息

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://docs.xradiotech.com

### 控制命令

> * N/A

### 代码结构
```
.
├── gcc
│   ├── localconfig.mk          # 本工程的配置规则，用于覆盖默认配置
│   └── Makefile                # 本工程的编译规则，可指定src、lib、ld、image.cfg、board_config等文件
├── image
│   └── xr872
│       └── image.cfg           # 本工程的镜像分区配置
├── main.c                      # 本工程的入口，完成timer初始化和演示timer示例
├── prj_config.h                # 本工程的配置规则
└── readme.md                   # 本工程的说明文档

#本程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xr872_evb_ai             #本工程在Makefile中指定使用xr872_evb_ai的板级配置
                ├── board_config.h     #本工程的板级配置，
                └── board_config.c     #本工程的板级pin mux的配置。
```
### 代码流程

> 1. main()入口：
> A）执行timer初始化操作
> B）执行timer_show()
> C）执行DeInit操作
> 2. timer_show()函数流程：
> A）开始计时10秒
> B）暂停计时10秒
> C）继续计时10秒
> C）结束计时

---

## 常见问题

> * N/A

## 参考文档

> * N/A
