# efpg 示例工程

> efpg 示例工程展示了XRadio SDK中进行efuse读写的代码实现方法。
>

---

## 适用平台

> 本工程适用以下芯片类型：
>
> 1. XR872系列芯片：XR872AT、XR872ET
> 2. XR808系列芯片：XR808ST、XR808CT

> 本工程适用以下评估板类型：
> 1. 底板：XR872MD_EVB、XR872MD_IO、XR808MD_EVB_IO
> 2. 模组：XR872AT_MD01、XR808ST_MD01、XR808CT_MD02、XR808CT_MD01

> 本工程在基于XR872ET的“XR872MD_EVB+XR872AT_MD01”评估板上测试通过。
> 若需要在其他适用芯片和评估板上运行本工程，请根据快速指南《XRadio_Quick_Start_Guide-CN》的提示进行相关配置修改。

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

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

> 无
>

---

## 工程说明

> 本工程会读取efuse里的各个区域的字段信息。如果使能宏EFPG_WRITE_ENABLE，则会往efuse的用户区域填充数据

### 操作说明

> 1. 编译工程，烧录镜像，启动即可

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

### 控制命令

> 无

### 代码结构
```
#本工程
.
├── gcc
│   ├── localconfig.mk          # 本工程的配置选项，主要用于覆盖默认全局配置
│   └── Makefile                # 本工程的编译规则，如ld文件、image.cfg、board_config.h等文件指定，可覆盖默认配置
├── main.c                      # 本工程的代码实现
├── prj_config.h                # 本工程的配置选项，主要用于功能的选择。
└── readme.md                   # 本工程的说明文档

#本程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xradio_evb           #本工程在Makefile中指定使用xradio_evb的板级配置
                ├── board_config.h     #本工程的板级配置，
                └── board_config.c     #本工程的板级pin mux的配置。
```
### 代码流程

> 1. main()入口： 调用efpg_read_example()读取efuse信息；调用efpg_write_example往efuse用户区域填充数据。
> 2. efpg_read_example()函数流程：
>   A）读取hosc信息
>   B）读取boot信息
>   C）读取mac地址信息
>   D）读取chipid信息
>   E）读取用户区域数据
> 3. efpg_write_example()函数流程：
>   A）往用户区域填充数据
>   B）读取用户区域数据
>   C）打印填充的数据，和读取到的数据
>

---

## 常见问题

无

## 参考文档

无