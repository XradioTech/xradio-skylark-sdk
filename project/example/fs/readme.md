# FATFS示例工程

> FATFS示例工程展示了FATFS模块常用接口的使用方法。
>
> 本工程中提供以下模块接口使用的示例：
>
> 1. 获取SD卡信息（容量）
> 2. 打印文件内容
> 3. 拷贝文件
> 4. 扫描文件
> 5. 删除目录
> 6. 拷贝目录

---

## 适用平台

> 本工程适用以下芯片类型：
>
> 1. XR872系列芯片：XR872AT、XR872ET

> 本工程适用以下评估板类型：
> 1. 底板：XR872MD_IO、XR872MD_EVB
> 2. 模组：XR872AT_MD01

> 本工程在基于XR872ET的“XR872ET_VER_DIG_V1_0”数字板上测试通过。
> 若需要在其他适用芯片和评估板上运行本工程，请根据快速指南《XRadio_Quick_Start_Guide-CN》的提示进行相关配置修改。

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

---

## 工程配置

> localconfig.mk：
>
> - N/A
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
> - PRJCONF_MMC_EN: 必选项，配置使用SD卡功能，需使能。
>
> - PRJCONF_CONSOLE_EN： 可选项，配置使用控制台功能，需要禁用。若使能需要在当前目录下添加command文件。

## 模块依赖

> N/A

## 工程说明

> 本工程对FATFS常用的功能的使用进行介绍，功能示例包括：获取SD卡容量信息、打印文件内容、拷贝文件、扫扫描目录下的文件、删除目录及拷贝目录，相应的功能示例的测试用宏来配置，宏保存在main.c文件中。
>
> 工程默认情况不演示“打印文件内容”的示例，若使用该示例，需要将宏FS_FILE_PRINT_TEST打开（即置1），同时使能FATFS的字符功能，即将_USE_STRFUNC的值修改为2（文件位置：ffconf.h）。

### 操作说明：

> 1. 在SD卡的根目录下创建文件print_file.txt、copy_file.txt、创建目录dir_src、dir_test，文件或目录的内容可以自主添加，但要保证文件/目录的绝对路径大小不能超过256个字节。然后将SD卡插入数字板的卡槽。
> 2. 编译工程，烧录镜像，启动即可
> 3. 系统启动后，可以看到测试的打印信息

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

### 代码结构
```
.
├── gcc
│   ├── localconfig.mk          # 本工程的配置规则，用于覆盖默认配置
│   └── Makefile                # 本工程的编译选项，可覆盖默认配置
├── image
│   └── xr872
│       └── image.cfg           # 本工程的镜像分区配置
├── main.c                      # 本工程的入口，进行FATFS常用功能的示例说明
├── prj_config.h                # 本工程的配置规则
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

> 1. main()入口： 执行FATFS常用功能点的操作示例。
> 

> 更详细的开发指南请参考《XRADIO_FatFs_Developer_Guide-CN.doc》

---

## 常见问题

> N/A

## 参考文档

> 文档资源

1. 《XRADIO_FatFs_Developer_Guide-CN.docc》
