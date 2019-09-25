

# Button示例工程

> Button示例工程展示了XRadio SDK中button模块的接口使用方法及流程。
>
> 本工程中提供以下按键示例：
>
> 1. AD按键

------

## 适用平台

> 本工程适用以下芯片类型：
>
> 1. XR872系列芯片：XR872AT、XR872ET

> 本工程适用以下评估板类型：
>
> 1. XR872ET_STORYTOY_PRO_V1_0

> 本工程在基于XR872ET的“XR872ET_STORYTOY_PRO_V1_0”评估板上测试通过。暂且不支持其他评估板，若需要支持其他评估板，则需要进行board_config.c适配，详细请参考《XR872_Button_Developer_Guide-CN.doc》。

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

## 工程配置

> localconfig.mk：
>
> - N/A
>
> Makefile：
>
> - PRJ_BOARD：必选项，选择板子的板级配置路径
>
> board_config.h：
>
> - N/A
>
> board_config.c：
>
> - g_ad_buttons：必选项，AD按键硬件配置数组。
> - g_gpio_buttons：必选项，DPIO按键硬件配置数组。

## 模块依赖

> 必选项
>
> 1. hal_adc.c：adc驱动
> 2. hal_gpio.c：gpio驱动
> 3. buttons.c：按键模块代码
> 4. buttons_low_level.c：按键底层驱动

> 可选项
>
> 无

## 工程说明

> 本工程代码需要配合`board_config.c`来使用，`board_config.c`中保存了按键的配置信息。本工程使用的`board_config.c`的路径为：`project/common/board/xradio_storybot/board_config.c`。本工程只支持`XR872ET_STORYTOY_PRO_V1_0`工程板，如果需要支持其他板子，则需要选择对应的`board_config.c`，可修改gcc目录下`localconfig.mk`中的`__PRJ_CONFIG_BOARD`。
>
> 由于`XR872ET_STORYTOY_PRO_V1_0`工程板不含GPIO按键，故本工程只支持AD按键。

### 操作说明

> 1. 编译工程，烧录镜像，重启即可
> 2. 打开串口调试工具，并连接串口
> 3. 系统启动后，按下板子上的按键，串口打印按下按键按下的信息

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

### 代码结构

```
#本工程
.
├── gcc
│   ├── localconfig.mk    # 本工程的配置选项，主要用于覆盖默认全局配置
│   └── Makefile          # 本工程的编译规则，如ld文件、image.cfg、board_config.h等文件指定，可覆盖默认配置
├── image
│   └── xr872
├── main.c                # 本工程的入口，示例代码
├── prj_config.h          # 本工程的配置选项，主要用于功能的选择。
└── readme.md             # 本工程的说明文档

#本程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xradio_storybot        #本工程在localconfig.mk中指定使用xradio_storybot的板级配置
                ├── board_config.h     #本工程的板级配置
                └── board_config.c     #本工程的板级pin mux的配置。
```

### 代码流程

> 1. 注册按键底层驱动接口，配置按键模块，初始化按键模块。
>
> 2. 创建各按键对象。
> 3. 设置各按键对象的回调函数。
> 4. 启动各按键对象。

> 更详细的开发指南请参考《XR872_Button_Developer_Guide-CN.doc》

## 常见问题

> 无
>

## 参考文档

> 文档资源
>
> 1. 《XR872_Button_Developer_Guide-CN.doc》