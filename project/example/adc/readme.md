# ADC示例工程

> ADC示例工程展示了电源电压测量的使用方法。
>
> 本工程中提供以下模块接口使用的示例：
>
> 1. ADC模块的VBAT通道测量电源电压

---

## 适用平台

> 本工程适用以下芯片类型：
> 1. XR808系列芯片：XR808CT、XR808ST
> 2. XR872系列芯片：XR872AT、XR872ET

> 本工程适用以下评估板类型：
> 1. 底板：XR808MD_EVB_IO、XR872MD_IO、XR872MD_EVB
> 2. 模组：XR808CT_MD01、XR808CT_MD02、XR808ST_MD01、XR872AT_MD01

> 本工程在基于XR872AT的“XR872AT_VER_V1_0”板上测试通过。
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
> board_config.h
>
> - N/A
>
> board_config.c
>
> - 根据实际电路配置对应ADC通道PIN脚(通道8不需要配置)
>
>   ```
>   static const GPIO_PinMuxParam g_pinmux_adc[] = {
>   	{ GPIO_PORT_A, GPIO_PIN_10,  { GPIOA_P10_F2_ADC_CH0,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
>   	{ GPIO_PORT_A, GPIO_PIN_11,  { GPIOA_P11_F2_ADC_CH1,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
>   ...,
>   };
>   ```
>
> prj_config.h
>
> - N/A

## 模块依赖

> N/A

## 工程说明

> 本工程对ADC模块的通道测量电源电压的使用进行介绍。
>
> 宏ADC_CHL表示测试的ADC通道，工程默认采用的ADC通道是VBAT，该通道最高可测7.5V的电压，也可选择其他普通通道，即ADC的0~6通道（普通通道最高可测2.5V电压）。
>
> 宏ADC_BURST_MODE表示ADC的工作模式选择，工程默认为0，即工作模式选择continuous conversion 。配置为1时，工作模式采用burst conversion 。如果选择的是ADC通道是VBAT，ADC_BURST_MODE必须配置为0。
>
> 关于ADC值与电压的转换关系如下：
>
> ```
> voltage = adc_value * 2500 * ratio / 4096, 单位为mv
> ratio取值如下：
> 	1，当检测的ADC通道是chl0~chl6
>   	3，当检测的ADC通道是chl8
> ```
>
> 注：上述的两个宏均定义在工程的main.c文件中。

### 操作说明：

> 1. 连接XR872AT_VER_V1_0板，编译工程，烧录镜像，启动。
> 3. 系统启动后，给ADC通道管脚供电压，可以测量出所供电压的大小。
> 3. 在串口软件上可以看到测量的电压值信息。

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
├── main.c                      # 本工程的入口，进行ADC的电压测量示例说明
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

> 1. main()入口： 执行ADC测量电压的操作示例。
> 

> 更详细的开发指南请参考《XRADIO_ADC_Developer_Guide-CN.doc》

---

## 常见问题

> - 电压值计算误差范围：-50~50mv

## 参考文档

> 文档资源

1. 《XRADIO_ADC_Developer_Guide-CN.doc》
