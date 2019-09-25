# wlan low power 示例工程

> wlan low power 示例工程展示了XRadio SDK中进入wlan低功耗模式的代码实现方法。
>
> 本工程中提供以下场景的wlan低功耗的示例：
> 1. 连接AP时休眠场景

---

## 适用平台

> 本工程适用以下芯片类型：
>
> 1. XR808系列芯片：XR808ST

> 本工程适用以下评估板类型：
> 1. 底板：XR808MD_EVB_IO
> 2. 模组：XR808ST_MD01

> 本工程在基于XR808ST的“XR808MD_EVB_IO+XR808ST_MD01”评估板上测试通过。
> 若需要在其他评估板上运行本工程，请根据快速指南《XRADIO_Quick_Start_Guide-CN》的提示和硬件电路设计进行相关配置修改。

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

## 工程配置

> localconfig.mk：
> * __CONFIG_WLAN_STA: 必选项，配置使用WLAN STA功能
>
> Makefile：
> * PRJ_BOARD: 必选项，需要使用自定义的板级配置
>
> board_config.h
> * BOARD_LOSC_EXTERNAL： 必选项，配置为1以使用外部32kHz低频晶振
> * BOARD_CPU_CLK_FACTOR： 必选项，设置为PRCM_SYS_CLK_FACTOR_160M以使用较低CPU频率
>
> board_config.c
> * N/A
>
> prj_config.h
> * PRJCONF_PM_EN: 必选项，配置PM模式
> * PRJCONF_NET_EN: 必选项，配置使用网络功能

## 模块依赖

> 必选项
> 1. wlan模块： 完成STA相关功能
> 2. pm模块： 完成休眠相关功能
> 3. gpio模块：完成外部DC-DC模块控制

> 可选项：N/A

> WLAN低功耗相关信息可参考以下文档：
> 《XRADIO_Wlan_Low_Power_Developer_Guide-CN.doc》

---

## 工程说明

> 本工程实现为进行低功耗的配置检查或设置，统计AP的Beacon延迟，然后连接AP并休眠120秒，最后唤醒等待用户输入命令。

### 操作说明：

> 1. 使用直流分析仪等测量功耗设备与XR808ST评估板连接，测试点V-BAT管脚，输出电压为3.6v。
> 2. 准备AP设置，工作在2.4GHz频段，设置ssid为“wlan_low_power”，password为“12345678”
> 3. 编译工程，烧录镜像，启动即可
> 4. 系统启动后，工程会进行配置检查或设置，配置为低功耗模式，然后进行AP的Beacon延迟检查，最后连接AP并休眠约120s
> 5. 在这120s内，用户可以通过直接分析仪等测量功耗设备获知此时XR808ST评估板在连接AP休眠时的功耗。
> 6. 在120s后，系统唤醒，此时用户可以使用提示的命令再次进入休眠和测试。

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

### 控制命令

> 1. PM操作
> 先设置硬件Timer，用于延迟若干秒后唤醒设置，然后设置PM模块进入standby模式

```
$ pm wk_timer 120
$ pm standby
```

> 2. 关联AP
> 设置关联AP的ssid和password

```
$ net sta config your_ssid your_password
```

### 代码结构
```
#本工程
.
├── board                 # 本工程在makefile中指定使用板级配置位置
│   ├── board_config.c    # 本工程的板级配置
│   └── board_config.h    # 本工程的板级pin mux的配置
├── command.c             # 本工程的控制台命令入口
├── command.h
├── gcc
│   ├── localconfig.mk    # 本工程的配置选项，可覆盖全局默认配置
│   └── Makefile          # 本工程的编译规则，可指定src、lib、ld、image.cfg、board_config等文件
├── main.c                # 本工程的入口，完成平台初始化和示例指引
├── prj_config.h          # 本工程的配置选项，主要用于功能的选择。
└── readme.md             # 本工程的说明文档
```
### 代码流程

> 1. main()入口：
> A）完成平台初始化platform_init()
> B）开始连接AP时休眠的操作示例
> C）示例完成，等待用户输出命令

> 更详细的开发指南请参考《XRADIO_Wlan_Low_Power_Developer_Guide-CN.doc》

---

## 常见问题

> 问：DC-DC模式下功耗与预期不符

答：按如下步骤检查：

1.在示例完成，等待用户输入命令时，使用电压表确认VDD-ANA是否为1.8v。
如果不是1.8v，那么SY8088工作异常，需要进行SY8088电路完整性检查。
如果为1.8v，那么SY8088工作正常，此时可能为XR808ST工作异常，检查内部LDO模式是否设置输出为1.4v。

2.确认XR808ST模组上的SY8088的相关器件完整。
如果不完整，则是SY8088的外围电路可能未焊好，检查原理图和PCB图，修复电路。
如果完整，那么可能是SY8088的使能管脚PA23未设置正常，检查PA23的输出是否为3.3v，若不是3.3v，检查GPIO设置代码。

若以上设置均正常，请咨询FAE

## 参考文档

> 文档资源

1. 《XRADIO_Wlan_Low_Power_Developer_Guide-CN.doc》