# pm 示例工程

> pm 示例工程展示了XRadio SDK中进行休眠唤醒的代码实现方法。
>
> 本工程中提供以下模块功能使用的示例：
> 1. 进入sleep低功耗模式
> 2. 进入standby低功耗模式
> 3. 进入hibernation低功耗模式
> 4. 配置uart0中断作为唤醒源
> 5. 配置timer 10s作为唤醒源
> 6. 配置wlan作为唤醒源，检测是否连接AP
> 7. 配置wakeup io 5下降沿触发中断作为唤醒源
> 8. 配置wakeup io debounce为17个32K晶振cycle
> 9. 测试不同dtim下的最低联网功耗

---

## 适用平台

> 本工程适用以下芯片类型：
>
> 1. XR872系列芯片：XR872AT、XR872ET

> 本工程适用以下评估板类型：
> 1. 底板：XR872MD_EVB
> 2. 模组：XR872AT_MD01

> 本工程在基于XR872AT的“XR872MD_EVB+XR872AT_MD01”评估板上测试通过。
> 若需要在其他适用芯片和评估板上运行本工程，请根据快速指南《XRadio_Quick_Start_Guide-CN》的提示进行相关配置修改。

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

## 工程配置

> localconfig.mk：
> * N/A
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
> * PRJCONF_PM_EN: 必选项，配置使用pm功能
> * PRJCONF_NET_PM_EN: 必选项，配置使用网络pm功能
> * PRJCONF_NET_EN: 必选项，配置使用网络功能

## 模块依赖

> 必选项
> 1. libpm.a： 电源管理核心模块
> 2. libxrwireless.a： 网络驱动核心模块

> 可选项
> * N/A


## 工程说明

> 本工程通过串口UART0打印信息演示pm模块的sleep/standby/hibernation/wakeup功能的使用。

### 操作说明：

> 1. 使用串口线连接UART0接口
> 2. 编译工程，烧录镜像，复位启动
> 3. 系统启动后，可以通过串口软件看到示例的打印信息

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki


### 控制命令

> 1. 联网操作
> 进行网络唤醒时，必须完成联网操作，通过以下命令可以完成AP的连接。

```
$ net sta config ssid_example
$ net sta enable
```

### 代码结构
```
#本工程
.
├── command.c             # 本工程的控制台命令入口
├── command.h
├── gcc
│   ├── localconfig.mk    # 本工程的配置选项，可覆盖全局默认配置
│   └── Makefile          # 本工程的编译规则，可指定src、lib、ld、image.cfg、board_config等文件
├── main.c                # 本工程的入口，完成平台初始化和进入休眠唤醒测试
├── prj_config.h          # 本工程的配置选项，主要用于功能的选择。
└── readme.md             # 本工程的说明文档

#本程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xradio_evb             #本工程在localconfig.mk中指定使用xradio_evb的板级配置
                ├── board_config.h     #本工程的板级配置，
                └── board_config.c     #本工程的板级pin mux的配置。
```
### 代码流程


> 1. main()入口：
> A）平台相关初始化
> B）打印pm基本功能信息
> C）等待3秒

> 2. sleep流程：
> A）配置uart0作为唤醒中断
> B）配置timer 10s作为唤醒中断
> C）配置wakeup io 5作为唤醒中断，且配置wakeup io debounce为17个32K晶振cycle
> D）进入sleep低功耗模式
> E）被唤醒后CPU继续执行
> F）关闭uart0作为唤醒中断
> G）关闭timer作为唤醒中断
> H）关闭wakeup io作为唤醒中断

> 3. standby流程：
> A）通过cmd命令链接AP,例如：
     步骤1：net sta config ap_ssid [ap_psk]
     步骤2：net sta enable
> B）配置wlan检测是否连接AP
> C）配置timer 10s作为唤醒中断
> D）配置wakeup io 5作为唤醒中断，且配置wakeup io debounce为17个32K晶振cycle
> E）进入standby低功耗模式
> F）被唤醒后CPU继续执行
> G）关闭wlan检测AP功能
> H）关闭timer作为唤醒中断
> I）关闭wakeup io作为唤醒中断

> 4. hibernation流程：
> A）配置timer 10s作为唤醒中断
> B）配置wakeup io 5作为唤醒中断，且配置wakeup io debounce为17个32K晶振cycle
> C）进入sleep低功耗模式
> D）被唤醒后会reset soc重新启动
---

## 常见问题

> 问1：系统进入不了低功耗模式
答：检查系统中断产生情况，如果休眠过程中，系统产生了中断，系统休眠流程会被打断而退出。

> 问2：系统不能唤醒不了
答：1.检查唤醒源配置是否正确，wake gpio 编号和状态是否正确
    2.wakeup timer 的时间是否合理，如果 wakeup timer 时间较短，系统还没完全进入休眠，则 wakeup timer会提前产生中断并被清理，不能起到唤醒系统的作用。

> 问3：进入低功耗模式后功耗偏高
答：1.检查所有设备是否处于合理状态，休眠期间不使用的设备需要设置处于低功耗模式或关闭状态。
    2.休眠期间需要工作并唤醒系统的设备，需要设置成可工作的低功耗模式。
    上述检查都进行后，功耗仍然不能降低至合理值的，需要测试各个分量的功耗，依次找出不合理的分量。

## 参考文档

> 文档资源

1. 没有

> WiKi资源

1. 没有