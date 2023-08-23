---
markdown:
  image_dir: /assets/other
  path: /README.md
  ignore_from_front_matter: true
  absolute_image_path: true
export_on_save:
  markdown: true
---

# STM32 双 APP 交替 IAP

## 怎样使用？

### 开发环境

硬件：STM32F407ZGT6 或者其它有带 DMA 通道 USART 的单片机 + 1个按键 + LED 指示灯
系统：Windows、Linux、Mac 均可
软件：STM32CubeMX + Segger Embedded Studio 或 arm-none-eabi 交叉编译工具链

*推荐：[Segger Embedded Studio + STM32CubeMX 跨平台开发环境搭建](https://blog.csdn.net/skb666/article/details/131658780)

```bash
// arm-none-eabi
sudo apt install binutils-arm-none-eabi gcc-arm-none-eabi make
// pipenv
python -m pip install pipenv -U
```

### 生成固件

**克隆仓库**

```bash
git clone https://github.com/skb666/stm32f4_iap.git
```

**编译 bootloader、factory、app**

用 `Segger Embedded Studio` 打开 `iap.emProject` 后点击 `Build -> Build Solution` 进行编译，或者分别进入各目录进行编译：

```bash
cd stm32f4_iap
cd bootloader && make -j12 && cd -  // 编译 bootloader
cd factory && make -j12 && cd -     // 编译 factory
cd app && make -j12 && cd -         // 编译 app
```

### 下载引导程序

可以通过 `Segger Embedded Studio` 分别编译下载 bootloader 和 factory 程序，或者：

```bash
cd uart_iap
// 设置虚拟环境
python -m pipenv update
// 合并 bootloader 和 factory
python -m pipenv run python ./merge.py -a ../bootloader/build/bootloader.bin -c ../factory/build/factory.bin -o output.bin
```

在将 `bootloader` 和 `factory` 合并之后，手动使用 `jlink`、`stlink` 等下载器将 `output.bin` 烧入到单片机 FLASH 地址 `0x8000000` 处

### 升级 APP

在编译完 APP 后，用 `USB to TTL` 连接 PC 与 MCU

```bash
python -m pipenv run python ./uart_iap.py -d COM5 -b 921600 -f ../app/emStudio/Output/Release/Exe/app.bin
python -m pipenv run python ./uart_iap.py -d /dev/ttyUSB0 -b 921600 -f ../app/build/app.bin
```

## 解决了什么问题？

1. 现有的 IAP 双 APP 升级方案在编译 APP 需要手动修改 FLASH 地址，大都需要维护两份 APP 代码
2. 在升级 APP 时不再执行其它业务流程，无法进行后台升级
3. 升级完成后会立即重启，用户无法控制

## FLASH 分区设计

| 扇区  |  启始地址  |    大小 | 名称       | 成分             |
| :---: | :--------: | ------: | :--------- | :--------------- |
|   0   | 0x08000000 |  0x4000 | bootloader | 升级、引导 APP   |
|   1   | 0x08004000 |  0x4000 | factory    | IAP              |
|   2   | 0x08008000 |  0x4000 | param      | 引导参数         |
|   3   | 0x0800C000 |  0x4000 | param_bak  | 参数备份         |
|   4   | 0x08010000 | 0x10000 | -          | -                |
|   5   | 0x08020000 | 0x20000 | -          | -                |
|  6,7  | 0x08040000 | 0x40000 | app_run    | APP 固定引导位置 |
|  8,9  | 0x08080000 | 0x40000 | app1       | APP + IAP        |
| 10,11 | 0x080C0000 | 0x40000 | app2       | APP + IAP        |
