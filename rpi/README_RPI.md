# 树莓派使用该 GPS 模块（UART/NMEA）示例

该示例会从串口读取 GPS 输出的 NMEA 语句，抓取并解析 `$GPRMC/$GNRMC`（RMC）帧，输出 UTC、定位有效性(A/V)、经纬度（十进制度 + 原始字段）。

## 1) 接线（Raspberry Pi ⇄ GPS）

- **Pi 5V(或3.3V，按你的GPS模块要求)** → GPS VCC  
- **Pi GND** → GPS GND  
- **Pi GPIO14 / TXD0 (物理脚 8)** → GPS RXD（一般可不接，除非要给GPS发配置指令）
- **Pi GPIO15 / RXD0 (物理脚 10)** ← GPS TXD

注意：多数 GPS 模块串口电平是 **3.3V TTL**。如果你的模块是 5V TTL，需要电平转换，避免损坏树莓派。

## 2) 开启树莓派 UART（重要）

用 `raspi-config`：

- **Interface Options → Serial Port**
  - “Login shell over serial?” 选择 **No**（关闭串口登录占用）
  - “Enable serial port hardware?” 选择 **Yes**

重启后，常见串口设备：

- 板载 UART：`/dev/serial0`（推荐用这个，不必关心具体映射到 `ttyAMA0/ttyS0`）
- USB-TTL：通常是 `/dev/ttyUSB0`

## 3) 安装依赖

在树莓派上进入本目录：

```bash
python3 -m pip install -r requirements.txt
```

## 4) 运行

默认读 `/dev/serial0`，波特率 9600：

```bash
python3 gps_rmc_reader.py
```

打印原始 NMEA 行（调试用）：

```bash
python3 gps_rmc_reader.py --raw
```

如果你是 USB-TTL：

```bash
python3 gps_rmc_reader.py --port /dev/ttyUSB0 --baud 9600
```

## 5) 输出示例

- 有效定位（status=A）时，会输出十进制度经纬度
- 无效定位（status=V）时，只输出原始字段

## 6) 与主应用 app.py 集成

项目根目录的 `app.py` 已集成本 RPi GPS 模块：启动时会自动从串口读取 NMEA、解析 RMC，用解析到的经纬度替代“手机端 GPS”参与地图与损伤定位。无需再使用手机打开 `/gps` 页面即可获得定位。

- 串口与波特率可通过环境变量覆盖：`GPS_PORT=/dev/ttyUSB0 GPS_BAUD=9600 python app.py`
- 主应用依赖本目录的 `gps_rmc_reader.parse_rmc_from_line` 及 `pyserial`，请在本项目根目录运行并安装：`pip install pyserial`（或 `pip install -r rpi/requirements.txt`）。

