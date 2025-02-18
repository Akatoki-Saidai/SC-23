import RPi.GPIO as GPIO
import spidev # SPI通信用
import time
import datetime

# ピンの設定
MOSFET_PIN = 2 # MOSFETを制御するGPIOピン (BCM)
# SPIの設定
SPI_BUS = 0
SPI_DEVICE = 0
CURRENT_SENSOR_CHANNEL = 0 # MCP3008のチャンネル

# GPIOの設定
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOSFET_PIN, GPIO.OUT)

# SPIの設定
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)

# 焼き切りを開始する電流値 (A)
THRESHOLD_CURRENT = 0.1

try:
    while True:
        # 電流値を読み取る
        current = read_current()

        # 電流値が閾値を超えたら
        if current > THRESHOLD_CURRENT:
            GPIO.output(MOSFET_PIN, GPIO.HIGH) # MOSFETをONにする
            print("焼き切り開始！")
        else:
            GPIO.output(MOSFET_PIN, GPIO.LOW) # MOSFETをOFFにする

        time.sleep(0.1) # 0.1秒ごとに電流値を確認

except KeyboardInterrupt:
    GPIO.cleanup() # GPIOをクリーンアップ
    spi.close() # SPIをクローズ

# 電流値を読み取る関数
def read_current():
    # SPI通信でMCP3008から値を読み取る
    r = spi.xfer2([0x01, (0x08 + CURRENT_SENSOR_CHANNEL) << 4, 0x00])
    adc_out = ((r[1] & 3) << 8) + r[2]
    voltage = adc_out * (3.3 / 1023.0) # 3.3VはRaspberry Piの電源電圧
    # 電流センサーの仕様に合わせて変換式を調整してください
    current = voltage / 0.185 # ACS712 5Aの場合
    return current
