import RPi.GPIO as GPIO
import time
import datetime

# ピンの設定
MOSFET_PIN = 2 # MOSFET を制御する GPIO ピン (BCM)
SWITCH_PIN = 17 # スイッチが接続された GPIO ピン (BCM)

# GPIO の設定
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOSFET_PIN, GPIO.OUT)
GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP) # 内部プルアップ抵抗を使用

try:
    while True:
        # 現在時刻を取得
        now = datetime.datetime.now()

        # 焼き切りを開始する時刻を設定
        start_time = datetime.datetime(now.year, now.month, now.day, 12, 0, 0) # 12時0分0秒

        # スイッチが押されたら、焼き切りを開始
        if GPIO.input(SWITCH_PIN) == GPIO.LOW:
            if now >= start_time:
                GPIO.output(MOSFET_PIN, GPIO.HIGH) # MOSFET を ON にする
                print("焼き切り開始！")
            else:
                print("まだ焼き切り時間ではありません")
        else:
            GPIO.output(MOSFET_PIN, GPIO.LOW) # MOSFET を OFF にする

        time.sleep(1) # 1秒ごとに時刻を確認

except KeyboardInterrupt:
    GPIO.cleanup() # GPIO をクリーンアップ
