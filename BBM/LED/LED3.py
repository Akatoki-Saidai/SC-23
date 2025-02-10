#https://monomonotech.jp/kurage/raspberrypi/led_chikachika.html

#timeモジュールをインポート
import time

#RPi.GPIOモジュールをインポート
import RPi.GPIO as GPIO

# BCM(GPIO番号)で指定する設定
GPIO.setmode(GPIO.BCM)

# GPIO17を出力モード設定
GPIO.setup(17, GPIO.OUT)

while True:
  # GPIO17の出力を1にして、LED点灯
  GPIO.output(17, 1)

  # 0.5秒待つ
  time.sleep(0.5)

  # GPIO17の出力を0にして、LED消灯
  GPIO.output(17, 0)

  # 0.5秒待つ
  time.sleep(0.5)