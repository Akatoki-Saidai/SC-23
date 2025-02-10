import RPi.GPIO as GPIO  # GPIOに関わるライブラリ宣言
import time  # Pythonで時間に関わる処理のライブラリ宣言

# GPIOピンの番号を変数に格納
LED = 2
LED2 = 4
LED3 = 17
wait = 1  # 待機時間（秒）

# GPIOのモードをBCMに設定
GPIO.setmode(GPIO.BCM)

# LEDを接続するGPIOピンを出力モードに設定
GPIO.setup(LED, GPIO.OUT)
GPIO.setup(LED2, GPIO.OUT)
GPIO.setup(LED3, GPIO.OUT)

# LEDを点滅させるループ
try:
    while True:
        GPIO.output(LED, GPIO.HIGH)  # GPIO2の出力をHIGH（点灯）
        time.sleep(wait)
        GPIO.output(LED, GPIO.LOW)  # GPIO2の出力をLOW（消灯）

        GPIO.output(LED2, GPIO.HIGH)  # GPIO4の出力をHIGH（点灯）
        time.sleep(wait)
        GPIO.output(LED2, GPIO.LOW)  # GPIO4の出力をLOW（消灯）

        GPIO.output(LED3, GPIO.HIGH)  # GPIO17の出力をHIGH（点灯）
        time.sleep(wait)
        GPIO.output(LED3, GPIO.LOW)  # GPIO17の出力をLOW（消灯）

        time.sleep(wait)

except KeyboardInterrupt:
    GPIO.cleanup()  # 終了時にGPIOをリセット
    print("\nGPIOをクリーンアップして終了")
