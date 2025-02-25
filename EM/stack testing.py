import time
import make_csv
from BNO055 import BNO055  # BNO055クラスをインポート
import motor               # モーター制御モジュール（仮）

#LEDの設定
#RPi.GPIOモジュールをインポート
import RPi.GPIO as GPIO
# BCM(GPIO番号)で指定する設定
GPIO.setmode(GPIO.BCM)
# GPIO17を出力モード設定
GPIO.setup(17, GPIO.OUT)

# BNO055 初期化
bno = BNO055()
if not bno.begin():
    print("Error initializing BNO055")
    exit()
time.sleep(1)
bno.setExternalCrystalUse(True)

#モーター制御用
motor_right = motor.Motor("right")
motor_left = motor.Motor("left")

def check_stuck():
    #スタック検知を行い、スタック解除の動作を実行する
    try:
        is_stacking = True

        # 5回ジャイロデータを取得し、動きがほぼないかチェック
        for _ in range(5):
            gyro_data = bno.getVector(BNO055.VECTOR_GYROSCOPE)
            gyro_xyz = abs(gyro_data[0]) + abs(gyro_data[1]) + abs(gyro_data[2])
            is_stacking = is_stacking and (gyro_xyz < 0.75)
            time.sleep(0.1)  # 100msごとにデータ取得

        # スタック検知時の処理
        if is_stacking:
            # GPIO17の出力を1にして、LED点灯
            GPIO.output(17, 1)
        
            print("Stacking detected!")
            make_csv.print("warning", "Stacking detected!")

            # スタック解除のための動作
            motor.backward(motor_right, motor_left)  # 3秒後退
            time.sleep(3)

            motor.rightturn(motor_right, motor_left)  # 1秒右旋回
            time.sleep(1)

            motor.forward(motor_right, motor_left)  # 2秒前進
            time.sleep(2)

            motor.brake(motor_right, motor_left)  # 停止

            # GPIO17の出力を0にして、LED消灯
            GPIO.output(17, 0)

    except Exception as e:
        print(f"An error occurred in stack check: {e}")
        make_csv.print("error", f"An error occurred in stack check: {e}")

