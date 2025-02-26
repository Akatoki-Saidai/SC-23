import time
import smbus
import RPi.GPIO as GPIO
import make_csv

from bme280ver3 import BME280Sensor
from bno055 import BNO055

def main():
    # 温湿度気圧のセットアップ
    try:
        bus = smbus.SMBus(1)
        bme = BME280Sensor(bus_number=1)

        # 初めは異常値が出てくるので，空測定
        for i in range(10):
            try:
                bme.read_data()
            except Exception as e:
                print(f"An error occurred during empty measurement in BME: {e}")
                print('msg', f"An error occurred during empty measurement in BME: {e}")

        temp_raw, pres_raw = bme.read_data()  # ここでデータを取得
        pressure = bme.compensate_P(pres_raw)  # 気圧を補正して取得
        temperature=bme.compensate_T(temp_raw)  # 温度補正
        make_csv.print("alt_base_press", pressure)
        baseline = bme.baseline(pressure)
    except Exception as e:
        print(f"An error occurred in setting bme object: {e}")
        print('serious_error', f"An error occurred in setting bme280 object: {e}")

    # 9軸のセットアップ
    try:
        bno = BNO055()
        if bno.begin() is not True:
            print("Error initializing device")
        time.sleep(1)
        bno.setExternalCrystalUse(True)
    except Exception as e:
        print(f"An error occurred in setting bno055: {e}")

    phase = 0  # フェーズ0から開始

    try:
        # ここからずっと繰り返し
        while True:
            # ************************************************** #
            #             待機フェーズ(phase = 0)                #
            # ************************************************** #
            if(phase == 0):
                try:
                    temp_raw, pres_raw = bme.read_data()  # ここでデータを取得
                    pressure = bme.compensate_P(pres_raw)  # 気圧を補正して取得
                    temperature = bme.compensate_T(temp_raw)  # 温度補正
                    time.sleep(1.0)
                    alt_1 = bme.altitude(pressure, temperature, qnh=baseline)

                    linear_accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)
                    accel_x, accel_y, accel_z = linear_accel
                    print(f"accel_z: ", {linear_accel})
                    time.sleep(0.5)

                    # 落下検知の要件に高度が10m以上上昇したか？を追加予定
                    if(alt_1 >= 5):
                        phase = 1  # 下向き加速度が5.0m/s^2を超え,かつ高度が10m以上上昇したら落下検知
                        print("Go to falling phase")

                except Exception as e:
                    print(f"An error occurred in phase0 : {e}")

            # ************************************************** #
            #             落下フェーズ(phase = 1)                #
            # ************************************************** #
            if(phase == 1):
                try:
                    temp_raw, pres_raw = bme.read_data()  # ここでデータを取得
                    pressure = bme.compensate_P(pres_raw)  # 気圧を補正して取得
                    temperature = bme.compensate_T(temp_raw)  # 温度補正
                    time.sleep(1.0)
                    alt_2 = bme.altitude(pressure, temperature, qnh=baseline)

                    linear_accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)
                    accel_x, accel_y, accel_z = linear_accel
                    print(f"accel_z: ", {linear_accel})
                    time.sleep(0.5)

                    # 落下終了検知の要件に高度が基準高度であるか？加速度変化がないか？を追加予定
                    if(accel_z > -0.50) and (alt_2 <= 1):  # 下向き加速度が0.5m/s^2以下だったらフェーズ2に移行
                        phase = 2
                        print("Go to phase2")
                        
                except Exception as e:
                    print(f"An error occurred in phase1 : {e}")

            # ************************************************** #
            #             遠距離フェーズ(phase = 2)                #
            # ************************************************** #
            if(phase == 2):
                try:
                    print("2")
                    time.sleep(10000) # デバッグ用
                    # 仮にdistance変数を設定 (実際の距離センサーの値を取得する必要があります)
                    distance = get_distance_from_sensor()  # センサーから距離を取得する関数を追加
                    if(distance <= 10):  # 目標までの距離が10mを切ったらフェーズ3に移行
                        phase = 3
                except Exception as e:
                    print(f"An error occurred in phase2 : {e}")

            # ************************************************** #
            #             近距離フェーズⅠ(phase = 3)              #
            # ************************************************** #
            if(phase == 3):
                try:
                    # 仮にred変数を設定 (実際の赤色センサーの値を取得する必要があります)
                    red = get_red_value_from_sensor()  # センサーから赤色値を取得する関数を追加
                    if(red >= 30):  # 赤色が30%以上しめていたらフェーズ4に移行
                        phase = 4
                except Exception as e:
                    print(f"An error occurred in phase3 : {e}")

            # ************************************************** #
            #             近距離フェーズⅡ(phase = 4)              #
            # ************************************************** #
            if(phase == 4):
                try:
                    if(red >= 80):  # 赤色が80%以上しめていたらフェーズ5に移行
                        phase = 5
                except Exception as e:
                    print(f"An error occurred in phase4 : {e}")

            # ************************************************** #
            #             ゴールフェーズ(phase = 5)                #
            # ************************************************** #
            if(phase == 5):
                try:
                    # LEDを点灯
                    print("Goal reached! LED on.")
                    # ここでLED点灯処理を行う
                except Exception as e:
                    print(f"An error occurred in phase5 : {e}")

    except Exception as e:
        print(f"An error occurred in setting : {e}")


# 以下，いつか消すコード
# コードの置き場
# 自作関数を他のスクリプトで作った人はとりあえずこのスクリプトで呼び出す方法をここにメモして

# 温湿度気圧センサー
try:
    while True:
        data = bme.read_data()  # ここでデータを取得
        pressure = bme.compensate_P(data)  # 気圧を補正して取得
        print("alt: ", bme.altitude(pressure, qnh=baseline))  # 初期高度に対する相対高度出力
        time.sleep(1)
except Exception as e:
    print(f"An error occurred in setting : {e}")


# 9軸の値を取得してprint(サンプルコード)
try:
    while True:
        euler = bno.getVector(BNO055.VECTOR_EULER)
        print("オイラー角:", euler)
        print("加速度:", bno.getVector(BNO055.VECTOR_LINEARACCEL))
        print("加速度:", bno.getVector(BNO055.VECTOR_ACCELEROMETER))
        print("磁力計:", bno.getVector(BNO055.VECTOR_MAGNETOMETER))
        print("ジャイロ:", bno.getVector(BNO055.VECTOR_GYROSCOPE))
        print("重力:", bno.getVector(BNO055.VECTOR_GRAVITY))

except Exception as e:
    print(f"An error occurred in print bno055 date: {e}")
    
# ニクロム線を切ります
# 使うpin番号
pin = 16
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.OUT)
GPIO.output(pin, 1)
# 電流を送る時間
time.sleep(5)
GPIO.output(pin, 0)

############################################
# ここより下は消さない
#################################

# メイン関数
# 備考:main()に投げるだけ
if __name__ == "__main__":
    main()

