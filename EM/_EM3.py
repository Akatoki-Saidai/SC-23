import time
import smbus
import RPi.GPIO as GPIO
import make_csv

from bme280ver2 import BME280Sensor
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

        data = bme.read_data()  # ここでデータを取得
        pressure = bme.compensate_P(data)  # 気圧を補正して取得
        make_csv.print("alt_base_press", pressure)
        baseline = bme.baseline(pressure)

    except Exception as e:
        print(f"An error occurred in setting bme object: {e}")
        print('serious_error', f"An error occurred in setting bme280 object: {e}")

    # 9軸のセットアップ
    try:
        bno = BNO055()
        if not bno.begin():
            print("Error initializing device")
        time.sleep(1)
        bno.setExternalCrystalUse(True)

    except Exception as e:
        print(f"An error occurred in setting bno055: {e}")

    phase = 0  # フェーズ0から開始

    try:
        # ここから無限ループ
        while True:
            # ************************************************** #
            #             待機フェーズ(phase = 0)                #
            # ************************************************** #
            if phase == 0:
                try:
                    data = bme.read_data()  
                    pressure = bme.compensate_P(data)  
                    time.sleep(1.0)
                    alt_1 = bme.altitude(pressure, qnh=baseline)

                    linear_accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)
                    accel_x, accel_y, accel_z = linear_accel
                    print(f"accel_z: {accel_z}")
                    time.sleep(0.5)

                    # 落下検知の要件: 高度が10m以上上昇し、加速度が-5.0m/s^2以下
                    if accel_z < -5.0 and alt_1 >= 1:
                        phase = 1
                        print("Go to falling phase")

                except Exception as e:
                    print(f"An error occurred in phase0 : {e}")

            # ************************************************** #
            #             落下フェーズ(phase = 1)                #
            # ************************************************** #
            elif phase == 1:
                try:
                    consecutive_count = 0  # 連続成功回数をカウント

                    for _ in range(10):  # 最大試行回数
                        data = bme.read_data()
                        pressure = bme.compensate_P(data)
                        alt_2 = bme.altitude(pressure, qnh=baseline)

                        linear_accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)
                        accel_x, accel_y, accel_z = linear_accel

                        if accel_z > -0.5 and alt_2 <= 1:  
                            consecutive_count += 1
                            print(f"条件を満たしました: {consecutive_count}/5")
                        else:
                            consecutive_count = 0  # 条件が崩れたらリセット

                        if consecutive_count >= 5:
                            phase = 2
                            print("フェーズ2に移行")
                            break  

                except Exception as e:
                    print(f"An error occurred in phase1: {e}")

            # ************************************************** #
            #             遠距離フェーズ(phase = 2)                #
            # ************************************************** #
            elif phase == 2:
                try:
                    print("フェーズ2: デバッグ中")
                    time.sleep(10)  # デバッグ用
                    distance = get_distance_from_sensor()  
                    if distance <= 10:
                        phase = 3

                except Exception as e:
                    print(f"An error occurred in phase2 : {e}")

            # ************************************************** #
            #             近距離フェーズⅠ(phase = 3)              #
            # ************************************************** #
            elif phase == 3:
                try:
                    red = get_red_value_from_sensor()  
                    if red >= 30:  
                        phase = 4

                except Exception as e:
                    print(f"An error occurred in phase3 : {e}")

            # ************************************************** #
            #             近距離フェーズⅡ(phase = 4)              #
            # ************************************************** #
            elif phase == 4:
                try:
                    if red >= 80:
                        phase = 5

                except Exception as e:
                    print(f"An error occurred in phase4 : {e}")

            # ************************************************** #
            #             ゴールフェーズ(phase = 5)                #
            # ************************************************** #
            elif phase == 5:
                try:
                    print("Goal reached! LED on.")
                except Exception as e:
                    print(f"An error occurred in phase5 : {e}")

    except Exception as e:
        print(f"An error occurred in setting : {e}")

# メイン関数の呼び出し
if __name__ == "__main__":
    main()


