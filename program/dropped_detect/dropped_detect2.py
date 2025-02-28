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
                make_csv.print('msg', f"An error occurred during empty measurement in BME: {e}")

        data = bme.read_data()  # ここでデータを取得
        pressure = bme.compensate_P(data)  # 気圧を補正して取得
        make_csv.print("alt_base_press", pressure)
        baseline = bme.baseline(pressure)
        make_csv.print("msg", "all clear(bme280)")

    except Exception as e:
        print(f"An error occurred in setting bme object: {e}")
        make_csv.print('serious_error', f"An error occurred in setting bme280 object: {e}")

    # 9軸のセットアップ
    try:
        bno = BNO055()
        if bno.begin() is not True:
            print("Error initializing device")
            make_csv.print("serious_error", "Error initializing device")
        time.sleep(1)
        bno.setExternalCrystalUse(True)
        make_csv.print("msg", "all clear(bno055)")

    except Exception as e:
        print(f"An error occurred in setting bno055: {e}")
        make_csv.print("serious_error", f"An error occurred in setting bno055: {e}")

    # 落下検知の処理
    dropping = True
    phase = 1

    try:
        consecutive_count = 0  # 連続成功回数をカウント

        for _ in range(10):  # 最大試行回数 (適当に10回としている)
            data = bme.read_data()  # ここでデータを取得
            pressure = bme.compensate_P(data)  # 気圧を補正して取得
            alt_2 = bme.altitude(pressure, qnh=baseline)

            linear_accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)
            accel_x, accel_y, accel_z = linear_accel

            # 落下終了検知の要件: 高度が基準高度であり、加速度変化がないか？
            if (accel_z > -0.5) and (alt_2 <= 1):  
                consecutive_count += 1  # 連続回数を増やす
                print(f"条件を満たしました: {consecutive_count}/5")
            else:
                consecutive_count = 0  # 条件が崩れたらリセット
            
            # 5回連続で条件を満たしたらフェーズ2に移行
            if consecutive_count >= 5:
                phase = 2
                print("フェーズ2に移行")
                break  # ループ終了

    except Exception as e:
        print(f"An error occurred in phase1: {e}")

if __name__ == "__main__":
    main()


