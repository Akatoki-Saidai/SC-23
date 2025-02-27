import time
import smbus
import RPi.GPIO as GPIO
import make_csv

from bme280ver2 import BME280Sensor
from bno055 import BNO055

def main():

    #温湿度気圧のセットアップ
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
        make_csv.print("alt_base_press",pressure)
        baseline = bme.baseline(pressure)
        make_csv.print("msg","all clear(bme280)")
    except Exception as e:
        print(f"An error occured in setting bme object: {e}")
        make_csv.print('serious_error', f"An error occured in setting bme280 object: {e}")
    #led_red.blink(0.5, 0.5, 10, 0)

    #9軸のセットアップ
    try:
        bno = BNO055()
        if bno.begin() is not True:
            print("Error initializing device")
            make_csv.print("serious_error","Error initializing device")
        time.sleep(1)
        bno.setExternalCrystalUse(True)
        make_csv.print("msg","all clear(bno055)")

    except Exception as e:
        print(f"An error occurred in setting bno055: {e}")
        make_csv.print("serious_error",f"An error occurred in setting bno055: {e}")


    dropping=True
    phase = 1

    while True:

        try:
            
            if dropping:
                dropping = False
                for i in range(5):
                    data = bme.read_data()  # ここでデータを取得
                    pressure = bme.compensate_P(data)  # 気圧を補正して取得
                    alt_2 = bme.altitude(pressure, qnh=baseline)
                    linear_accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)
                    accel_x, accel_y, accel_z = linear_accel
                    print(f"accel_z: {accel_z}")
                    time.sleep(1.0)
                    #落下終了検知の要件に高度が基準高度であるか？加速度変化がないか？
                    if accel_z < -0.5 or alt_2 > 0.50: #下向き加速度が0.5m/s^2以下でなかったら落下中と検知
                        dropping = True
                        make_csv.print("msg","落下してます")
                        print("落下してます")
                    else:
                        make_csv.print("msg","落下してません")
                        print("落下してません")

            else:#5回計測して落下中と検知されなかったら次のフェーズへ
                phase = 2
                make_csv.print("msg","フェーズ2へ行きます")
                print("フェーズ2へ行きます")


        except Exception as e:
            print(f" An error occurred in phase1 : {e}")
            make_csv.print("error",f" An error occurred in phase1 : {e}")











# メイン関数
# 備考:main()に投げるだけ
if __name__ == "__main__":
	main()
