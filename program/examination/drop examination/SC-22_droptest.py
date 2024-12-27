import time
from smbus2 import SMBus
import serial
import busio
from gpiozero import LED
from picamera2 import Picamera2
import math
import numpy as np
import cv2

# scに使用ライブラリほぼまとめました
import motor
import print_override as override
from camera import Camera
from bme280 import BME280
from bno055 import BNO055
from micropyGPS import MicropyGPS
import csv_print as csv
import calc_xy

def main():
    
    # ---セットアップゾーン---
    # LEDをセット
    try:
        led_green = LED(27)
        led_green.off()
        led_red = LED(10)
        led_red.off()
    except Exception as e:
        print(f"An error occured in setting LED: {e}")
        csv.print('error', f"An error occured in setting LED: {e}")
    
    
        
    # baselineも先に定義
    baseline = 1013.25
    first_altitude = 0


    try:
        #BNOの電源ピンをHighにする
        v_bno = LED(11)
        v_bno.on()
        #BNOのリセットピンをHighにする
        v_bno_reset = LED(24)
        v_bno_reset.on()

        #BMPの電源ピンをHighにする
        v_bme = LED(22)
        v_bme.on()

        # 抵抗を明示的にLowにしておく
        NiCr_PIN = LED(17)
        NiCr_PIN.off()
        
        # wait
        time.sleep(3)


    except Exception as e:
        print(f"An error occured in turn on bmp, bno: {e}")
        csv.print('serious_error', f"An error occured in turn on bmp, bno: {e}")
        led_red.blink(0.5, 0.5, 10, 0)




    #bno055のセットアップ
    try:
        bno = BNO055()
        if bno.begin() is not True:
            print("Error initializing device")
            pass
        time.sleep(1)
        bno.setExternalCrystalUse(False)
    except Exception as e:
        print(f"An error occured in setting bno055 object: {e}")
        csv.print('serious_error', f"An error occured in setting bno055 object: {e}")
        led_red.blink(0.5, 0.5, 10, 0)
        

    #bme280のセットアップ
    try:
        bus = SMBus(1)
        bme = BME280(i2c_dev=bus)

        # 初めは異常値が出てくるので，空測定
        for i in range(10):
            try:
                bme.get_temp_pres()
            except Exception as e:
                print(f"An error occurred during empty measurement in BMP: {e}")
                csv.print('msg', f"An error occurred during empty measurement in BMP: {e}")      
    except Exception as e:
        print(f"An error occured in setting bmp object: {e}")
        csv.print('serious_error', f"An error occured in setting bmp280 object: {e}")
        led_red.blink(0.5, 0.5, 10, 0)

    # bme280高度算出用基準気圧取得
    try:
        baseline = bme.get_baseline()
        print("baseline: ", baseline)
        # csv.print('alt_base_press', baseline)
        first_altitude = bme.get_altitude(qnh=baseline)
        csv.print('msg', f'first_altitude: {first_altitude}')

    except Exception as e:
        print(f"An error occured in getting bmp data: {e}")
        csv.print('serious_error', f"An error occured in getting bmp280 data: {e}")
        led_red.blink(0.5, 0.5, 10, 0)





    # ---繰り返しゾーン---

    while True:
        try:

            csv.print('phase', phase)

            # ************************************************** #
            #             待機フェーズ(phase = 0)                #
            # ************************************************** #
            
            if (phase == 0):

                try:
                    led_green.on()
                    led_red.off()

                    # bmp280で高度(altitude)を計測
                    def get_pressure():
                        try:
                            # temperature = bmp.get_temperature()
                            # pressure = bmp.get_pressure()
                            altitude = bme.get_altitude(qnh=baseline)
                            # print(f"temperture{temperature:05.2f}*C")
                            # print(f"pressure: {pressure:05.2f}hPa")
                        # 高度をprint
                            print(f"Relative altitude: {altitude:05.2f} metres")
                        except Exception as e:
                            print(f"An error occured in reading bmp: {e}")
                            csv.print('error', f"An error occured in reading bmp: {e}")
                except Exception as e:
                    print(f"An error occured in waiting phase: {e}")
                    csv.print('error', f"An error occured in waiting phase: {e}")

                    
                #52行までhttps://github.com/yuzu2yan/Noshiro_Space_Event_2023/blob/main/bno055.py を参考にした
                i2c = busio.I2C(3, 2)#i2c分からなかった
                sensor = BNO055.bno055_I2C(i2c)

                def get_accel():
                    accelX, accelY, accelZ = sensor.acceleration
                    accelZ = accelZ - 9.8
                    accel = np.sqrt(accelX**2 + accelY**2 + accelZ**2)
                    data = [sensor.magnetic[0], sensor.magnetic[1], sensor.magnetic[2], sensor.acceleration[0], sensor.acceleration[1], sensor.acceleration[2], accel, sensor.calibration_status[3], sensor.calibration_status[2]]
                    """
                    data = [magX, magY, magZ, accelX, accelY, accelZ, accel, calib_mag, calib_accel]
                    calib status : 0 ~ 3
                    """
                    return data

                # bmeの高度の値とbaselineの値(地上の高度)とz軸方向の加速度を比較し，その結果で条件分岐
                # 条件式を記述し，フェーズ移行
                if (altitude - first_altitude > 10 and data[5] >= 10):
                    phase = 1
                    print("detected drop!")
                    csv.print('msg', 'detected drop!')
                    led_green.blink(0.5, 0.5)
                else:
                    pass

        except Exception as e:
            print(f"An error occured in waiting phase: {e}")
            csv.print('error', f"An error occured in waiting phase: {e}")



            
            
            # ************************************************** #
            #             落下フェーズ(phase = 1)                #
            # ************************************************** #
            
            
