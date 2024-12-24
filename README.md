# SC-23
2024年度B1で構成される電装班のプログラムコード置き場です。
#52行までは9軸を用いた加速度検知、56行以降から高度に関して
import smbus
import time
import board
import busio
from bme280 import BME280
from bno055 import BNO055
import numpy as np
import csv_print as csv

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


#if __name__ == '__main__':
    get_accel()()
    while True:
        data = get_accel()()
        print('magX : ', data[0])
        print('magY : ', data[1])
        print('calib status : ', data[6])
        hearding_ang = np.degrees(np.arctan2(data[1], data[0]))
        if hearding_ang < 0:
            hearding_ang += 360
        print("heading_ang : ", hearding_ang)
        print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
        print("accel : ", data[6])
        time.sleep(1)
    
    
    # while True:
    #     print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
    #     print("Magnetometer (microteslas): {}".format(sensor.magnetic))
    #     print("Gyroscope (rad/sec): {}".format(sensor.gyro))
    #     print("Euler angle: {}".format(sensor.euler))
    #     print("Quaternion: {}".format(sensor.quaternion))
    #     print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
    #     print("Gravity (m/s^2): {}".format(sensor.gravity))
    #     print()
    #     time.sleep(1)


# baseline,first_altitudeのの先に定義    
baseline = 1013.25
first_altitude = 0

#droptest.pyより
bus_number  = 1
i2c_address = 0x76

bus = smbus.SMBus(bus_number)

digT = []
digP = []
digH = []

t_fine = 0.0

# ... (writeReg, get_calib_param, compensate_T, compensate_H functions remain the same) ...

def get_pressure():  # 気圧値のみを返す関数
    """BME280センサーから気圧値を取得し、数値で返す関数

    Returns:
        float: 気圧値 (hPa)
    """
    # センサーから生データを読み取る
    data = []
    for i in range(0xF7, 0xF7 + 8):
        data.append(bus.read_byte_data(i2c_address, i))
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
    # hum_raw  = (data[6] << 8)  |  data[7]  # 今回は湿度を使用しないためコメントアウト

    compensate_T(temp_raw)  # 温度補正は気圧計算に必要なので実行
    # compensate_H(hum_raw)  # 今回は湿度を使用しないためコメントアウト

    # 気圧を計算する
    global t_fine
    pressure = 0.0

    v1 = (t_fine / 2.0) - 64000.0
    v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * digP[5]
    v2 = v2 + ((v1 * digP[4]) * 2.0)
    v2 = (v2 / 4.0) + (digP[3] * 65536.0)
    v1 = (((digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8) + ((digP[1] * v1) / 2.0)) / 262144
    v1 = ((32768 + v1) * digP[0]) / 32768

    if v1 == 0:
        return 0  # v1が0の場合はエラーとして0を返す
    pressure = ((1048576 - pres_raw) - (v2 / 4096)) * 3125
    if pressure < 0x80000000:
        pressure = (pressure * 2.0) / v1
    else:
        pressure = (pressure / v1) * 2
    v1 = (digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
    v2 = ((pressure / 4.0) * digP[7]) / 8192.0
    pressure = pressure + ((v1 + v2 + digP[6]) / 16.0)

    return pressure / 100  # 気圧値をhPa単位で返す


def setup():
    # ... (setup function remains the same) ...

setup()
get_calib_param()

# 使用例
pressure_value = get_pressure()  # 気圧値を取得
print(f"気圧: {pressure_value} hPa")


#高度の計算
# 標準大気モデルの定数
T0 = 288.15  # 海面温度（ケルビン）
L = 0.0065   # 温度減率（K/m）
P0 = 1013.25  # 海面の気圧（hPa）
g = 9.80665  # 重力加速度（m/s^2）
R = 287.053  # 空気の比ガス定数（J/(kg·K)

# 高度の計算式
altitude = (T0 / L) * (1 - (pressure_value / P0) ** (R * L / g))
return altitude
