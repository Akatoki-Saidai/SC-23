# このコードは、気圧のみを取得する関数を作成しただけです。

# y方向の加速度の取得を行う関数「get_yaccel()」を作ってください。

# 関数「get_pressure()」、「get_yaccel()」を用いてフローチャートの条件を満たして落下検知をしたら、↲
# 文字列「detected drop!」を表示させてください。
import smbus
import time

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