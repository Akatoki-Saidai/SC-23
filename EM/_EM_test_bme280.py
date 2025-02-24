import smbus
import time
from bme280em2 import BME280
import make_csv

class BME280_Extended(BME280):
    # 圧力を取得するメソッドをクラス内に定義
    def read_pressure(self):
        data = []
        # 圧力データの取得
        for i in range(0xF7, 0xF7 + 3):  # 3バイトのみを取得
            data.append(self.bus.read_byte_data(self.i2c_address, i))
        if len(data) < 3:
            raise ValueError("Failed to read enough pressure data")
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        return self.compensate_P(pres_raw)

def main():
    try:
        bus = smbus.SMBus(1)
        bme = BME280_Extended()  # BME280_Extendedインスタンスを作成
        bme.i2c_dev = bus   # I2Cデバイスを設定

        # 初めは異常値が出てくるので、空測定
        for i in range(10):
            try:
                # 圧力を取得
                pressure = bme.read_pressure()
                if pressure is None:
                    raise ValueError("Failed to get valid sensor data")
                print(f"Pressure: {pressure}")
            except Exception as e:
                print(f"An error occurred during empty measurement in BME: {e}")
                make_csv.print('msg', f"An error occurred during empty measurement in BME: {e}")
      
    except Exception as e:
        print(f"An error occurred in setting BME object: {e}")
        make_csv.print('serious_error', f"An error occurred in setting BME280 object: {e}")
    
    # BME280高度算出用基準気圧取得
    try:
        # 高度取得
        first_altitude = bme.get_altitude()  # bmeを使用して高度を取得
        make_csv.print('msg', f'first_altitude: {first_altitude}')
    except Exception as e:
        print(f"An error occurred in getting BME data: {e}")
        make_csv.print('serious_error', f"An error occurred in getting BME280 data: {e}")

# メイン関数
if __name__ == "__main__":
    main()
