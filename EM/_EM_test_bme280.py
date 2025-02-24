import smbus
import time
from bme280 csv import BME280
import make_csv

# 気圧を読み取る関数
def read_pressure(self):
    data = []
    for i in range(0xF7, 0xF7 + 8):
        data.append(self.bus.read_byte_data(self.i2c_address, i))
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    return self.compensate_P(pres_raw)

def main():
  #bme280のセットアップ
  try:
        bus = smbus.SMBus(1)
        bme = BME280(i2c_dev=bus)

        # 初めは異常値が出てくるので，空測定
        for i in range(10):
            try:
                bme.compensate_P()
            except Exception as e:
                print(f"An error occurred during empty measurement in BMP: {e}")
                make_csv.print('msg', f"An error occurred during empty measurement in BMP: {e}")      
    except Exception as e:
        print(f"An error occured in setting bmp object: {e}")
        make_csv.print('serious_error', f"An error occured in setting bmp280 object: {e}")
        #led_red.blink(0.5, 0.5, 10, 0)
  
    
  



      
     # bme280高度算出用基準気圧取得
    try:
        baseline = bmp.get_baseline()
        print("baseline: ", baseline)
        make_csv.print('alt_base_press', baseline)
        first_altitude = bmp.get_altitude(qnh=baseline)
	make_csv.print('msg', f'first_altitude: {first_altitude}')

    except Exception as e:
        print(f"An error occured in getting bmp data: {e}")
        make_csv.print('serious_error', f"An error occured in getting bmp280 data: {e}")
        #led_red.blink(0.5, 0.5, 10, 0)

# メイン関数
# 備考:main()に投げるだけ
if __name__ == "__main__":
	main()
