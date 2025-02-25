import smbus
import time
from bme280em3 import BME280Sensor

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
except Exception as e:
    print(f"An error occured in setting bme object: {e}")
    print('serious_error', f"An error occured in setting bme280 object: {e}")
    #led_red.blink(0.5, 0.5, 10, 0)

# bmp280高度算出用基準気圧取得
try:
    print(5)
    data = bme.read_data()  # ここでデータを取得
    pressure = bme.compensate_P(data)  # 気圧を補正して取得
    baseline = bme.baseline(pressure)
    print(6)
    print("baseline: ", baseline)
    print(7)
    print('alt_base_press', baseline)
    first_altitude = bme.altitude(pressure)
    print(8)
    print('msg', f'first_altitude: {first_altitude}')
    print(9)
    time.sleep(1)
    print(5)
    time.sleep(1)
    print(4)
    time.sleep(1)
    print(3)
    time.sleep(1)
    print(2)
    time.sleep(1)
    print(1)
    time.sleep(1)
    data = bme.read_data()  # ここでデータを取得
    print(0)
    pressure = bme.compensate_P(data)  # 気圧を補正して取得
    print(0)
    print("alt: ", bme.altitude(pressure, qnh=baseline))
    
    
except Exception as e:
    print(f"An error occured in getting bme data: {e}")
    print('serious_error', f"An error occured in getting bme280 data: {e}")
