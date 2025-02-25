import smbus
import time
from bme280em3 import BME280Sensor

try:
        bus = smbus.SMBus(1)
        bme = BME280Sensor(i2c_dev=bus)

        # 初めは異常値が出てくるので，空測定
        for i in range(10):
            try:
                bme.read_data()
            except Exception as e:
                print(f"An error occurred during empty measurement in BME: {e}")
                csv.print('msg', f"An error occurred during empty measurement in BME: {e}")      
    except Exception as e:
        print(f"An error occured in setting bme object: {e}")
        csv.print('serious_error', f"An error occured in setting bme280 object: {e}")
        #led_red.blink(0.5, 0.5, 10, 0)

    # bmp280高度算出用基準気圧取得
    try:
        baseline = bme.baseline(pressure)
        print("baseline: ", baseline)
        csv.print('alt_base_press', baseline)
        first_altitude = bme.altitude(pressure)
        csv.print('msg', f'first_altitude: {first_altitude}')

    except Exception as e:
        print(f"An error occured in getting bme data: {e}")
        csv.print('serious_error', f"An error occured in getting bme280 data: {e}")
