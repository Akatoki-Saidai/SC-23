import time
import serial
from micropyGPS import MicropyGPS

def main():
    # シリアル通信設定
    uart = serial.Serial('/dev/serial0', 38400, timeout=10)
    # gps設定
    my_gps = MicropyGPS(9, 'dd')
    # 10秒ごとに表示
    tm_last = 0
    while True:
        sentence = uart.readline()
        if len(sentence) > 0:
            for x in sentence:
                if 10 <= x <= 126:
                    stat = my_gps.update(chr(x))
                    if stat:
                        tm = my_gps.timestamp
                        tm_now = (tm[0] * 3600) + (tm[1] * 60) + int(tm[2])
                        if (tm_now - tm_last) >= 10:
                            print('=' * 20)
                            print(my_gps.date_string(), tm[0], tm[1], int(tm[2]))
                            print("latitude:", my_gps.latitude[0], ", longitude:", my_gps.longitude[0])
                            tm_last = tm_now

                            # 追加されたprint文
                            # 緯度を表示
                            print(f"Latitude: {my_gps.latitude_string()}")  # 例: Latitude: 35.6895° N
                            # 経度を表示
                            print(f"Longitude: {my_gps.longitude_string()}")  # 例: Longitude: 139.6917° E
                            # 速度を表示
                            print(f"Speed: {my_gps.speed_string()}")  # 例: Speed: 50.0 km/h
                            # 日付を表示
                            print(f"Date: {my_gps.date_string()}")  # 例: Date: 2025/02/22

if __name__ == "__main__":
    main()
