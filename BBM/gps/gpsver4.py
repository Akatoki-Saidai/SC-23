import time
import serial

def parse_nmea_sentence(sentence):
    """NMEA文を解析して緯度、経度、日付を取得する"""
    parts = sentence.split(',')
    if parts[0] == '$GPRMC' and parts[2] == 'A':  # RMC文でデータが有効な場合
        # 緯度の解析
        lat = float(parts[3][:2]) + float(parts[3][2:]) / 60
        if parts[4] == 'S':
            lat = -lat

        # 経度の解析
        lon = float(parts[5][:3]) + float(parts[5][3:]) / 60
        if parts[6] == 'W':
            lon = -lon

        # 日付の解析
        date = parts[9]
        day = date[:2]
        month = date[2:4]
        year = '20' + date[4:6]

        return lat, lon, f'{year}/{month}/{day}'
    return None, None, None

def main():
    # シリアル通信設定
    uart = serial.Serial('/dev/serial0', 38400, timeout=10)
    # 10秒ごとに表示
    tm_last = 0
    while True:
        sentence = uart.readline().decode('ascii', errors='replace').strip()
        if sentence.startswith('$'):
            lat, lon, date = parse_nmea_sentence(sentence)
            if lat is not None and lon is not None and date is not None:
                tm_now = time.time()
                if (tm_now - tm_last) >= 10:
                    print('=' * 20)
                    # 緯度を表示
                    print(f"Latitude: {lat:.6f}°")  # 例: Latitude: 35.6895°
                    # 経度を表示
                    print(f"Longitude: {lon:.6f}°")  # 例: Longitude: 139.6917°
                    # 日付を表示
                    print(f"Date: {date}")  # 例: Date: 2025/02/22
                    tm_last = tm_now

if __name__ == "__main__":
    main()
