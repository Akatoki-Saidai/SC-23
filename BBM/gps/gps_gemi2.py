import serial
import time

# シリアルポートの設定
ser = serial.Serial('/dev/ttyS0', 9600)  # Raspberry Pi 4BのUART0を使用する場合

# UART接続を閉じる
ser.close()

# １秒待機
print("待機中")
time.sleep(1)

# UART通信を再開する
print("UART通信開始")
ser.open()

try:
    while True:
        line = serial.Serial('/dev/serial0', 9600, timeout=10).readline().decode('utf-8').strip()
        if line.startswith('$GNGGA') or line.startswith('$GNGLL'):
            parts = line.split(',')
            if len(parts) >= 5:
                latitude = parts[2]
                latitude_direction = parts[3]
                longitude = parts[4]
                longitude_direction = parts[5]
                print(f"{latitude},{latitude_direction},{longitude},{longitude_direction}")
        time.sleep(1)

except KeyboardInterrupt:
    ser.close()
    print('Serial communication closed.')
