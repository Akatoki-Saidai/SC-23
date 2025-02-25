import serial
import time

def get_latitude():
    """緯度を取得する関数（値が取得できるまで無限ループ）"""
    ser = serial.Serial('/dev/serial0', 9600, timeout=10)
    print("緯度取得開始")

    while True:
        line = ser.readline().decode('utf-8').strip()
        if line.startswith('$GNGGA') or line.startswith('$GNGLL'):
            parts = line.split(',')
            if len(parts) >= 7 and int(parts[6]) > 0:
                latitude = float(parts[2]) / 100
                ser.close()
                print("緯度取得完了")
                return latitude
        time.sleep(0.1)

def get_longitude():
    """経度を取得する関数（値が取得できるまで無限ループ）"""
    ser = serial.Serial('/dev/serial0', 9600, timeout=10)
    print("経度取得開始")

    while True:
        line = ser.readline().decode('utf-8').strip()
        if line.startswith('$GNGGA') or line.startswith('$GNGLL'):
            parts = line.split(',')
            if len(parts) >= 7 and int(parts[6]) > 0:
                longitude = float(parts[4]) / 100
                ser.close()
                print("経度取得完了")
                return longitude
        time.sleep(0.1)

# 例: 緯度と経度を取得して表示
latitude = get_latitude()
longitude = get_longitude()

print(f"緯度: {latitude}")
print(f"経度: {longitude}")
