import serial
import time

"""緯度を取得する関数（値が取得できるまで無限ループ）"""
def get_latitude():
    ser = serial.Serial('/dev/serial0', 9600, timeout=10)
    # print("緯度取得開始")

    while True:
        line = ser.readline().decode('utf-8').strip()
        if line.startswith('$GNGGA') or line.startswith('$GNGLL'):
            parts = line.split(',')
            try:
                if len(parts) >= 7 and int(parts[6]) > 0:
                    latitude_deg_min = float(parts[2]) / 100  # 度分形式で取得
                    
                    # 度数形式に変換
                    degrees = int(latitude_deg_min)
                    minutes = latitude_deg_min - degrees
                    latitude = degrees + minutes / 0.6 
                    
                    ser.close()
                    # print("緯度取得完了")
                    return latitude
                    break
            except:
                pass  # 例外処理を追加しました。
        time.sleep(0.01)

"""経度を取得する関数（値が取得できるまで無限ループ）"""
def get_longitude():
    ser = serial.Serial('/dev/serial0', 9600, timeout=10)
    # print("経度取得開始")

    while True:
        line = ser.readline().decode('utf-8').strip()
        if line.startswith('$GNGGA') or line.startswith('$GNGLL'):
            parts = line.split(',')
            try:
                if len(parts) >= 7 and int(parts[6]) > 0:
                    longitude_deg_min = float(parts[4]) / 100 # 度分形式で取得
                    
                    # 度数形式に変換
                    degrees = int(longitude_deg_min)
                    minutes = longitude_deg_min - degrees
                    longitude = degrees + minutes / 0.6
                    
                    ser.close()
                    # print("経度取得完了")
                    return longitude
                    break
            except:
                pass  # 例外処理を追加しました。
        time.sleep(0.01)

# 例: 緯度と経度を取得して表示
latitude = 0
longitude = 0
n = input("平均を何回取りますか")
for i in range(1, n + 1):
    print(f"{i}回目")
    latitude += get_latitude()
    longitude += get_longitude()
latitude / n
longitude / n

print(f"緯度: {latitude}")
print(f"経度: {longitude}")
