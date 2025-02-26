import serial
import time
import math
import warnings
import pyproj

def get_latitude():
    """緯度を取得する関数（値が取得できるまで無限ループ）"""
    ser = serial.Serial('/dev/serial0', 9600, timeout=10)
    print("緯度取得開始")

    while True:
        line = ser.readline().decode('utf-8').strip()
        if line.startswith('$GNGGA') or line.startswith('$GNGLL'):
            parts = line.split(',')
            try:
                if len(parts) >= 7 and int(parts[6]) > 0:
                    latitude = float(parts[2]) / 100
                    ser.close()
                    print("緯度取得完了")
                    return latitude
                    break
            except:
                pass  # 例外処理を追加しました。
        time.sleep(0.01)

def get_longitude():
    """経度を取得する関数（値が取得できるまで無限ループ）"""
    ser = serial.Serial('/dev/serial0', 9600, timeout=10)
    print("経度取得開始")

    while True:
        line = ser.readline().decode('utf-8').strip()
        if line.startswith('$GNGGA') or line.startswith('$GNGLL'):
            parts = line.split(',')
            try:
                if len(parts) >= 7 and int(parts[6]) > 0:
                    longitude = float(parts[4]) / 100
                    ser.close()
                    print("経度取得完了")
                    return longitude
                    break
            except:
                pass  # 例外処理を追加しました。
        time.sleep(0.1)


# FutureWarningを抑制
warnings.filterwarnings("ignore", category=FutureWarning)

# モータを起動させたときの機体の回転速度ω[rad/s]
omega = math.pi / 2  # rad/s

# WGS84楕円体のパラメータを定義
# a: 長半径 (メートル)
# b: 短半径 (メートル)
# f: 扁平率
a = 6378137.0
b = 6356752.314245
f = (a - b) / a

# 埼玉大学　サークル会館 (ゴール地点の例)
# 緯度経度をWGS84楕円体に基づいて設定
goal_lat = 35.861311223522975  # 緯度
goal_lon = 139.60753289749846  # 経度

# 平壌　朝鮮中央動物園 (初期計測位置の例、スタート地点の例)
# 緯度経度をWGS84楕円体に基づいて設定
start_lat = 39.075796567290304  # 緯度
start_lon = 125.81494394894224  # 経度

# pyprojを使ってWGS84楕円体に基づく投影を定義
# Proj('+proj=latlong +ellps=WGS84') は、
# 緯度経度を扱うための投影を指定しています。
# ellps=WGS84 でWGS84楕円体を指定しています。
wgs84 = pyproj.Proj('+proj=latlong +ellps=WGS84')

def calculate_distance_and_angle(current_lat, current_lon):
    """現在地からゴールまでの距離と角度を計算する関数"""

    # 現在地の緯度経度をメートルに変換
    # pyproj.transform を使って、
    # 緯度経度 (current_lat, current_lon) を
    # WGS84楕円体に基づいてメートル単位に変換しています。
    current_x, current_y = pyproj.transform(wgs84, pyproj.Proj('+proj=utm +zone=54 +ellps=WGS84'), current_lon, current_lat)

    # スタート地点の緯度経度をメートルに変換
    start_x, start_y = pyproj.transform(wgs84, pyproj.Proj('+proj=utm +zone=54 +ellps=WGS84'), start_lon, start_lat)

    # ゴール地点の緯度経度をメートルに変換
    goal_x, goal_y = pyproj.transform(wgs84, pyproj.Proj('+proj=utm +zone=54 +ellps=WGS84'), goal_lon, goal_lat)

    # スタート地点から現在地までの距離を計算する
    distance_start_loc = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)

    # スタート地点からゴール地点までの距離を計算
    distance_start_goal = math.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)

    # 現在地からゴール地点までの距離を計算
    distance_loc_goal = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)

    # ゴールへの方向を計算 (ラジアン)
    # ゴール地点、現在地、スタート地点を結ぶ三角形において、
    # 現在地における角度 (theta_for_goal) を余弦定理を用いて計算しています。
    try:
        theta_for_goal = math.pi - math.acos((distance_start_loc ** 2 + distance_loc_goal ** 2 - distance_start_goal ** 2) / (2 * distance_start_loc * distance_loc_goal))
        return distance_loc_goal, theta_for_goal
    except:
        print("移動していません")  # 例外処理: ゼロ除算が発生した場合の処理
        return 100, math.pi * 2


# 初期位置を取得
current_lat = get_latitude()
current_lon = get_longitude()

# ゴールに到達するまで繰り返す
while True:
    # 距離と角度を計算
    distance_to_goal, angle_to_goal = calculate_distance_and_angle(current_lat, current_lon)

    # 距離と角度の情報を表示
    print("現在地からゴール地点までの距離:", distance_to_goal, "メートル")
    print("theta_for_goal(rad):", angle_to_goal)
    print("theta_for_goal°:", str(angle_to_goal * 180 / math.pi) + "°")

    # 進行方向を決定
    if angle_to_goal > 0:
        print("進行方向に対して左方向にゴールがあります")
        # ゴールへの角度に比例した時間だけ左回転
        rotation_time = angle_to_goal / omega  # 回転時間 = 角度 / 回転速度
        # 左に回転する処理をここに記述 (例: motor(-0.5, 0.5))
        # ... (replace with your motor control function) ...
        time.sleep(rotation_time)  # 計算された時間だけ回転
        # ... (stop motor) ...
    else:
        print("進行方向に対して右方向にゴールがあります")
        # ゴールへの角度に比例した時間だけ右回転
        rotation_time = abs(angle_to_goal) / omega  # 回転時間 = 角度 / 回転速度
        # 右に回転する処理をここに記述 (例: motor(0.5, -0.5))
        # ... (replace with your motor control function) ...
        time.sleep(rotation_time)  # 計算された時間だけ回転
        # ... (stop motor) ...

    # 現在地を更新
    current_lat = get_latitude()
    current_lon = get_longitude()

    # ゴールに到達したらループを抜ける
    if distance_to_goal < 10:  # 10メートル以内になったら近距離フェーズに入る
        print("近距離フェーズに移行")
        break

    # 一時停止 (デバッグ用)
    time.sleep(1)
