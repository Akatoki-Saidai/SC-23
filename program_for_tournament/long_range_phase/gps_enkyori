import serial
import time
import math
import warnings
import pyproj

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
                    latitude = float(parts[2]) / 100
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
                    longitude = float(parts[4]) / 100
                    ser.close()
                    # print("経度取得完了")
                    return longitude
                    break
            except:
                pass  # 例外処理を追加しました。
        time.sleep(0.01)

"""現在地からゴールまでの距離と角度を計算する関数"""
def calculate_distance_and_angle(current_lat, current_lon, previous_lat, previous_lon):
    # 現在地の緯度経度をメートルに変換
    current_x, current_y = pyproj.transform(wgs84, pyproj.Proj('+proj=utm +zone=54 +ellps=WGS84'), current_lon, current_lat)

    # 前回の現在地（スタート地点）の緯度経度をメートルに変換
    previous_x, previous_y = pyproj.transform(wgs84, pyproj.Proj('+proj=utm +zone=54 +ellps=WGS84'), previous_lon, previous_lat)

    # ゴール地点の緯度経度をメートルに変換
    goal_x, goal_y = pyproj.transform(wgs84, pyproj.Proj('+proj=utm +zone=54 +ellps=WGS84'), goal_lon, goal_lat)

    # スタート地点（前回の現在地）から現在地までの距離を計算する
    distance_current_loc = math.sqrt((current_x - previous_x)**2 + (current_y - previous_y)**2)

    # スタート地点（前回の現在地）からゴール地点までの距離を計算
    distance_current_goal = math.sqrt((goal_x - previous_x)**2 + (goal_y - previous_y)**2)

    # 現在地からゴール地点までの距離を計算
    distance_loc_goal = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)

    # ゴールへの方向を計算 (ラジアン)
    try:
        theta_for_goal = math.pi - math.acos((distance_current_loc ** 2 + distance_loc_goal ** 2 - distance_current_goal ** 2) / (2 * distance_current_loc * distance_loc_goal))
        return distance_loc_goal, theta_for_goal
    except:
        print("移動していません")  # 例外処理: ゼロ除算が発生した場合の処理
        return 2323232323, math.pi * 2

# FutureWarningを抑制
warnings.filterwarnings("ignore", category=FutureWarning)

##################################################
#                      入力                      #
##################################################
# モータを起動させたときの機体の回転速度ω[rad/s]
omega = math.pi / 2  # rad/s

# WGS84楕円体のパラメータを定義
a = 6378137.0
b = 6356752.314245
f = (a - b) / a

##################################################
#                      入力                      #
##################################################
# 宇宙航空研究開発機構(JAXA)種子島宇宙センターグラウンド (ゴール地点の例)
# 緯度経度をWGS84楕円体に基づいて設定
goal_lat = 30.374896924724634  # 緯度
goal_lon = 130.95764140244341  # 経度

# pyprojを使ってWGS84楕円体に基づく投影を定義
wgs84 = pyproj.Proj('+proj=latlong +ellps=WGS84')

# 初期位置の緯度経度を取得
current_lat = get_latitude()
current_lon = get_longitude()

# 移動していない判定のカウンター
no_movement_count = 0

# ゴールの10 m以内に到達するまで繰り返す
while True:
    # 前回の現在地を保存
    previous_lat = current_lat
    previous_lon = current_lon

    print(current_lat, current_lon)  # 現在位置

    # 距離と角度を計算し、表示
    distance_to_goal, angle_to_goal = calculate_distance_and_angle(current_lat, current_lon, previous_lat, previous_lon)
    print("現在地からゴール地点までの距離:", distance_to_goal, "メートル")
    print("theta_for_goal°:", str(angle_to_goal * 180 / math.pi) + "°")

    # 移動していない判定
    if distance_to_goal == 2323232323:  # calculate_distance_and_angle関数で移動していないと判定された場合
        no_movement_count += 1
        print("移動していない判定:", no_movement_count, "回")
        if no_movement_count >= 10:
            print("移動していない判定が10回に達しました。強制的に近距離フェーズに移行します。")
            break  # whileループを抜けて近距離フェーズに移行
    else:
        no_movement_count = 0  # 移動が検出されたらカウンターをリセット

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

    # ゴールの10 m以内に到達したらループを抜け近距離フェーズへ
    if distance_to_goal <= 10:
        print("近距離フェーズに移行")
        break

    # 一時停止 (デバッグ用)
    time.sleep(1)
