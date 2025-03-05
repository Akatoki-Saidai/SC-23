import RPi.GPIO as GPIO  # GPIOモジュールをインポート

from gpiozero import Motor
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory

import time
import random
import numpy as np

from bno055 import BNO055

delta_power = 0.20

# DCモータのピン設定
PIN_AIN1 = 18
PIN_AIN2 = 23
PIN_BIN1 = 24
PIN_BIN2 = 13

dcm_pins = {
    "left_forward": PIN_AIN2,
    "left_backward": PIN_AIN1,
    "right_forward": PIN_BIN2,
    "right_backward": PIN_BIN1,
}

def main():
    # GPIOピン番号モードの設定
    GPIO.setmode(GPIO.BCM)  # または GPIO.setmode(GPIO.BOARD)

    # GPIOピンを出力モードに設定
    GPIO.setup(PIN_AIN1, GPIO.OUT)
    GPIO.setup(PIN_AIN2, GPIO.OUT)
    GPIO.setup(PIN_BIN1, GPIO.OUT)
    GPIO.setup(PIN_BIN2, GPIO.OUT)

    # 初期化
    factory = PiGPIOFactory()
    motor_left = Motor( forward=dcm_pins["left_forward"],
                        backward=dcm_pins["left_backward"],
                        pin_factory=factory)
    motor_right = Motor( forward=dcm_pins["right_forward"],
                        backward=dcm_pins["right_backward"],
                        pin_factory=factory)

    # モーターピンをLOWに設定して、終了後にモーターが動かないようにする
    GPIO.output(PIN_AIN1, GPIO.LOW)
    GPIO.output(PIN_AIN2, GPIO.LOW)
    GPIO.output(PIN_BIN1, GPIO.LOW)
    GPIO.output(PIN_BIN2, GPIO.LOW)

    # GPIOクリーンアップ
    GPIO.cleanup()

if __name__ == "__main__":
    main()






    try:
        # GPIOピン番号ではなく、普通のピン番号
        PIN_AIN1 = 18#12
        PIN_AIN2 = 23#16
        PIN_BIN1 = 24#33
        PIN_BIN2 = 13#18

        motor_right, motor_left = motor.setup(PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2)

    except Exception as e:
        print(f"An error occured in setting motor_driver: {e}")
        csv.print('serious_error', f"An error occured in setting motor_driver: {e}")
        # led_red.blink(0.5, 0.5, 10, 0)



def setup(AIN1, AIN2, BIN1, BIN2):

    dcm_pins = {
                "left_forward": BIN2,
                "left_backward": BIN1,
                "right_forward": AIN1,
                "right_backward": AIN2,
            }


# GPIOピン番号ではなく、普通のラズパイピン番号
PIN_AIN1 = 18#12
PIN_AIN2 = 23#16
PIN_BIN1 = 13#33
PIN_BIN2 = 24#18

delta_power = 0.20

# モーターの初期化
try:
    factory = PiGPIOFactory()
    motor_left = Motor(forward=PIN_BIN2, backward=PIN_BIN1, pin_factory=factory)
    motor_right = Motor(forward=PIN_AIN1, backward=PIN_AIN2, pin_factory=factory)
except Exception as e:
    print(f"An error occured in setting motor_driver: {e}")
    csv.print('serious_error', f"An error occured in setting motor_driver: {e}")
    # led_red.blink(0.5, 0.5, 10, 0)

delta_power = 0.20

def setup(AIN1, AIN2, BIN1, BIN2):
    dcm_pins = {
                "left_forward": BIN2,
                "left_backward": BIN1,
                "right_forward": AIN1,
                "right_backward": AIN2,
            }

    factory = PiGPIOFactory()
    left = motor_left( forward=dcm_pins["left_forward"],
                        backward=dcm_pins["left_backward"],
                        pin_factory=factory)
    right = motor_right( forward=dcm_pins["right_forward"],
                        backward=dcm_pins["right_backward"],
                        pin_factory=factory)
    
    return right, left#returnをすることで他の関数でもこの値を使うことができる。

# 前進関数
def accel(right, left):
    csv.print('motor', [0, 0])
    power = 0
    for i in range(int(1 / delta_power)):
        if 0<=power<=1:
                right.value = power
        left.value = power
        power += delta_power

    right.value = 1
    left.value = 1

    csv.print('motor', [1, 1])
    csv.print('msg', 'motor: accel')

# ブレーキ関数
def brake(right, left):
    power_r = float(right.value)
    power_l = float(left.value)

    csv.print('motor', [power_r, power_l])

    for i in range(int(1 / delta_power)):
        if 0<=power_r<=1 and 0<=power_l<=1:
            right.value = power_r
            left.value = power_l
        if power_r > 0:
            power_r -= delta_power
        elif power_r < 0:
            power_r += delta_power
        else:
            pass
        if power_l > 0:
            power_l -= delta_power
        elif power_l < 0:
            power_l += delta_power
        else:
            pass

    right.value = 0
    left.value = 0
    csv.print('motor', [0, 0])
    csv.print('msg', 'motor: brake')

# 左旋回
def leftturn(right, left):
    
    right.value = 0
    left.value = 0
    csv.print('motor', [0, 0])
    power = 0
    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            right.value = power
            left.value = -1 * power
        
        power += delta_power

    power = 1
    right.value = 1
    left.value = -1
    csv.print('motor', [-1, 1])

    


# 右旋回
def rightturn(right, left):
    
    right.value = 0
    left.value = 0
    csv.print('motor', [0, 0])
    power = 0
    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            right.value = -1 * power
            left.value = power
        
        power += delta_power

    power = 1
    right.value = -1
    left.value = 1
    csv.print('motor', [1, -1])

   
    



def rightonly(right, left):
    
    right.value = 0
    left.value = 0
    csv.print('motor', [0, 0])

    power = 0
    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            right.value = power

        power += delta_power

    power = 1
    right.value = 1
    csv.print('motor_r', 1)

    time.sleep(0.1)

    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            right.value = power
            
        power -= delta_power

    right.value = 0
    csv.print('motor_r', 0)




def leftonly(right, left):
    
    right.value = 0
    left.value = 0
    csv.print('motor', [0, 0])
    power = 0

    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            left.value = power
        
        power += delta_power

    power = 1
    left.value = 1
    csv.print('motor_l', 1)

    time.sleep(0.1)

    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            left.value = power
        
        power -= delta_power
        
    left.value = 0
    csv.print('motor_l', 0)


# 指定した角度だけ右に曲がる
def right_angle(bno, angle_deg, right, left):
    csv.print('msg', f'motor: turn {angle_deg} deg to right')
    angle_rad = angle_deg*np.pi/180
    start_time = time.time()
    prev_time = time.time()
    rot_angle = 0

    # だんだん加速
    for i in range(int(1 / delta_power)):
        right.value, left.value = -i*delta_power, i*delta_power
        gyro = bno.getVector(BNO055.VECTOR_GYROSCOPE)
        angle_diff = gyro[2]*(time.time() - prev_time)  # Δ角度 = 角速度 * Δ時間
        prev_time = time.time()
        rot_angle += angle_diff
        if 3 < gyro[2]:
            break
    right.value, left.value = -1, 1
    csv.print('motor', [left.value, right.value])

    while (prev_time-start_time) < 5:
        try:
            gyro = bno.getVector(BNO055.VECTOR_GYROSCOPE)
            angle_diff = gyro[2]*(time.time() - prev_time)  # Δ角度 = 角速度 * Δ時間
            prev_time = time.time()
            rot_angle += angle_diff
            
            # 指定した角度になる直前に止まる
            if rot_angle + 0.45 > angle_rad:
                break
            
            # ひっくり返っているか判定
            if 0 < bno.getVector(BNO055.VECTOR_GRAVITY)[2]:
                csv.print('warning', 'Starts orientation correction in right_angle')
                accel(right, left)
                time.sleep(0.5)
                brake(right, left)
                csv.print('msg', 'Finish correcting the orientation in right_angle')
        except Exception as e:
            print(f'An error occured in right_angle: {e}')
            csv.print('error', f'An error occured in right_angle: {e}')
    else:
        # スタックしてます
        print('stacking now! in right_angle')
        csv.print('warning', 'stacking now! in right_angle')

        accel(right, left)
        time.sleep(1)
        brake(right, left)

        leftturn(right, left)

        accel(right, left)
        time.sleep(1)
        brake(right, left)

        rightturn(right, left)
    
    # だんだん減速
    for i in range(int(1 / delta_power)):
        right.value, left.value = -1 + i*delta_power, 1 - i*delta_power
    right.value , left.value = 0, 0
    csv.print('motor', [left.value, right.value])

# 指定した角度だけ左に曲がる
def left_angle(bno, angle_deg, right, left):
    csv.print('msg', f'motor: turn {angle_deg} deg to left')
    angle_rad = angle_deg*np.pi/180
    start_time = time.time()
    prev_time = time.time()
    rot_angle = 0
    csv.print('motor', [left.value, right.value])

    # だんだん加速
    for i in range(int(1 / delta_power)):
        right.value, left.value = i*delta_power, -i*delta_power
        gyro = bno.getVector(BNO055.VECTOR_GYROSCOPE)
        angle_diff = gyro[2]*(time.time() - prev_time)  # Δ角度 = 角速度 * Δ時間
        prev_time = time.time()
        rot_angle += angle_diff
        if 3 < gyro[2]:
            break
    right.value, left.value = 1, -1

    while (prev_time-start_time) < 5:
        try:
            gyro = bno.getVector(BNO055.VECTOR_GYROSCOPE)
            angle_diff = gyro[2]*(time.time() - prev_time)  # Δ角度 = 角速度 * Δ時間
            prev_time = time.time()
            rot_angle += angle_diff
            
            # 指定した角度になる直前に止まる
            if rot_angle - 0.45 < -angle_rad:
                break

            # ひっくり返っているか判定
            if 0 < bno.getVector(BNO055.VECTOR_GRAVITY)[2]:
                csv.print('warning', 'Starts orientation correction in left_angle')
                accel(right, left)
                time.sleep(0.5)
                brake(right, left)
                csv.print('msg', 'Finish correcting the orientation in left_angle')
        except Exception as e:
            print(f'An error occured in left_angle: {e}')
            csv.print('error', f'An error occured in left_angle: {e}')
    else:
        # スタックしてます
        print('stacking now! in left_angle')
        csv.print('warning', 'stacking now! in left_angle')

        accel(right, left)
        time.sleep(1)
        brake(right, left)
        
        rightturn(right, left)

        accel(right, left)
        time.sleep(1)
        brake(right, left)

        leftturn(right, left)
    
    # だんだん減速
    for i in range(int(1 / delta_power)):
        right.value, left.value = 1 - i*delta_power, -1 + i*delta_power
    right.value , left.value = 0, 0
    csv.print('motor', [left.value, right.value])


#ここからは未知(2025年2月22日)
def retreat(right, left):
    csv.print('motor', [0, 0])
    power = 0
    for i in range(int(1 / delta_power)):
        if 0<=power<=1:
                right.value = -power
        left.value = -power
        power += delta_power

    right.value = -1
    left.value = -1

    csv.print('motor', [-1, -1])
    csv.print('msg', 'motor: accel')

def stop():
    motor_left.value = 0.0
    motor_right.value = 0.0
    time.sleep(1)

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
def calculate_distance_and_angle(current_lat, current_lon, start_lat, start_lon):
    # 現在地の緯度経度をメートルに変換
    current_x, current_y = pyproj.transform(wgs84, pyproj.Proj('+proj=utm +zone=54 +ellps=WGS84'), current_lon, current_lat)
    # スタート地点の緯度経度をメートルに変換
    start_x, start_y = pyproj.transform(wgs84, pyproj.Proj('+proj=utm +zone=54 +ellps=WGS84'), start_lon, start_lat)
    # ゴール地点の緯度経度をメートルに変換
    goal_x, goal_y = pyproj.transform(wgs84, pyproj.Proj('+proj=utm +zone=54 +ellps=WGS84'), goal_lon, goal_lat)

    # スタート地点から現在地までの距離を計算する
    distance_current_loc = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)
    # スタート地点からゴール地点までの距離を計算
    distance_current_goal = math.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)
    # 現在地からゴール地点までの距離を計算
    distance_loc_goal = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)



    # ゴールへの方向を計算 (ラジアン)
    try:
        theta_for_goal = math.pi - math.acos((distance_current_loc ** 2 + distance_loc_goal ** 2 - distance_current_goal ** 2) / (2 * distance_current_loc * distance_loc_goal))
        
        # 進行方向に対して左右どちらの方向にあるかを判定
        # ここで、進行方向を北向きとした場合の判定ロジックが必要になります。
        # 例えば、現在地と前回の現在地から進行方向ベクトルを計算し、
        # ゴール地点へのベクトルとの外積を計算することで判定できます。
        # 外積の結果が正であれば左方向、負であれば右方向となります。
        
        # 以下は、進行方向が北向きで、東向きを正、西向きを負とした場合の例です。
        # 適切な判定ロジックに置き換えてください。
        if (current_lon - start_lon) * (goal_lat - current_lat) - (current_lat - start_lat) * (goal_lon - current_lon) < 0:
            theta_for_goal *= -1  # 右方向の場合は角度を負にする

        return distance_loc_goal, theta_for_goal
    except:
        print("移動していません")  # 例外処理: ゼロ除算が発生した場合の処理
        return 2323232323, math.pi * 2 # error code

# FutureWarningを抑制
warnings.filterwarnings("ignore", category=FutureWarning)

##################################################
#                      入力                      #
##################################################
# 機体をモータ出力最大で回転させたときの機体の回転速度ω[rad/s]
omega = math.pi * 2  # rad/s

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
start_lat = get_latitude()
start_lon = get_longitude()

# 5 s前進
print("5秒前進")
accel(motor_right, motor_left)
time.sleep(5)
stop()

# 現在位置の緯度経度を取得
current_lat = get_latitude()
current_lon = get_longitude()

# ゴールの10 m以内に到達するまで繰り返す
while True:
    print(current_lat, current_lon)  # 現在位置

    # 距離と角度を計算し、表示
    distance_to_goal, angle_to_goal = calculate_distance_and_angle(current_lat, current_lon, start_lat, start_lon)
    print("現在地からゴール地点までの距離:", distance_to_goal, "メートル")
    print("theta_for_goal°:", str(angle_to_goal * 180 / math.pi) + "°")

    # 前回の現在地を保存
    start_lat = current_lat
    start_lon = current_lon

    # 進行方向を決定
    if angle_to_goal > 0:
        print("進行方向に対して左方向にゴールがあります")
        # ゴールへの角度に比例した時間だけ左回転
        rotation_time = angle_to_goal / omega  # 回転時間 = 角度 / 回転速度
        # 左に回転する処理をここに記述 (例: motor(-0.5, 0.5))
        drv8835.leftturn(motor_right,motor_left)
        time.sleep(rotation_time)
        drv8835.stop()
        # 5秒前進
        drv8835.accel(motor_right,motor_left)
        time.sleep(5)
        drv8835.stop()

    else:
        print("進行方向に対して右方向にゴールがあります")
        # ゴールへの角度に比例した時間だけ右回転
        rotation_time = abs(angle_to_goal) / omega  # 回転時間 = 角度 / 回転速度
        # 右に回転する処理をここに記述 (例: motor(0.5, -0.5))
        drv8835.rightturn(motor_right,motor_left)
        time.sleep(rotation_time)
        drv8835.stop()
        # 5秒前進
        drv8835.accel(motor_right,motor_left)
        time.sleep(5)
        drv8835.stop()

    # 現在地を更新
    current_lat = get_latitude()
    current_lon = get_longitude()

    # ゴールの10 m以内に到達したらループを抜け近距離フェーズへ
    if distance_to_goal <= 10:
        print("近距離フェーズに移行")
        break

    # ちょいまち
    time.sleep(1)
