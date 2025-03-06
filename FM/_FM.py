import time
import smbus
import RPi.GPIO as GPIO
import make_csv
import cv2

import serial
import time
import math
import warnings
import pyproj

from bno055 import BNO055
import make_csv
from bme280ver2 import BME280Sensor
from picamera2 import Picamera2




####
#モータ系
##

from gpiozero import Motor
from gpiozero.pins.pigpio import PiGPIOFactory
import numpy as np

####################################
#変数___入力
##########################



# モータを起動させたときの機体の回転速度ω[rad/s]
omega = math.pi / 2  # rad/s

# WGS84楕円体のパラメータを定義
a = 6378137.0
b = 6356752.314245
f = (a - b) / a

# 宇宙航空研究開発機構(JAXA)種子島宇宙センターグラウンド (ゴール地点の例)
# 緯度経度をWGS84楕円体に基づいて設定
goal_lat = 30.374896924724634  # 緯度
goal_lon = 130.95764140244341  # 経度

# pyprojを使ってWGS84楕円体に基づく投影を定義
wgs84 = pyproj.Proj('+proj=latlong +ellps=WGS84')

##############################
#LEDの話
############################

# BCM(GPIO番号)で指定する設定
GPIO.setmode(GPIO.BCM)
# GPIO5を出力モード設定
GPIO.setup(5, GPIO.OUT)

##############################################
###GPSの話
##################################################



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

    # 前回の現在地（スタート地点）の緯度経度をメートルに変換
    start_x, start_y = pyproj.transform(wgs84, pyproj.Proj('+proj=utm +zone=54 +ellps=WGS84'), start_lon, start_lat)

    # ゴール地点の緯度経度をメートルに変換
    goal_x, goal_y = pyproj.transform(wgs84, pyproj.Proj('+proj=utm +zone=54 +ellps=WGS84'), goal_lon, goal_lat)

    # スタート地点から現在地までの距離を計算する
    distance_start_current = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)

    # スタート地点からゴール地点までの距離を計算
    distance_start_goal = math.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)

    # 現在地からゴール地点までの距離を計算
    distance_current_goal = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)

    # ゴールへの方向を計算 (ラジアン)
    try:
        theta_for_goal = math.pi - math.acos((distance_start_current ** 2 + distance_start_goal ** 2 - distance_current_goal ** 2) / (2 * distance_current_loc * distance_loc_goal))
        return distance_start_goal, theta_for_goal
    except:
        print("移動していません")  # 例外処理: ゼロ除算が発生した場合の処理
        return 2323232323, math.pi * 2

# FutureWarningを抑制
warnings.filterwarnings("ignore", category=FutureWarning)



#################################################
##モータードライバの話
#################################################

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
    make_csv.print('serious_error', f"An error occured in setting motor_driver: {e}")
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
def retreat(right, left):
    make_csv.print('motor', [0, 0])
    power = 0
    for i in range(int(1 / delta_power)):
        if 0<=power<=1:
                right.value = power
        left.value = power
        power += delta_power

    right.value = 1
    left.value = 1

    make_csv.print('motor', [1, 1])
    make_csv.print('msg', 'motor: accel')

# ブレーキ関数
def brake(right, left):
    power_r = float(right.value)
    power_l = float(left.value)

    make_csv.print('motor', [power_r, power_l])

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
    make_csv.print('motor', [0, 0])
    make_csv.print('msg', 'motor: brake')

# 左旋回
def rightturn(right, left):

    right.value = 0
    left.value = 0
    make_csv.print('motor', [0, 0])
    power = 0
    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            right.value = power
            left.value = -1 * power

        power += delta_power

    power = 1
    right.value = 1
    left.value = -1
    make_csv.print('motor', [-1, 1])

# 右旋回
def leftturn(right, left):
    
    right.value = 0
    left.value = 0
    make_csv.print('motor', [0, 0])
    power = 0
    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            right.value = -1 * power
            left.value = power
        
        power += delta_power

    power = 1
    right.value = -1
    left.value = 1
    make_csv.print('motor', [1, -1])


def rightonly(right, left):
    
    right.value = 0
    left.value = 0
    make_csv.print('motor', [0, 0])

    power = 0
    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            right.value = power

        power += delta_power

    power = 1
    right.value = 1
    make_csv.print('motor_r', 1)

    time.sleep(0.1)

    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            right.value = power
            
        power -= delta_power

    right.value = 0
    make_csv.print('motor_r', 0)


def leftonly(right, left):
    
    right.value = 0
    left.value = 0
    make_csv.print('motor', [0, 0])
    power = 0

    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            left.value = power
        
        power += delta_power

    power = 1
    left.value = 1
    make_csv.print('motor_l', 1)

    time.sleep(0.1)

    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            left.value = power
        
        power -= delta_power
        
    left.value = 0
    make_csv.print('motor_l', 0)

# 指定した角度だけ右に曲がる
def right_angle(bno, angle_deg, right, left):
    make_csv.print('msg', f'motor: turn {angle_deg} deg to right')
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
    make_csv.print('motor', [left.value, right.value])

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
                make_csv.print('warning', 'Starts orientation correction in right_angle')
                accel(right, left)
                time.sleep(0.5)
                brake(right, left)
                make_csv.print('msg', 'Finish correcting the orientation in right_angle')
        except Exception as e:
            print(f'An error occured in right_angle: {e}')
            make_csv.print('error', f'An error occured in right_angle: {e}')
    else:
        # スタックしてます
        print('stacking now! in right_angle')
        make_csv.print('warning', 'stacking now! in right_angle')

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
    make_csv.print('motor', [left.value, right.value])

# 指定した角度だけ左に曲がる
def left_angle(bno, angle_deg, right, left):
    make_csv.print('msg', f'motor: turn {angle_deg} deg to left')
    angle_rad = angle_deg*np.pi/180
    start_time = time.time()
    prev_time = time.time()
    rot_angle = 0
    make_csv.print('motor', [left.value, right.value])

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
                make_csv.print('warning', 'Starts orientation correction in left_angle')
                accel(right, left)
                time.sleep(0.5)
                brake(right, left)
                make_csv.print('msg', 'Finish correcting the orientation in left_angle')
        except Exception as e:
            print(f'An error occured in left_angle: {e}')
            make_csv.print('error', f'An error occured in left_angle: {e}')
    else:
        # スタックしてます
        print('stacking now! in left_angle')
        make_csv.print('warning', 'stacking now! in left_angle')

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
    make_csv.print('motor', [left.value, right.value])


#ここからは未知(2025年2月22日)
def accel(right, left):
    make_csv.print('motor', [0, 0])
    power = 0
    for i in range(int(1 / delta_power)):
        if 0<=power<=1:
                right.value = -power
        left.value = -power
        power += delta_power

    right.value = -1
    left.value = -1

    make_csv.print('motor', [-1, -1])
    make_csv.print('msg', 'motor: accel')

def stop():
    motor_left.value = 0.0
    motor_right.value = 0.0

'''
print("retreat")
retreat(motor_right,motor_left)
time.sleep(2)
stop()

rint("accel")
accel(motor_right,motor_left)
time.sleep(2)
stop()

print("rightturn")
rightturn(motor_right,motor_left)
stop()

print("leftturn")
leftturn(motor_right,motor_left)
stop()

print("Finish!!!!!!!!!!")

#####################################################

'''


#################################
##カメラの話
############################


def red_detect(frame):
    # HSV色空間に変換
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 赤色のHSVの値域1
    hsv_min = np.array([0, 117, 104])
    hsv_max = np.array([11, 255, 255])
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

    # 赤色のHSVの値域2
    hsv_min = np.array([169, 117, 104])
    hsv_max = np.array([179, 255, 255])
    mask2 = cv2.inRange(hsv, hsv_min, hsv_max)
 
    return mask1 + mask2

def analyze_red(frame, mask):
        
    camera_order = 4
    # 画像の中にある領域を検出する
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
    #画像の中に赤の領域があるときにループ
    if 0 < len(contours):
                    
        # 輪郭群の中の最大の輪郭を取得する-
        biggest_contour = max(contours, key=cv2.contourArea)

        # 最大の領域の外接矩形を取得する
        rect = cv2.boundingRect(biggest_contour)

        # 最大の領域の中心座標を取得する
        center_x = (rect[0] + rect[2] // 2)
        center_y = (rect[1] + rect[3] // 2)

        # 最大の領域の面積を取得する-
        area = cv2.contourArea(biggest_contour)

        # 最大の領域の長方形を表示する
        cv2.rectangle(frame, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 0, 255), 2)

        # 最大の領域の中心座標を表示する
        cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)

        # 最大の領域の面積を表示する
        cv2.putText(frame, str(area), (rect[0], rect[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 1)

        cv2.putText(frame, str(center_x), (center_x, center_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 1)


        frame_center_x = frame.shape[1] // 2
        # 中心座標のx座標が画像の中心より大きいか小さいか判定
        if area > 50000:
            print("十分近い")
            camera_order = 0
            stop()

        else:
            if frame_center_x -  50 <= center_x <= frame_center_x + 50:
                #accel(motor_right,motor_left)
                print("赤色物体は画像の中心にあります。")#直進
                camera_order = 1
                
            elif center_x > frame_center_x + 50:
                #rightturn(motor_right,motor_left)
                print("赤色物体は画像の右側にあります。")#右へ
                camera_order = 2
                stop()
                
            elif center_x < frame_center_x - 50:
                #leftturn(motor_right,motor_left)
                print("赤色物体は画像の左側にあります。")#左へ
                camera_order = 3
                stop()

    else:
        print("何もないです未検出")
        #accel(motor_right, motor_left)
        camera_order = 4

        # red_result = cv2.drawContours(mask, [biggest_contour], -1, (0, 255, 0), 2)

    return camera_order





############################################
#スタック検知の話
##########################################


def check_stuck():
    #スタック検知を行い、スタック解除の動作を実行する
    try:
        is_stacking = True
        
        bno = BNO055()
        if not bno.begin():
            print("Error initializing device")
            make_csv.print("Error initializing device")
        time.sleep(1)
        bno.setExternalCrystalUse(True)

        # 5回ジャイロデータを取得し、動きがほぼないかチェック
        for _ in range(5):
            gyro_data = bno.getVector(BNO055.VECTOR_GYROSCOPE)
            gyro_xyz = abs(gyro_data[0]) + abs(gyro_data[1]) + abs(gyro_data[2])
            is_stacking = is_stacking and (gyro_xyz < 0.75)
            time.sleep(0.1)  # 100msごとにデータ取得

        # スタック検知時の処理
        if is_stacking:
            # GPIO5の出力を1にして、LED点灯
            for i in range(0,2):
                GPIO.output(5,1)
                time.sleep(0.5)
                GPIO.output(5,0)
                time.sleep(0.5)
            GPIO.output(5, 1)
        
            print("Stacking detected!")
            make_csv.print("warning", "Stacking detected!")

            # スタック解除のための動作
            retreat(motor_right, motor_left)  # 3秒後退
            time.sleep(3)

            rightturn(motor_right, motor_left)  # 1秒右旋回
            time.sleep(1)

            accel(motor_right, motor_left)  # 2秒前進
            time.sleep(2)

            stop()  # 停止

            # GPIO17の出力を0にして、LED消灯
            GPIO.output(17, 0)

            time.sleep(1)

    except Exception as e:
        print(f"An error occurred in stack check: {e}")
        make_csv.print("error", f"An error occurred in stack check: {e}")







def main():
    
    CameraStart = False

    # 温湿度気圧のセットアップ
    try:
        bus = smbus.SMBus(1)
        bme = BME280Sensor(bus_number=1)

        # 初めは異常値が出てくるので，空測定
        for i in range(10):
            try:
                bme.read_data()
            except Exception as e:
                print(f"An error occurred during empty measurement in BME: {e}")
                make_csv.print('msg', f"An error occurred during empty measurement in BME: {e}")

        data = bme.read_data()  # ここでデータを取得
        pressure = bme.compensate_P(data)  # 気圧を補正して取得
        make_csv.print("alt_base_press", pressure)
        baseline = bme.baseline(pressure)

    except Exception as e:
        print(f"An error occurred in setting bme object: {e}")
        make_csv.print('serious_error', f"An error occurred in setting bme280 object: {e}")

    # 9軸のセットアップ
    try:
        bno = BNO055()
        if not bno.begin():
            print("Error initializing device")
            make_csv.print("Error initializing device")
        time.sleep(1)
        bno.setExternalCrystalUse(True)

    except Exception as e:
        print(f"An error occurred in setting bno055: {e}")
        make_csv,print('serious_error',f"An error occurred in setting bno055: {e}")


    ######################################################
    #　モータードライバのセットアップ
    ################################################


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



    phase = 1  # フェーズ0から開始

    ready = False




    try:
        print("セットアップ完了")
        make_csv.print("msg","セットアップ完了")
        make_csv.print("phase",0)
            
        # ここから無限ループ
        while True:

            # ************************************************** #
            #             待機フェーズ(phase = 0)                #
            # ************************************************** #
            if phase == 0:
                try:
                    data = bme.read_data()
                    pressure = bme.compensate_P(data)
                    time.sleep(1.0)
                    alt_1 = bme.altitude(pressure, qnh=baseline)

                    #linear_accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)
                    #accel_x, accel_y, accel_z = linear_accel
                    #print(f"accel_z: {accel_z}")
                    #make_csv.print("msg",f"accel_z: {accel_z}")
                    time.sleep(0.5)

                    #ドローンで上げきっているとき
                    if ready:
                        if alt_1 <= 10:
                            phase = 1
                            print("Go to falling phase")
                            make_csv.print("msg","Go to falling phase")
                            make_csv.print("phase",1)

                    #まだドローンで上に上げきっていないとき
                    else:

                        if alt_1 >= 15:
                            ready = True
                            time.sleep(15)

                    time.sleep(0.5)




                except Exception as e:
                    print(f"An error occurred in phase0 : {e}")
                    make_csv.print("error",f"An error occurred in phase0 : {e}")

            # ************************************************** #
            #             落下フェーズ(phase = 1)                #
            # ************************************************** #
            elif phase == 1:
                try:
                    consecutive_count = 0  # 連続成功回数をカウント

                    for _ in range(10):  # 最大試行回数
                        data = bme.read_data()
                        pressure = bme.compensate_P(data)
                        alt_2 = bme.altitude(pressure, qnh=baseline)

                        linear_accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)
                        accel_x, accel_y, accel_z = linear_accel

                        if abs(accel_x) + abs(accel_y) + abs(accel_z) < 0.5 and alt_2 <= 1:
                            consecutive_count += 1
                            print(f"落下終了の条件を満たしました: {consecutive_count}/5")
                            make_csv.print("msg",f"落下終了の条件を満たしました: {consecutive_count}/5")
                            time.sleep(1)
                        else:
                            consecutive_count = 0  # 条件が崩れたらリセット
                            print(f"落下終了の条件を満たしませんでしたｗｗｗｗ")
                            time.sleep(0.5)

                        if consecutive_count >= 5:

                            make_csv.print("msg","ニクロム線切断開始")
                            print("ニクロム線切断開始")

                            #ニクロム線切断
                            pin = 16
                            '''
                            GPIO.setmode(GPIO.BCM)
                            GPIO.setup(pin, GPIO.OUT)
                            GPIO.output(pin, 1)
                            time.sleep(5)
                            GPIO.output(pin, 0)
                            '''
                            make_csv.print("msg","ニクロム線切断完了")
                            print("ニクロム線切断完了")


                            phase = 2

                            print("フェーズ2に移行")
                            make_csv.print("msg","フェーズ2に移行")
                            make_csv.print("phase",2)

                            time.sleep(1)


                            # 初期位置の緯度経度を取得
                            start_lat = get_latitude()
                            start_lon = get_longitude()
                            print("aaaa")

                            # 移動していない判定のカウンター
                            no_movement_count = 0
                            #遠距離フェーズ最初の5秒前進を実行
                            accel(motor_right,motor_left)
                            print("uuuuu")
                            time.sleep(5)
                            stop()
                            print("haaaaaaaa")

                            #5秒進んだ先での現在位置を得る
                            current_lat = get_latitude()
                            current_lon = get_longitude()
                            print(current_lat, current_lon)  # 現在位置


                            break

                except Exception as e:
                    print(f"An error occurred in phase1: {e}")
                    make_csv.print("error",f"An error occurred in phase1 : {e}")

            # ************************************************** #
            #             遠距離フェーズ(phase = 2)              #
            # ************************************************** #
            elif phase == 2:

              try:

                # 距離と角度を計算し、表示
                distance_to_goal, angle_to_goal = calculate_distance_and_angle(current_lat, current_lon, start_lat, start_lon)
                print("現在地からゴール地点までの距離:", distance_to_goal, "メートル")
                print("angle_to_goal°:", str(angle_to_goal * 180 / math.pi) + "°")
                make_csv.print("distance_to_goal", distance_to_goal)
                make_csv.print("angle_to_goal", angle_to_goal)

                # 前回の現在地を保存
                start_lat = current_lat
                start_lon = current_lon

                # 進行方向を決定
                if angle_to_goal > 0:
                    print("進行方向に対して左方向にゴールがあります")
                    # ゴールへの角度に比例した時間だけ左回転
                    rotation_time = angle_to_goal / omega  # 回転時間 = 角度 / 回転速度
                    # 左に回転する処理をここに記述 (例: motor(-0.5, 0.5))
                    leftturn(motor_right,motor_left)
                    time.sleep(rotation_time)
                    stop()
                    
                    # 5秒前進
                    accel(motor_right,motor_left)
                    time.sleep(2)

                else:
                    print("進行方向に対して右方向にゴールがあります")
                    # ゴールへの角度に比例した時間だけ右回転
                    rotation_time = abs(angle_to_goal) / omega  # 回転時間 = 角度 / 回転速度
                    # 右に回転する処理をここに記述 (例: motor(0.5, -0.5))
                    rightturn(motor_right,motor_left)
                    time.sleep(rotation_time)
                    stop()
                    time.sleep(1)

                    # 5秒前進
                    accel(motor_right,motor_left)
                    time.sleep(2)

                #スタック検知
                is_stacking = 1
                for i in range(5):
                    Gyro = bno.getVector(BNO055.VECTOR_GYROSCOPE)
                    gyro_xyz = abs(Gyro[0]) + abs(Gyro[1]) + abs(Gyro[2])
                    is_stacking = is_stacking and (gyro_xyz < 0.75)
                    time.sleep(0.2)
                if is_stacking:
                    #スタック検知がyesの場合
                    time.sleep(1)
                    for i in range(1, 5 + 1):
                        GPIO.output(17, 1)
                        time.sleep(0.5)
                        GPIO.output(17, 0)
                        time.sleep(0.5)
                    retreat(motor_right,motor_left)
                    time.sleep(3)
                    rightturn(motor_right,motor_left)
                    time.sleep(1)
                    accel(motor_right,motor_left)
                    time.sleep(2)#ここにスタックしたときの処理
                else:
                    #スタック検知できなかったら
                    time.sleep(2) # l165 + l169 = 3s ∴あと2s
                    #３秒動かすコード
                    accel(motor_right, motor_left)  # ここで動かす処理を追加
                    time.sleep(3) 

                # ... (stop motor) ...
                stop()
                time.sleep(1)
                #モーター止める

                    # 機体がひっくり返ってたら回る
                try:
                    accel_start_time = time.time()
                    if 0 < bno.getVector(BNO055.VECTOR_GRAVITY)[2]:
                        while 0 < bno.getVector(BNO055.VECTOR_GRAVITY)[2] and time.time()-accel_start_time < 5:
                            print('muki_hantai')
                            make_csv.print('warning', 'muki_hantai')
                            accel(motor_right, motor_left)
                            time.sleep(0.5)
                    else:
                        if time.time()-accel_start_time >= 5:
                        # 5秒以内に元の向きに戻らなかった場合
                            stop()
                            time.sleep(1)
                            rightturn(motor_right, motor_left)
                            time.sleep(1)
                            leftturn(motor_right, motor_left)
                            time.sleep(1)
                            stop()
                            
                            continue
                        else:
                            print('muki_seizyou')
                            make_csv.print('msg', 'muki_seizyou')
                            stop()
                            time.sleep(1)
                except Exception as e:
                    print(f"An error occured while changing the orientation: {e}")
                    make_csv.print('error', f"An error occured while changing the orientation: {e}")





                # 現在地を更新
                current_lat = get_latitude()
                current_lon = get_longitude()

                # ゴールの10 m以内に到達したらループを抜け近距離フェーズへ
                if distance_to_goal <= 10:
                    print("近距離フェーズに移行")
                    phase = 3
                    make_csv.print("phase",3)
              except Exception as e:
                print(f"An error occurred in phase1: {e}")
                make_csv.print("error",f"An error occurred in phase1 : {e}")


            # ************************************************** #
            #             近距離フェーズ(phase = 3)              #
            # ************************************************** #
            elif phase == 3:
                try:


                    


                        # フレームを取得

                        if (phase == 3 and CameraStart == False):
                            
                            try:
                                picam2 = Picamera2()
                                config = picam2.create_preview_configuration({"format": 'XRGB8888', "size": (320, 240)})
                                picam2.configure(config)

                            except Exception as e:
                                print(f"An error occurred in init camera: {e}")

                            picam2.start()
                            CameraStart = True
                            time.sleep(0.1)


                        if (CameraStart == True):

                            frame = picam2.capture_array()


                            # 赤色を検出
                            mask = red_detect(frame)

                            camera_order = analyze_red(frame, mask)
                            
                            

                            if camera_order == 1:
                                accel(motor_right,motor_left)
                                time.sleep(2)
                                stop()

            
                            elif camera_order == 2:
                                rightturn(motor_right,motor_left)
                                time.sleep(0.1)
                                stop()

                            elif camera_order == 3:
                                leftturn(motor_right,motor_left)
                                time.sleep(0.1)
                                stop()

                            check_stuck()





                            # 面積のもっとも大きい領域を表示
                            # 結果表示
                            # cv2.putText(frame, "o", (frame.shape[1] // 2 ,frame.shape[1] // 2 ), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3)
                            cv2.imshow("Frame", frame)
                            # cv2.imshow("Mask", mask)
                            cv2.waitKey(1)
                            time.sleep(0.1) # フレーム再取得までの時間

                            if camera_order == 0:
                                print("Goal Goal Goal")
                                phase = 4
                                make_csv.print("phase",4)
                                # カメラを終了
                                picam2.close()
                                stop()





                except Exception as e:
                    print(f"An error occurred in phase3 : {e}")


            # ************************************************** #
            #             ゴールフェーズ(phase = 4)              #
            # ************************************************** #
            elif phase == 4:
                try:
                    print("Goal reached! LED on.")
                    #LED点灯
                    GPIO.output(5, 1)
                    make_csv.print("msg","Goal reached! LED on.")
                    time.sleep(10)
                except Exception as e:
                    print(f"An error occurred in phase5 : {e}")

    except Exception as e:
        print(f"An error occurred in setting : {e}")

# メイン関数の呼び出し
if __name__ == "__main__":
    main()



