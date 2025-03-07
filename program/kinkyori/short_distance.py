import cv2
import numpy as np
from picamera2 import Picamera2
import time

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
        # csv.print('serious_error', f"An error occured in setting motor_driver: {e}")
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
    # csv.print('serious_error', f"An error occured in setting motor_driver: {e}")
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
    # csv.print('motor', [0, 0])
    power = 0
    for i in range(int(1 / delta_power)):
        if 0<=power<=1:
                right.value = power
        left.value = power
        power += delta_power

    right.value = -1
    left.value = -0.7

    # csv.print('motor', [-1, -0.7])
    # csv.print('msg', 'motor: accel')

# ブレーキ関数
def brake(right, left):
    power_r = float(right.value)
    power_l = float(left.value)

    # csv.print('motor', [power_r, power_l])

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
    # csv.print('motor', [0, 0])
    # csv.print('msg', 'motor: brake')

# 左旋回
def leftturn(right, left):
    
    right.value = 0
    left.value = 0
    # csv.print('motor', [0, 0])
    power = 0
    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            right.value = power
            left.value = -1 * power
        
        power += delta_power

    power = 1
    right.value = -0.5
    left.value = 0.5
    # csv.print('motor', [-0.5, 0.5])

    


# 右旋回
def rightturn(right, left):
    
    right.value = 0
    left.value = 0
    # csv.print('motor', [0, 0])
    power = 0
    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            right.value = -1 * power
            left.value = power
        
        power += delta_power

    power = 1
    right.value = 0.5
    left.value = -0.5
    # csv.print('motor', [0.5, -0.5])

   
    



def rightonly(right, left):
    
    right.value = 0
    left.value = 0
    # csv.print('motor', [0, 0])

    power = 0
    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            right.value = power

        power += delta_power

    power = 1
    right.value = 1
    # csv.print('motor_r', 1)

    time.sleep(0.1)

    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            right.value = power
            
        power -= delta_power

    right.value = 0
    # csv.print('motor_r', 0)




def leftonly(right, left):
    
    right.value = 0
    left.value = 0
    # csv.print('motor', [0, 0])
    power = 0

    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            left.value = power
        
        power += delta_power

    power = 1
    left.value = 1
    # csv.print('motor_l', 1)

    time.sleep(0.1)

    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            left.value = power
        
        power -= delta_power
        
    left.value = 0
    # csv.print('motor_l', 0)


# 指定した角度だけ右に曲がる
def right_angle(bno, angle_deg, right, left):
    # csv.print('msg', f'motor: turn {angle_deg} deg to right')
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
    # csv.print('motor', [left.value, right.value])

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
                # csv.print('warning', 'Starts orientation correction in right_angle')
                accel(right, left)
                time.sleep(0.5)
                brake(right, left)
                # csv.print('msg', 'Finish correcting the orientation in right_angle')
        except Exception as e:
            print(f'An error occured in right_angle: {e}')
            # csv.print('error', f'An error occured in right_angle: {e}')
    else:
        # スタックしてます
        print('stacking now! in right_angle')
        # csv.print('warning', 'stacking now! in right_angle')

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
    # csv.print('motor', [left.value, right.value])

# 指定した角度だけ左に曲がる
def left_angle(bno, angle_deg, right, left):
    # csv.print('msg', f'motor: turn {angle_deg} deg to left')
    angle_rad = angle_deg*np.pi/180
    start_time = time.time()
    prev_time = time.time()
    rot_angle = 0
    # csv.print('motor', [left.value, right.value])

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
                # csv.print('warning', 'Starts orientation correction in left_angle')
                accel(right, left)
                time.sleep(0.5)
                brake(right, left)
                # csv.print('msg', 'Finish correcting the orientation in left_angle')
        except Exception as e:
            print(f'An error occured in left_angle: {e}')
            # csv.print('error', f'An error occured in left_angle: {e}')
    else:
        # スタックしてます
        print('stacking now! in left_angle')
        # csv.print('warning', 'stacking now! in left_angle')

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
    # csv.print('motor', [left.value, right.value])


#ここからは未知(2025年2月22日)
def retreat(right, left):
    # csv.print('motor', [0, 0])
    power = 0
    for i in range(int(1 / delta_power)):
        if 0<=power<=1:
                right.value = -power
        left.value = -power
        power += delta_power

    right.value = 1
    left.value = 1

    # csv.print('motor', [-1, -1])
    # csv.print('msg', 'motor: accel')

def stop():
    motor_left.value = 0.0
    motor_right.value = 0.0
    time.sleep(1)

# なんかフェーズの変数(近距離フェーズ)
phase = 2
CameraStart = False

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
                print("赤色物体は画像の中心にあります。")#直進
                camera_order = 1
                
            elif center_x > frame_center_x + 50:
                print("赤色物体は画像の右側にあります。")#右へ
                camera_order = 2
                stop()
                
            elif center_x < frame_center_x - 50:
                print("赤色物体は画像の左側にあります。")#左へ
                camera_order = 3
    else:
        print("何もないです未検出")
        camera_order = 4

        # red_result = cv2.drawContours(mask, [biggest_contour], -1, (0, 255, 0), 2)

    return camera_order
    
try:
    picam2 = Picamera2()
    config = picam2.create_preview_configuration({"format": 'XRGB8888', "size": (320, 240)})
    picam2.configure(config)

except Exception as e:
    print(f"An error occurred in init camera: {e}")


if picam2 is None:
    print("Failed to initialize the camera. Exiting...")
else:

    while True:
        # フレームを取得

        if (phase == 2 and CameraStart == False):

            picam2.start()
            CameraStart = True
        
        if (CameraStart == True):
        
            frame = picam2.capture_array()

            # 赤色を検出
            mask = red_detect(frame)

            camera_order = analyze_red(frame, mask)

            if camera_order == 1:
                accel(motor_right,motor_left)
                time.sleep(1)
                stop()
            
            elif camera_order == 2:
                rightturn(motor_right,motor_left)
                time.sleep(0.2)
                stop()

            elif camera_order == 3:
                leftturn(motor_right,motor_left)
                time.sleep(0.2)
                stop()

            elif camera_order == 4:
                rightturn(right, left)
                time.sleep(0.5)
                stop()

            check_stuck()

            # 面積のもっとも大きい領域を表示
            # 結果表示
            # cv2.putText(frame, "o", (frame.shape[1] // 2 ,frame.shape[1] // 2 ), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3)
            cv2.imshow("Frame", frame)
            # cv2.imshow("Mask", mask)
            time.sleep(0.1) # フレーム再取得までの時間

            # qキーを押すと終了(手動停止)
            #なんか反応しないときある
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break

        if (camera_order == 0):
            print("Goal Goal Goal")
            break

# カメラを終了
picam2.close()
stop()

#LED点灯
GPIO.output(5, 1)
time.sleep(23)

# ウィンドウを閉じる
cv2.destroyAllWindows()
