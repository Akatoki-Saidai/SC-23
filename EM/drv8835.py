import RPi.GPIO as GPIO  # GPIOモジュールをインポート

from gpiozero import Motor
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory

import time
import random
import numpy as np

# 制御量の出力用
import make_csv as csv

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

    # 正回転 -> 停止 -> 逆回転 -> 停止
    try:
        # 最高速で正回転 - 1秒
        print("最高速で正回転 - 1秒")
        motor_left.value = 1.0
        motor_right.value = 1.0
        sleep(1)
        # 少し遅く正回転 - 1秒
        print("少し遅く正回転 - 1秒")
        motor_left.value = 0.75
        motor_right.value = 0.75
        sleep(1)
        # 遅く正回転 - 2秒
        print("遅く正回転 - 1秒")
        motor_left.value = 0.5
        motor_right.value = 0.5
        sleep(1)
        # 停止 - 1秒
        motor_left.value = 0.0
        motor_right.value = 0.0
        sleep(1)
        # 最高速で逆回転 - 1秒
        print("最高速で逆回転 - 1秒")
        motor_left.value = -1.0
        motor_right.value = -1.0
        sleep(1)
        # 少し遅く逆回転 - 1秒
        print("少し遅く逆回転 - 1秒")
        motor_left.value = -0.75
        motor_right.value = -0.75
        sleep(1)
        # 遅く逆回転 - 2秒
        print("遅く逆回転 - 1秒")
        motor_left.value = -0.5
        motor_right.value = -0.5
        sleep(1)
        # 停止 - 1秒
        motor_left.value = 0.0
        motor_right.value = 0.0
        sleep(1)
        # 停止
        motor_left.value = 0.0
        motor_right.value = 0.0
    except KeyboardInterrupt:
        print("stop")
        # 停止
        motor_left.value = 0.0
        motor_right.value = 0.0

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


print("retreat")
retreat(motor_right,motor_left)
time.sleep(2)
stop()

print("accel")
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
