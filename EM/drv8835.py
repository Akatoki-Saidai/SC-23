import time
from gpiozero import Motor
from gpiozero.pins.pigpio import PiGPIOFactory
import random
import numpy as np

# 制御量の出力用
import csv_print as csv

from bno055 import BNO055

delta_power = 0.20

def setup(AIN1, AIN2, BIN1, BIN2):

    dcm_pins = {
                "left_forward": BIN2,
                "left_backward": BIN1,
                "right_forward": AIN1,
                "right_backward": AIN2,
            }

    factory = PiGPIOFactory()
    left = Motor( forward=dcm_pins["left_forward"],
                        backward=dcm_pins["left_backward"],
                        pin_factory=factory)
    right = Motor( forward=dcm_pins["right_forward"],
                        backward=dcm_pins["right_backward"],
                        pin_factory=factory)
    
    return right, left


def accel(right, left):#加速
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


def brake(right, left):#減速
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


def leftturn(right, left):#左に旋回
    
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

    time.sleep(0.5 + random.random()/2)

    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            right.value = power
            left.value = -1 * power
        
        power -= delta_power

    right.value = 0
    left.value = 0
    csv.print('motor', [0, 0])
    csv.print('msg', 'motor: leftturn')




def rightturn(right, left):#右に旋回
    
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

    time.sleep(0.5 + random.random()/2)

    for i in range(int(1 / delta_power)):
        if (-1 <= power <= 1):
            right.value = -1 * power
            left.value = power
        
        power -= delta_power
            
    right.value = 0
    left.value = 0
    csv.print('motor', [0, 0])
    csv.print('msg', 'motor: rightturn')

    



def rightonly(right, left):#右の車輪のみ回す()
    
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




def leftonly(right, left):#左の車輪のみ回す
    
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
