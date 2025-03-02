import cv2
import numpy as np
from picamera2 import Picamera2

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
                accel(motor_right,motor_left)
                print("赤色物体は画像の中心にあります。")#直進
                camera_order = 1
                
            elif center_x > frame_center_x + 50:
                rightturn(motor_right,motor_left)
                print("赤色物体は画像の右側にあります。")#右へ
                camera_order = 2
                stop()
                
            elif center_x < frame_center_x - 50:
                leftturn(motor_right,motor_left)
                print("赤色物体は画像の左側にあります。")#左へ
                camera_order = 3
                stop()

    else:
        print("何もないです未検出")
        accel(motor_right, motor_left)

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
            time.sleep(0.1) # フレーム再取得までの時間
        
            #こっからガチのテスト用
            #print(len(contours))

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

'''
#################################################################

while area<10000:#カメラでゴール判定まで持ってく
    #スタック検知
        is_stacking = 1
        for i in range(5):
            Gyro = BNO055.getVector(BNO055.VECTOR_GYROSCOPE)
            gyro_xyz = abs(Gyro[0]) + abs(Gyro[1]) + abs(Gyro[2])
            is_stacking = is_stacking and (gyro_xyz < 0.75)
            time.sleep(0.2)
        if is_stacking:
            #スタック検知がyesの場合
            drv8835.retreat(motor_right,motor_left)
            time.sleep(3)
            drv8835.rightturn(motor_right,motor_left)
            time.sleep(1)
            drv8835.accel(motor_right,motor_left)
            time.sleep(2)#ここにスタックしたときの処理
        else:
        #スタック検知できなかったら

        #あと3秒動かすコードをここに書く


    # ... (stop motor) ...
    #モーター止める

        # 機体がひっくり返ってたら回る
        try:
            accel_start_time = time.time()
            if 0 < BNO055.getVector(BNO055.VECTOR_GRAVITY)[2]:
                while 0 < BNO055.getVector(BNO055.VECTOR_GRAVITY)[2] and time.time()-accel_start_time < 5:
                    print('muki_hantai')
                    make_csv.print('warning', 'muki_hantai')
                    drv8835.accel(motor_right, motor_left)
                    time.sleep(0.5)
                    drv8835.stop()
            else:
                if time.time()-accel_start_time >= 5:
            # 5秒以内に元の向きに戻らなかった場合
                    drv8835.rightturn(motor_right, motor_left)
                    drv8835.leftturn(motor_right, motor_left)
                    continue
                else:
                    print('muki_naotta')
                    make_csv.print('msg', 'muki_naotta')
                    drv8835.brake(motor_right, motor_left)
                    drv8835.stop()
        except Exception as e:
            print(f"An error occured while changing the orientation: {e}")
            make_csv.print('error', f"An error occured while changing the orientation: {e}")
    else:
        break

if area>=10000:
    break
'''
