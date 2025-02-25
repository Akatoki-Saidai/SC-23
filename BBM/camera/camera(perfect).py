import cv2
import numpy as np
from picamera2 import Picamera2
import time


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

        # #最大の領域の中心座標を取得する
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
        if area > 10000:
            print("十分近い")
            camera_order = 0

        else:
            if frame_center_x -  50 <= center_x <= frame_center_x + 50:
                print("赤色物体は画像の中心にあります。")#直進
                camera_order = 1
            elif center_x > frame_center_x + 50:
                print("赤色物体は画像の右側にあります。")#右へ
                camera_order = 2
            elif center_x < frame_center_x - 50:
                print("赤色物体は画像の左側にあります。")#左へ
                camera_order = 3

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

        
            # 面積のもっとも大きい領域を表示
            # 結果表示
            # cv2.putText(frame, "o", (frame.shape[1] // 2 ,frame.shape[1] // 2 ), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3)
            cv2.imshow("Frame", frame)
            # cv2.imshow("Mask", mask)
            time.sleep(0.5) # フレーム再取得までの時間
        
            #こっからガチのテスト用
            #print(len(contours))

            # qキーを押すと終了(手動停止)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
    
        elif (camera_order == 0):
            print("Goal Goal Goal")
            break

# カメラを終了
picam2.close()

# ウィンドウを閉じる
cv2.destroyAllWindows()
