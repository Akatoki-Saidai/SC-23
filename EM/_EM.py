import time

from bno055 import BNO055

def main():


    #9軸のセットアップ
    try:
        bno = BNO055()
        if bno.begin() is not True:
            print("Error initializing device")
        time.sleep(1)
        bno.setExternalCrystalUse(True)
    except Exception as e:
        print(f"An error occurred in setting bno055: {e}")

    phase = 0 #フェーズ0から開始

    try:
        #ここからずっと繰り返し
        while True:
            # ************************************************** #
            #             待機フェーズ(phase = 0)                #
            # ************************************************** #
            if(phase==0):

                try:


                    linear_accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)
                    accel_x, accel_y, accel_z = linear_accel

                    #落下検知の要件に高度が10m以上上昇したか？を追加予定
                    if(accel_z < -5.0):
                        phase = 1 #下向き加速度が5.0m/s^2を超えたら落下検知

                except Exception as e:
                    print(f" An error occurred in phase0 : {e}")

            # ************************************************** #
            #             落下フェーズ(phase = 1)                #
            # ************************************************** #

            if(phase==1):
                try:
                    linear_accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)
                    accel_x, accel_y, accel_z = linear_accel

                    #落下終了検知の要件に高度が基準高度であるか？加速度変化がないか？を追加予定
                    if(accel_z > -0.5): #下向き加速度が0.5m/s^2以下だったらフェーズ2に移行
                        phase = 2
                except Exception as e:
                    print(f" An error occurred in phase1 : {e}")

            # ************************************************** #
            #             遠距離フェーズ(phase = 2)                #
            # ************************************************** #
            if(phase==2):
                try:
                    if(distance <= 10): #目標までの距離が10mを切ったらフェーズ3に移行
                        phase = 3
                except Exception as e:
                    print(f" An error occurred in phase2 : {e}")


            # ************************************************** #
            #             近距離フェーズⅠ(phase = 3)              #
            # ************************************************** #

            if(phase==3):
                try:
                    if(red >= 30): #赤色が30%以上しめていたらフェーズ4に移行
                        phase = 4
                except Exception as e:
                    print(f" An error occurred in phase3 : {e}")

            # ************************************************** #
            #             近距離フェーズⅡ(phase = 4)              #
            # ************************************************** #

            if(phase==4):
                try:
                    if(red >= 80): #赤色が80%以上しめていたらフェーズ5に移行
                        phase = 5
                except Exception as e:
                    print(f" An error occurred in phase4 : {e}")


            # ************************************************** #
            #             ゴールフェーズ(phase = 5)                #
            # ************************************************** #


            if(phase==5):
                try:
                    #LEDを点灯
                except Exception as e:
                    print(f" An error occurred in phase5 : {e}")



    except Exception as e:
        print(f"An error occurred in setting : {e}")



###################################################################
#以下，いつか消すコード
#コードの置き場
#自作関数を他のスクリプトで作った人はとりあえずこのスクリプトで呼び出す方法をここにメモして
##########################################################

    #9軸の値を取得してprint(サンプルコード)
    try:
        while True:
            euler = bno.getVector(BNO055.VECTOR_EULER)
            print("オイラー角:", euler)
            print("加速度:",bno.getVector(BNO055.VECTOR_LINEARACCEL))
            print("加速度:",bno.getVector(BNO055.VECTOR_ACCELEROMETER))
            print("磁力計:",bno.getVector(BNO055.VECTOR_MAGNETOMETER))
            print("ジャイロ:",bno.getVector(BNO055.VECTOR_GYROSCOPE))
            print("重力:",bno.getVector(BNO055.VECTOR_GRAVITY))

    except Exception as e:
         print(f"An error occurred in print bno055 date: {e}")
    return

############################################
#ここより下は消さない
#################################

# メイン関数
# 備考:main()に投げるだけ
if __name__ == "__main__":
	main()