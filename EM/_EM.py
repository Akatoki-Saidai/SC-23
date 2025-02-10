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

    #9軸の値を取得してprint(サンプルコード)
    try:
        while True:
            euler = bno.getVector(BNO055.VECTOR_EULER)
            print("オイラー角:", euler)

            print("加速度:",bno.getVector(BNO055.VECTOR_ACCELEROMETER))
            print("磁力計:",bno.getVector(BNO055.VECTOR_MAGNETOMETER))
            print("ジャイロ:",bno.getVector(BNO055.VECTOR_GYROSCOPE))
            print("重力:",bno.getVector(BNO055.VECTOR_GRAVITY))

    except Exception as e:
         print(f"An error occurred in print bno055 date: {e}")
    return


# メイン関数
# 備考:main()に投げるだけ
if __name__ == "__main__":
	main()