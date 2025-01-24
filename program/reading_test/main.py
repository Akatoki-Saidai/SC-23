from sensor import reading_sensor
#from 「関数があるスクリプト名」 import 「関数があるクラス名」

def main():
    try:
        a = reading_sensor.measurement()
        #「関数があるクラス」.「関数名」で実行したい関数を実行
        print (a)
    except Exception as e:
        print(f"An error occurred in print: {e}")

# メイン関数
# 備考:main()に投げるだけ
if __name__ == "__main__":
	main()