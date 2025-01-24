from sensor import reading_sensor

def main():
    try:
        read = reading_sensor #クラスの名前が長すぎるときはあだ名をつけることもできる
        print (read.measurement())
        print (read.plus(2,3))
        print (read.greeting())
    except Exception as e:
        print(f"An error occurred in print: {e}")


# メイン関数
# 備考:main()に投げるだけ
if __name__ == "__main__":
	main()