class reading_sensor:
#reading_sensorというクラスの中に関数を宣言する

    def measurement():
        try:
            a = 10 +5
            return a
        except Exception as e:
            print(f"An error occurred in measurement: {e}")

    def plus(b,c):

        try:
            a = b + c
            return a
        except Exception as e:
            print(f"An error occurred in measurement: {e}")

    def greeting():

        try:
            a = "Hello world"
            return a
        except Exception as e:
            print(f"An error occurred in measurement: {e}")