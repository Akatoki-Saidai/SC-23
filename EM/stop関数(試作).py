from gpiozero import Motor
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
import RPi.GPIO as GPIO

# DCモータのピン設定
PIN_AIN1 = 12
PIN_AIN2 = 26
PIN_BIN1 = 33
PIN_BIN2 = 18

dcm_pins = {
    "left_forward": PIN_AIN2,
    "left_backward": PIN_AIN1,
    "right_forward": PIN_BIN2,
    "right_backward": PIN_BIN1,
}

def stop_motor(motor_left, motor_right):
    """モーターを完全に停止する関数"""
    motor_left.value = 0.0
    motor_right.value = 0.0
    GPIO.output(PIN_AIN1, GPIO.LOW)
    GPIO.output(PIN_AIN2, GPIO.LOW)
    GPIO.output(PIN_BIN1, GPIO.LOW)
    GPIO.output(PIN_BIN2, GPIO.LOW)

def motor_operation(motor_left, motor_right):
    """モーター操作関数"""
    # モーターに対する指示がある場合、実行する
    # 例えば、モーターを前進させる指示があればその処理を行う
    print("モーターを前進させます。")
    motor_left.value = 1.0
    motor_right.value = 1.0
    sleep(2)  # 2秒間前進
    stop_motor(motor_left, motor_right)

    print("モーターを後退させます。")
    motor_left.value = -1.0
    motor_right.value = -1.0
    sleep(2)  # 2秒間後退
    stop_motor(motor_left, motor_right)

    print("モーターを停止します。")
    stop_motor(motor_left, motor_right)

def main():
    # 初期化
    factory = PiGPIOFactory()
    motor_left = Motor(forward=dcm_pins["left_forward"],
                       backward=dcm_pins["left_backward"],
                       pin_factory=factory)
    motor_right = Motor(forward=dcm_pins["right_forward"],
                        backward=dcm_pins["right_backward"],
                        pin_factory=factory)

    try:
        # 初期停止
        stop_motor(motor_left, motor_right)

        # モーター操作指示を実行
        while True:
            # 次のモーター操作指示がある場合にその処理を行う
            motor_operation(motor_left, motor_right)

    except KeyboardInterrupt:
        print("プログラム終了")
        stop_motor(motor_left, motor_right)
    finally:
        print("GPIOリソースを解放")
        factory.close()  # factory.close()でリソース解放
        GPIO.cleanup()  # GPIOの状態をリセット

if __name__ == "__main__":
    main()
