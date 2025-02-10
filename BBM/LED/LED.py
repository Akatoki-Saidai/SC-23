#https://zenn.dev/kotaproj/books/raspberrypi-tips/viewer/011_kiso_ledpwm

from gpiozero import PWMLED
from signal import pause
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep

# LEDのピン設定
PIN_LED1 = 21

def main():
    # LEDピン設定
    factory = PiGPIOFactory()
    led = PWMLED(PIN_LED1, pin_factory=factory)

    # だんだん明るくする
    # 25% -> 50% -> 75% -> 100%
    led.value = 0.25
    sleep(1)
    led.value = 0.50
    sleep(1)
    led.value = 0.75
    sleep(1)
    led.value = 1.0
    sleep(1)

    # 徐々に明るくし、徐々に暗くするを繰り返す
    try:
        led.pulse()
        pause()
    except KeyboardInterrupt:
        print("stop")
        led.off()


if __name__ == "__main__":
    main()