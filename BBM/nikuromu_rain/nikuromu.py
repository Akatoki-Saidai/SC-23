import time
import RPi.GPIO as GPIO
#使うpin番号
pin = 16
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.OUT)
GPIO.output(pin,1)
#休ませる時間
time.sleep(3)
GPIO.output(pin,0)
