# 起動時に実行するプログラムコード

import time
import datetime

print("Raspberry Pi 4b is loanched")
time.sleep(3)

dt_now = datetime.datetime.now()
print(dt_now) # 今の年月日、時刻を表示
print(type(dt_now))
# <class 'datetime.datetime'>
