#実行するスクリプトがあるフォルダと同じフォルダにmake_csv.pyをコピーし
#下の行のように「make_csv」をimportする
import make_csv

def main():
    make_csv.print("msg","ここに保存したいデータ")#この文のように書くことでcsvファイルに保存される
    #第一引数(上の文でいう「"msg"」の部分)に保存したいデータのタイプ(タイプの指定は後述※1)
    #基本的にはここで選んだタイプと同じ名前の列にデータが保存されます
    #第二引数(上の文でいう「"ここに保存したいデータ"」)に保存したいデータを入力する

    #※1
    #第一引数に関してだが基本は下のやつから選んでほしい
    #先輩のコード見て自分が記録しようとしてることがなんて名前のタイプ使ってたかを確認すると良い
    msg_types = ['time', 'file', 'func', 'line', 'serious_error', 'error', 'warning', 'msg', 'format_exception', 'phase', 'gnss_time', 'date', 'lat', 'lon', 'alt', 'alt_base_press', 'goal_lat', 'goal_lon', 'temp', 'press', 'camera_area', 'camera_order', 'camera_center_x', 'camera_center_y', 'camera_frame_size_x', 'camera_frame_size_y', 'motor_l', 'motor_r', 'goal_relative_x', 'goal_relative_y', 'goal_relative_angle_rad', 'goal_distance', 'accel_all_x', 'accel_all_y', 'accel_all_z', 'accel_line_x', 'accel_line_y', 'accel_line_z', 'mag_x', 'mag_y', 'mag_z', 'gyro_x', 'gyro_y', 'gyro_z', 'grav_x', 'grav_y', 'grav_z', 'euler_x', 'euler_y', 'euler_z', 'nmea']
    #先輩のコードを見てるとたまに上の行のやつにない奴が第一引数に入ってることがある(具体例は後述※2)
    #そのときは連絡ください

    #msg_typeを増やすこともできます
    #その時も連絡ください

    #以下書き方の具体例
    make_csv.print("msg","データタイプ「msg」は最も基本的なメッセージを記録したいとき使います")
    make_csv.print("msg","9軸セットアップ完了")
    make_csv.print("msg","上の行みたいにつかいましょう")

    make_csv.print("error","エラーメッセージを記録すんのに使う")
    make_csv.print("serious_error","深刻なエラーのとき使う，先輩はセンサーのセットアップ失敗したときに使ってた")

    #高度記録するやつ
    altitude = 12.54 #センサーで12.54mと取得できたとしよう
    make_csv.print("alt",altitude)

    #※2
    #上のmsg_typeにないやつが第一引数に入ってるパターン
    accel = [12,8,9.8] #加速度x軸方向が12m/s^2,y軸方向が8m/s^2,z軸方向が9.8m/s^2と取得できたとしよう
    make_csv.print("accel_all",accel)
    #これように同時に複数のデータいれるときは上のmsg_typeにないやつを使ってることがある


    #このコードを実行してできたやつをgithubにおいとくね

if __name__ == "__main__":
	main()