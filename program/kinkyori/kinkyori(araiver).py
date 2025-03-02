while area<10000:#カメラでゴール判定まで持ってく
    current_lat = get_latitude()
    current_lon = get_longitude()
    print(current_lat, current_lon)  # 現在位置

    # 距離と角度を計算し、表示
    distance_to_goal, angle_to_goal = calculate_distance_and_angle(current_lat, current_lon, previous_lat, previous_lon)
    print("現在地からゴール地点までの距離:", distance_to_goal, "メートル")
    print("theta_for_goal°:", str(angle_to_goal * 180 / math.pi) + "°")
    while True:#スタック検知のところまでこのwhileで持ってく。。
        if -math.pi/3<=angle_to_goal<=math.pi/3:
            drv8835.accel(motor_right,motor_left)
            time.sleep(3)
            drv8835.stop()
        elif -math.pi/3>angle_to_goal:
            drv8835.rightturn(motor_right,motor_left)
            time.sleep(rotation_time)  # 計算された時間だけ回転
            drv8835.stop()
            time.sleep(1)
            drv8835.accel(motor_right,motor_left)
            time.sleep(3)
            drv8835.stop()
        else:
            drv8835.leftturn(motor_right,motor_left)
            time.sleep(rotation_time)  # 計算された時間だけ回転
            drv8835.stop()
            time.sleep(1)
            drv8835.accel(motor_right,motor_left)
            time.sleep(3)
            drv8835.stop()

    

    #スタック検知
        is_stacking = 1
        for i in range(5):
            Gyro = BNO055.getVector(BNO055.VECTOR_GYROSCOPE)
            gyro_xyz = abs(Gyro[0]) + abs(Gyro[1]) + abs(Gyro[2])
            is_stacking = is_stacking and (gyro_xyz < 0.75)
            time.sleep(0.2)
        if is_stacking:
            #スタック検知がyesの場合
            drv8835.retreat(motor_right,motor_left)
            time.sleep(3)
            drv8835.rightturn(motor_right,motor_left)
            time.sleep(1)
            drv8835.accel(motor_right,motor_left)
            time.sleep(2)#ここにスタックしたときの処理
        else:
        #スタック検知できなかったら

        #あと3秒動かすコードをここに書く


    # ... (stop motor) ...
    #モーター止める

        # 機体がひっくり返ってたら回る
        try:
            accel_start_time = time.time()
            if 0 < BNO055.getVector(BNO055.VECTOR_GRAVITY)[2]:
                while 0 < BNO055.getVector(BNO055.VECTOR_GRAVITY)[2] and time.time()-accel_start_time < 5:
                    print('muki_hantai')
                    make_csv.print('warning', 'muki_hantai')
                    drv8835.accel(motor_right, motor_left)
                    time.sleep(0.5)
                    drv8835.stop()
            else:
                if time.time()-accel_start_time >= 5:
            # 5秒以内に元の向きに戻らなかった場合
                    drv8835.rightturn(motor_right, motor_left)
                    drv8835.leftturn(motor_right, motor_left)
                    continue
                else:
                    print('muki_naotta')
                    make_csv.print('msg', 'muki_naotta')
                    drv8835.brake(motor_right, motor_left)
                    drv8835.stop()
        except Exception as e:
            print(f"An error occured while changing the orientation: {e}")
            make_csv.print('error', f"An error occured while changing the orientation: {e}")
    else:
        break

if area>=10000:
    break
