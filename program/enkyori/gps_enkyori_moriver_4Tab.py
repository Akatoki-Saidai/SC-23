            # ************************************************** #
            #             遠距離フェーズ(phase = 2)              #
            # ************************************************** #
            elif phase == 2:


                # 距離と角度を計算し、表示
                distance_to_goal, angle_to_goal = calculate_distance_and_angle(current_lat, current_lon, start_lat, start_lon)
                print("現在地からゴール地点までの距離:", distance_to_goal, "メートル")
                print("angle_to_goal°:", str(angle_to_goal * 180 / math.pi) + "°")
                make_csv.print("distance_to_goal", distance_to_goal)
                make_csv.print("angle_to_goal", angle_to_goal)

                # 前回の現在地を保存
                start_lat = current_lat
                start_lon = current_lon

                # 進行方向を決定
                if angle_to_goal > 0:
                    print("進行方向に対して左方向にゴールがあります")
                    # ゴールへの角度に比例した時間だけ左回転
                    rotation_time = angle_to_goal / omega  # 回転時間 = 角度 / 回転速度
                    # 左に回転する処理をここに記述 (例: motor(-0.5, 0.5))
                    leftturn(motor_right,motor_left)
                    time.sleep(rotation_time)
                    stop()
                    
                    # 5秒前進
                    accel(motor_right,motor_left)
                    time.sleep(2)

                else:
                    print("進行方向に対して右方向にゴールがあります")
                    # ゴールへの角度に比例した時間だけ右回転
                    rotation_time = abs(angle_to_goal) / omega  # 回転時間 = 角度 / 回転速度
                    # 右に回転する処理をここに記述 (例: motor(0.5, -0.5))
                    rightturn(motor_right,motor_left)
                    time.sleep(rotation_time)
                    stop()
                    time.sleep(1)

                    # 5秒前進
                    accel(motor_right,motor_left)
                    time.sleep(2)

                #スタック検知
                is_stacking = 1
                for i in range(5):
                    Gyro = bno.getVector(BNO055.VECTOR_GYROSCOPE)
                    gyro_xyz = abs(Gyro[0]) + abs(Gyro[1]) + abs(Gyro[2])
                    is_stacking = is_stacking and (gyro_xyz < 0.75)
                    time.sleep(0.2)
                if is_stacking:
                    #スタック検知がyesの場合
                    time.sleep(1)
                    for i in range(1, 5 + 1):
                        GPIO.output(17, 1)
                        time.sleep(0.5)
                        GPIO.output(17, 0)
                        time.sleep(0.5)
                    retreat(motor_right,motor_left)
                    time.sleep(3)
                    rightturn(motor_right,motor_left)
                    time.sleep(1)
                    accel(motor_right,motor_left)
                    time.sleep(2)#ここにスタックしたときの処理
                else:
                    #スタック検知できなかったら
                    time.sleep(2) # l165 + l169 = 3s ∴あと2s
                    #３秒動かすコード
                    accel(motor_right, motor_left)  # ここで動かす処理を追加
                    time.sleep(3) 

                # ... (stop motor) ...
                stop()
                time.sleep(1)
                #モーター止める

                    # 機体がひっくり返ってたら回る
                try:
                    accel_start_time = time.time()
                    if 0 < bno.getVector(BNO055.VECTOR_GRAVITY)[2]:
                        while 0 < bno.getVector(BNO055.VECTOR_GRAVITY)[2] and time.time()-accel_start_time < 5:
                            print('muki_hantai')
                            make_csv.print('warning', 'muki_hantai')
                            accel(motor_right, motor_left)
                            time.sleep(0.5)
                    else:
                        if time.time()-accel_start_time >= 5:
                        # 5秒以内に元の向きに戻らなかった場合
                            stop()
                            time.sleep(1)
                            rightturn(motor_right, motor_left)
                            time.sleep(1)
                            leftturn(motor_right, motor_left)
                            time.sleep(1)
                            stop()
                            
                            continue
                        else:
                            print('muki_seizyou')
                            make_csv.print('msg', 'muki_seizyou')
                            stop()
                            time.sleep(1)
                except Exception as e:
                    print(f"An error occured while changing the orientation: {e}")
                    make_csv.print('error', f"An error occured while changing the orientation: {e}")





                # 現在地を更新
                current_lat = get_latitude()
                current_lon = get_longitude()

                # ゴールの10 m以内に到達したらループを抜け近距離フェーズへ
                if distance_to_goal <= 10:
                    print("近距離フェーズに移行")
                    phase = 3
                    make_csv.print("phase",3)

