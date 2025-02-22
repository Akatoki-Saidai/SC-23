import csv
import os
from make_csv import csv_f

def main():

    #csvファイルの1行目を記述
    #1行目にはまだデータの項目を追加予定
    line = ["time","Log","accel_x","accel_y","accel_z"]
    file_path = os.path.join(os.getcwd(), 'program/csv/test.csv')  # カレントディレクトリに保存
    with open(file_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(line)

    csv_f.printf("Log","csvプリントテスト")
    csv_f.printf("Log","csvプリントテスト2")
    return


# メイン関数
# 備考:main()に投げるだけ
if __name__ == "__main__":
	main()
