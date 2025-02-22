#最終版

import csv
import os


class csv_f:

    def printf(type,data):
        try:
            #if(type=="Log"):
            #if(type=="accel"):
            

            file_path = os.path.join(os.getcwd(), 'program/csv/test.csv')  # カレントディレクトリに保存
            with open(file_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(data+"\n")
        except Exception as e:
            print(f"An error occurred in make_csv: {e}")
