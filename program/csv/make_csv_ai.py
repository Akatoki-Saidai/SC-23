#動作確認済み

import csv
import os

def main():
    file_path = os.path.join(os.getcwd(), 'program/csv/test.csv')  # カレントディレクトリに保存
    with open(file_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([0, 1, 2])
        writer.writerow(['a', 'b', 'c'])

    with open('program/csv/test.csv', 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        for row in reader:
            print(row)

if __name__ == "__main__":
    main()