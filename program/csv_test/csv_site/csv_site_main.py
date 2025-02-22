import csv
import os
from csv_site_ import csv_f

d1 = {'a': 1, 'b': 2, 'c': 3}
d2 = {'a': 20, 'c': 30}

def main():
   file_path = os.path.join(os.getcwd(), 'program/csv/csv_site/test.csv') 
   with open(file_path, 'w') as f:
      #writer = csv.writer(f)
      writer = csv.DictWriter(f, ['a', 'b', 'c'])

   csv_f.printf("Log",d1,writer)
   csv_f.printf("Log",d2,writer)

   return


# メイン関数
# 備考:main()に投げるだけ
if __name__ == "__main__":
	main()

