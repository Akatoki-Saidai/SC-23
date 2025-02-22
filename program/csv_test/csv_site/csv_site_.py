

import csv
import os



class csv_f:

    def printf(type,data):
        try:
            file_path = os.path.join(os.getcwd(), 'program/csv/csv_site/test.csv') 
            with open(file_path, 'w') as f:
                #writer = csv.writer(f)
                writer = csv.DictWriter(f, ['a', 'b', 'c'])
                writer.writeheader()
                writer.writerow(data)


        except Exception as e:
            print(f"An error occurred in make_csv: {e}")
