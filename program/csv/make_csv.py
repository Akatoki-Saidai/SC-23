import csv
import pprint

with open('data/temp/sample_writer_row.csv', 'w') as f:
    writer = csv.writer(f)
    writer.writerow([0, 1, 2])
    writer.writerow(['a', 'b', 'c'])