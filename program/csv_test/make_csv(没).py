import csv
import pprint

def main():
    with open('test.csv', 'w') as f:
        writer = csv.writer(f)
        writer.writerow([0, 1, 2])
        writer.writerow(['a', 'b', 'c'])


if __name__ == "__main__":
    main()
