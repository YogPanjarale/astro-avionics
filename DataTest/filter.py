# code reads flight_data.csv and filters out comments
# and writes the filtered data to flight_data_filtered.csv
import csv

def filter_data():
    with open('flight_data.csv', 'r') as file:
        reader = csv.reader(file)
        with open('flight_data_filtered.csv', 'w') as file:
            writer = csv.writer(file)
            for row in reader:
                if row[0][0] == '#':
                    continue
                writer.writerow(row)
# replace spaces with commas
def clean():
    with open('flight_data_filtered.csv', 'r') as file:
        data = file.read()
        data = data.replace(' ', ',')
    with open('flight_data_filtered.csv', 'w') as file:
        file.write(data)
    print('Data filtered and written to flight_data_filtered.csv')
if __name__ == '__main__':
    filter_data()
    clean()