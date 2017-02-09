import csv
import sys

with open(sys.argv[1], 'rb') as csvfile:
    with open(sys.argv[2], 'wb+') as outfile:
        reader = csv.reader(csvfile, delimiter=',')
        reader.next()
        for row in reader:
            outfile.write(row[2][2:].decode('hex'))
