import csv
import numpy as np

FILES = [
    ['modules/map_tool/data/bts.csv',  'modules/map_tool/data/c_bts.csv'],
    ['modules/map_tool/data/bts2.csv', 'modules/map_tool/data/c_bts2.csv'],
    ['modules/map_tool/data/stb.csv',  'modules/map_tool/data/c_stb.csv'],
    ['modules/map_tool/data/stb2.csv', 'modules/map_tool/data/c_stb2.csv']
]

dx = -130.0
dy = 10.0


def write(in_file, out_file):
    with open(in_file, 'r') as infile:
        with open(out_file, 'w', newline='') as csvfile:
            reader = csv.reader(infile, delimiter=' ', quotechar='|')
            writer = csv.writer(csvfile, dialect='excel')
            writer.writerow(["x", "y", "z"])
            for row in reader:
                if len(row) > 1:
                    x = row[0][:-1]
                    y = row[1][:-1]
                    z = row[2][:-1]
                    #print(x)
                    writer.writerow([float(x) + dx, float(y) + dy, float(z)])

if __name__ == '__main__':
    for fin, fout in FILES:
        write(fin, fout)