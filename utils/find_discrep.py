#!/usr/bin/python3
import csv
import sys

def get_time(d):
    for name in "length realcost".split():
        if name in d:
            return float(d[name])
    raise Exception("can't find time")

def main():
    if len(sys.argv) != 3:
        print("usage:", sys.argv[0], "result1 result2")
        return

    one = {}
    two = {}
    with open(sys.argv[1]) as r1:
        with open(sys.argv[2]) as r2:
            csv1 = csv.DictReader(r1, delimiter=";")
            csv2 = csv.DictReader(r2, delimiter=";")
            for line in csv1:
                one[line["index"]] = get_time(line)
            for line in csv2:
                two[line["index"]] = get_time(line)
    for ind in one:
        if ind not in two:
            continue
        if abs(one[ind] - two[ind]) > 1e-8:
            print(ind)

if __name__ == "__main__":
    main()
