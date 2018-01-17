import sys
import math

def dist(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def parse_poly(line):
    cols = line.split(',')
    ps = []
    for i in cols[1:]:
        if len(i) == 0:
            break
        a, b = i.strip('()').split('&')
        ps.append((float(a), float(b)))
    if (dist(ps[0], ps[-1]) < 1e-8):
       ps.pop()
    return ps

def parse(fname):
    with open(fname, "r") as f:
        raw_lines = f.readlines()
    polys = []
    for line in raw_lines:
        poly = parse_poly(line)
        if (len(poly) >= 3):
            polys.append(poly)
    return polys

def get_eps(poly):
    n = len(poly)
    res = 1e18
    for i in range(n-1):
        for j in range(i+1, n-1):
            d = dist(poly[i], poly[j])
            if d < res:
                res = d

    return res

def rescale(polys):
    scale = 1e9
    for poly in polys:
        for i in range(len(poly)):
            poly[i] = (poly[i][0] * scale, poly[i][1] * scale)
    return polys

def main(fname):
    polys = []
    polys.extend(parse(fname))
    with open("real.poly", "w") as f:
        f.write("poly\n")
        f.write("1\n")
        f.write("%d\n" % len(polys))
        for p in polys:
            s = "%d %s" % (len(p), " ".join(["%f %f" % (i[0], i[1]) for i in p]))
            f.write("%s\n" % s)

if __name__ == "__main__":
    fname = sys.argv[1]
    main(fname)
