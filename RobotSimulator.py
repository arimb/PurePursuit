import configparser
import math

config = configparser.ConfigParser()
config.read("config.ini")

with open(config["PATH"]["FILE_LOCATION"]) as file:
    path = [([float(x) for x in line.split(",")[:-1]]) for line in file.readlines()]


pos = (0,0)
t = 0
point = (0,0)

while True:

    mindist = (0, math.sqrt((path[0][0] - pos[0]) ** 2 + (path[0][1] - pos[1]) ** 2))
    for i, p in enumerate(path[mindist[0]:]):
        dist = (i, math.sqrt((path[0][0]-pos[0])**2 + (path[0][1]-pos[1])**2) < mindist[1])
        if dist[1] < mindist[1]:
            mindist = dist

    for i, p in enumerate(path[:-1]):
        d = (path[i+1][0]-p[0], path[i+1][0]-p[0])
        f = (p[0]-pos[0], p[1]-pos[1])

        a = sum(j**2 for j in d)
        b = 2*sum(j*k for j,k in zip(d,f))
        c = sum(j**2 for j in f) - float(config["PATH"]["LOOKAHEAD"])**2
        disc = b**2 - 4*a*c
        if disc >= 0:
            disc = math.sqrt(disc)
            t1 = (-b - disc)/(2*a)
            t2 = (-b + disc)/(2*a)
            if 0<=t1<=1:
                if t1 > t:
                    t = t1
                    point = (p[0]+t*d[0], p[1]+t*d[1])
                    break
            if 0<=t2<=1:
                if t2 > t:
                    t = t2
                    point = (p[0]+t*d[0], p[1]+t*d[1])
                    break

    