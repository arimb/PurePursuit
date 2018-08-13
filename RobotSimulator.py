import configparser
import math
import numpy as np
import cv2

def closest():
    global path, pos

    mindist = (0, math.sqrt((path[0][0] - pos[0]) ** 2 + (path[0][1] - pos[1]) ** 2))
    for i, p in enumerate(path[mindist[0]:]):
        dist = (i, math.sqrt((path[0][0] - pos[0]) ** 2 + (path[0][1] - pos[1]) ** 2) < mindist[1])
        if dist[1] < mindist[1]:
            mindist = dist

    return mindist[0]
def find_lookahead():
    global path, t, pos

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
            print("d=" + str(d) + ", f=" + str(f) + ", disc=" + str(disc) + ", t1=" + str(t1) + ", t2=" + str(t2))
            if 0<=t1<=1:
                if t1 > t:
                    t = t1
                    return p[0]+t*d[0], p[1]+t*d[1]
            if 0<=t2<=1:
                if t2 > t:
                    t = t2
                    return p[0]+t*d[0], p[1]+t*d[1]
        else:
            print("d=" + str(d) + ", f=" + str(f) + ", disc=" + str(disc))

    return path[closest()][0:2]
def curvature(lookahead):
    global path, pos, angle
    # print(pos)
    # print(lookahead)
    return np.sign(math.sin(angle)*(lookahead[0]-pos[0]) - math.cos(angle)*(lookahead[1]-pos[1])) * \
           2*abs(lookahead[0]*(-math.tan(angle))+lookahead[1]+math.tan(angle)*pos[0]-pos[1])/math.sqrt(math.tan(angle)**2+1)/ \
           float(config["PATH"]["LOOKAHEAD"])
def turn(curv, vel, trackwidth):
    return  [vel*(2+curv*trackwidth)/2, vel*(2-curv*trackwidth)/2]

def draw_path(img):
    global path, start_pos
    cv2.circle(img, (int(start_pos[0] + path[0][0] * float(config["FIELD_IMAGE"]["PIXELS_PER_INCH"])/2),
                     int(start_pos[1] - path[0][1] * float(config["FIELD_IMAGE"]["PIXELS_PER_INCH"])/2)),
               2, (255 * (1 - path[0][2] / float(config["VELOCITY"]["MAX_VEL"])), 0,
                   255 * path[0][2] / float(config["VELOCITY"]["MAX_VEL"])), -1)
    for i in range(1, len(path)):
        cv2.circle(img, (int(start_pos[0] + path[i][0] * float(config["FIELD_IMAGE"]["PIXELS_PER_INCH"])/2),
                         int(start_pos[1] - path[i][1] * float(config["FIELD_IMAGE"]["PIXELS_PER_INCH"])/2)),
                   2, (255 * (1 - path[i - 1][2] / float(config["VELOCITY"]["MAX_VEL"])), 0,
                       255 * path[i - 1][2] / float(config["VELOCITY"]["MAX_VEL"])), -1)
        cv2.line(img, (int(start_pos[0] + path[i][0] * float(config["FIELD_IMAGE"]["PIXELS_PER_INCH"])/2),
                       int(start_pos[1] - path[i][1] * float(config["FIELD_IMAGE"]["PIXELS_PER_INCH"])/2)),
                 (int(start_pos[0] + path[i - 1][0] * float(config["FIELD_IMAGE"]["PIXELS_PER_INCH"])/2),
                  int(start_pos[1] - path[i - 1][1] * float(config["FIELD_IMAGE"]["PIXELS_PER_INCH"])/2)),
                 (255 * (1 - path[i - 1][2] / float(config["VELOCITY"]["MAX_VEL"])), 0,
                  255 * path[i - 1][2] / float(config["VELOCITY"]["MAX_VEL"])), 1)
def draw_robot(img):
    tmp = img.copy()
    cv2.drawContours(tmp, [np.array([(start_pos[0] + (pos[0]+length/2*math.sin(angle)-width/2*math.cos(angle))*scaler,
                                      start_pos[1] + (pos[1]+length/2*math.cos(angle)+width/2*math.sin(angle))*-scaler),
                                     (start_pos[0] + (pos[0]+length/2*math.sin(angle)+width/2*math.cos(angle))*scaler,
                                      start_pos[1] + (pos[1]+length/2*math.cos(angle)-width/2*math.sin(angle))*-scaler),
                                     (start_pos[0] + (pos[0]-length/2*math.sin(angle)+width/2*math.cos(angle))*scaler,
                                      start_pos[1] + (pos[1]-length/2*math.cos(angle)-width/2*math.sin(angle))*-scaler),
                                     (start_pos[0] + (pos[0]-length/2*math.sin(angle)-width/2*math.cos(angle))*scaler,
                                      start_pos[1] + (pos[1]-length/2*math.cos(angle)+width/2*math.sin(angle))*-scaler)])
                     .reshape((-1,1,2)).astype(np.int32)], 0, (0, 255, 255), 2)
    cv2.circle(tmp, (int(start_pos[0]+pos[0]*scaler), int(start_pos[1]-pos[1]*scaler)), int(config["PATH"]["LOOKAHEAD"]), (0, 255, 0), 1)
    cv2.circle(tmp, (int(start_pos[0]+lookahead[0]*scaler), int(start_pos[1]-lookahead[1]*scaler)), 4, (0,255,0), -1)
    cv2.imshow("img", tmp)
    cv2.waitKey()

config = configparser.ConfigParser()
config.read("config.ini")

with open(config["PATH"]["FILE_LOCATION"]) as file:
    path = [([float(x) for x in line.split(",")]) for line in file.readlines()]

img = np.zeros((741, 789, 3), np.uint8)
start_pos = (741 / 2, 789 / 2)
draw_path(img)
cv2.imshow("img", img)
cv2.waitKey(5)

scaler = float(config["FIELD_IMAGE"]["PIXELS_PER_INCH"])/2
width = float(config["ROBOT"]["TRACKWIDTH"])
length = float(config["ROBOT"]["LENGTH"])

pos = (0,0)
angle = 0
t = 0
wheels = (0,0)

dt=0.005

lookahead = find_lookahead()

draw_robot(img)

# while closest() != len(path)-1:
#
#     draw_robot(img)
#
#     lookahead = find_lookahead()
#     curv = curvature(lookahead)
#
#     vel = path[closest()][2]
#     last_wheels = wheels
#     wheels = turn(curv, vel, width)
#
#     for i, w in enumerate(wheels):
#         wheels[i] = last_wheels[i] + min(float(config["VELOCITY"]["MAX_VEL_CHANGE"])*dt, max(-float(config["VELOCITY"]["MAX_VEL_CHANGE"])*dt, w-last_wheels[i]))
#
#     pos = (pos[0] + (wheels[0]+wheels[1])/2*dt * math.sin(angle), pos[1] + (wheels[0]+wheels[1])/2*dt * math.cos(angle))
#     angle += math.atan((wheels[0]-wheels[1])/2*dt)
#     print(str(wheels) + ", " + str(angle))