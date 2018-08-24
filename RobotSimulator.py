import configparser
import math
import numpy as np
import cv2

def closest():
    global path, pos

    mindist = (0, math.sqrt((path[0][0] - pos[0]) ** 2 + (path[0][1] - pos[1]) ** 2))
    for i, p in enumerate(path):
        dist = math.sqrt((p[0]-pos[0])**2 + (p[1]-pos[1])**2)
        if dist < mindist[1]:
            mindist = (i, dist)

    return mindist[0]
def lookahead():
    global path, t, t_i, pos

    for i, p in enumerate(reversed(path[:-1])):
        i_ = len(path)-2 - i
        d = (path[i_+1][0]-p[0], path[i_+1][1]-p[1])
        f = (p[0]-pos[0], p[1]-pos[1])

        a = sum(j**2 for j in d)
        b = 2*sum(j*k for j,k in zip(d,f))
        c = sum(j**2 for j in f) - float(config["PATH"]["LOOKAHEAD"])**2
        disc = b**2 - 4*a*c
        if disc >= 0:
            disc = math.sqrt(disc)
            t1 = (-b + disc)/(2*a)
            t2 = (-b - disc)/(2*a)
            # print("t1=" + str(t1) + ", t2=" + str(t2))
            if 0<=t1<=1:
                # if (t1 >= t and i == t_i) or i > t_i:
                    t = t1
                    t_i = i_
                    # print("hit")
                    return p[0]+t*d[0], p[1]+t*d[1]
            if 0<=t2<=1:
                # if (t2 >= t and i == t_i) or i > t_i:
                    t = t2
                    t_i = i_
                    # print("hit")
                    return p[0]+t*d[0], p[1]+t*d[1]
    t = 0
    t_i = 0
    return path[closest()][0:2]
def curvature(lookahead):
    global path, pos, angle
    side = np.sign(math.sin(3.1415/2 - angle)*(lookahead[0]-pos[0]) - math.cos(3.1415/2 - angle)*(lookahead[1]-pos[1]))
    a = -math.tan(3.1415/2 - angle)
    c = math.tan(3.1415/2 - angle)*pos[0] - pos[1]
    # x = abs(-math.tan(3.1415/2 - angle) * lookahead[0] + lookahead[1] + math.tan(3.1415/2 - angle)*pos[0] - pos[1]) / math.sqrt((math.tan(3.1415/2 - angle))**2 + 1)
    x = abs(a*lookahead[0] + lookahead[1] + c) / math.sqrt(a**2 + 1)
    return side * (2*x/(float(config["PATH"]["LOOKAHEAD"])**2))
def turn(curv, vel, trackwidth):
    return  [vel*(2+curv*trackwidth)/2, vel*(2-curv*trackwidth)/2]

def draw_path(img):
    global path, start_pos
    cv2.circle(img, (int(start_pos[0]+path[0][0]*scaler), int(start_pos[1]-path[0][1]*scaler)), 2,
               (255*(1-path[0][2]/float(config["VELOCITY"]["MAX_VEL"])), 0, 255*path[0][2]/float(config["VELOCITY"]["MAX_VEL"])), -1)
    for i in range(1, len(path)):
        cv2.circle(img, (int(start_pos[0]+path[i][0]*scaler), int(start_pos[1]-path[i][1]*scaler)), 2,
                   (255*(1-path[i-1][2]/float(config["VELOCITY"]["MAX_VEL"])), 0, 255*path[i-1][2]/float(config["VELOCITY"]["MAX_VEL"])), -1)
        cv2.line(img, (int(start_pos[0]+path[i][0]*scaler), int(start_pos[1]-path[i][1]*scaler)),
                 (int(start_pos[0]+path[i-1][0]*scaler), int(start_pos[1]-path[i-1][1]*scaler)),
                 (255*(1-path[i-1][2]/float(config["VELOCITY"]["MAX_VEL"])), 0, 255*path[i-1][2]/float(config["VELOCITY"]["MAX_VEL"])), 1)
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
    cv2.circle(tmp, (int(start_pos[0]+path[close][0]*scaler), int(start_pos[1]-path[close][1]*scaler)), 4,
               (255*(1-path[close][2]/float(config["VELOCITY"]["MAX_VEL"])), 0, 255*path[close][2]/float(config["VELOCITY"]["MAX_VEL"])), -1)
    cv2.circle(tmp, (int(start_pos[0]+look[0]*scaler), int(start_pos[1]-look[1]*scaler)), 4, (0,255,0), -1)

    try:
        x3 = (pos[0]+look[0])/2
        y3 = -(pos[1]+look[1])/2
        q = math.sqrt((pos[0]-look[0])**2 + (pos[1]-look[1])**2)
        x = x3 - math.sqrt(1/curv**2 - (q/2)**2) * (pos[1]-look[1])/q * np.sign(curv)
        y = y3 - math.sqrt(1/curv**2 - (q/2)**2) * (pos[0]-look[0])/q * np.sign(curv)
        cv2.circle(tmp, (int(x*scaler+start_pos[0]), int(y*scaler+start_pos[1])), int(abs(1/curv*scaler)), (0,255,255), 1)
    except:
        pass

    cv2.line(tmp,
             (int(start_pos[0] + (pos[0]+length/2*math.sin(angle)-width/2*math.cos(angle))*scaler),
              int(start_pos[1] + (pos[1]+length/2*math.cos(angle)+width/2*math.sin(angle))*-scaler)),
             (int(start_pos[0] + (pos[0]+(length/2+wheels[0]/5)*math.sin(angle)-width/2*math.cos(angle))*scaler),
              int(start_pos[1] + (pos[1]+(length/2+wheels[0]/5)*math.cos(angle)+width/2*math.sin(angle))*-scaler)),
             (0,0,255), 2)
    cv2.line(tmp,
             (int(start_pos[0] + (pos[0]+length/2*math.sin(angle)+width/2*math.cos(angle))*scaler),
              int(start_pos[1] + (pos[1]+length/2*math.cos(angle)-width/2*math.sin(angle))*-scaler)),
             (int(start_pos[0] + (pos[0]+(length/2+wheels[1]/5)*math.sin(angle)+width/2*math.cos(angle))*scaler),
              int(start_pos[1] + (pos[1]+(length/2+wheels[1]/5)*math.cos(angle)-width/2*math.sin(angle))*-scaler)),
             (0,0,255), 2)

    cv2.imshow("img", tmp)
    # if itt%6==0:
    #     cv2.imwrite("images/" + str(itt) + ".png", tmp)
    #     print(itt)
    cv2.waitKey(5)

def click(event, x, y, flags, param):
    global pos, angle, t, t_i, wheels
    if event == cv2.EVENT_LBUTTONDOWN:
        pos = ((x-start_pos[0])/scaler,(start_pos[0]-y)/scaler)
        angle = 0
        t = 0
        t_i = 0
        wheels = [0, 0]


config = configparser.ConfigParser()
config.read("config.ini")

with open(config["PATH"]["FILE_LOCATION"]) as file:
    path = [([float(x) for x in line.split(",")]) for line in file.readlines()]

scaler = float(config["FIELD_IMAGE"]["PIXELS_PER_UNIT"])/2
width = float(config["ROBOT"]["TRACKWIDTH"])
length = float(config["ROBOT"]["LENGTH"])

pos = (0,0)
angle = math.atan2(path[1][0], path[1][1])
t = 0
t_i = 0
wheels = [0,0]

dt=0.005

field = cv2.imread(config["FIELD_IMAGE"]["FILE_LOCATION"])
img = np.zeros((field.shape[0], field.shape[1], 3), np.uint8)
start_pos = (field.shape[0]/2, field.shape[1]/2)
draw_path(img)
cv2.imshow("img", img)
cv2.setMouseCallback('img', click)
cv2.waitKey(5)

itt = 0
while closest() != len(path)-1:

    look = lookahead()
    close = closest()
    curv = curvature(look) if t_i>close else 0.00001
    vel = path[close][2]
    last_wheels = wheels
    wheels = turn(curv, vel, width)

    for i, w in enumerate(wheels):
        wheels[i] = last_wheels[i] + min(float(config["ROBOT"]["MAX_VEL_CHANGE"])*dt, max(-float(config["ROBOT"]["MAX_VEL_CHANGE"])*dt, w-last_wheels[i]))

    pos = (pos[0] + (wheels[0]+wheels[1])/2*dt * math.sin(angle), pos[1] + (wheels[0]+wheels[1])/2*dt * math.cos(angle))
    angle += math.atan((wheels[0]-wheels[1])/width*dt)
    # print(str(wheels) + ", " + str(angle))

    draw_robot(img)
    itt += 1

cv2.waitKey()