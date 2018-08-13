import cv2
import configparser
import math
import sys

# HANDLE MOUSE EVENTS FOR SELECTION
def click(event, x, y, flags, param):
    global waypoints, img, start_pos, mouse_down
    if mouse_down:
        tmp = img.copy()
        tmp = cv2.circle(tmp, (x,y), 3, (0,255,255), -1)
        if len(waypoints) > 0:
            cv2.line(tmp, (x, y), (start_pos[0]+waypoints[-1][0], start_pos[1]-waypoints[-1][1]), (0,255,255), 2)
        cv2.imshow("Field", tmp)
    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_down = True
    if event == cv2.EVENT_LBUTTONUP:
        mouse_down = False
        if len(waypoints) == 0:
            start_pos = (x,y)
        waypoints.append((x-start_pos[0], start_pos[1]-y))
        cv2.circle(img, (start_pos[0]+waypoints[-1][0], start_pos[1]-waypoints[-1][1]), 3, (0,255,255), -1)
        if len(waypoints) > 1:
            cv2.line(img, (start_pos[0]+waypoints[-2][0], start_pos[1]-waypoints[-2][1]),
                     (start_pos[0]+waypoints[-1][0], start_pos[1]-waypoints[-1][1]), (0,255,255), 2)
        cv2.imshow("Field", img)

# INITIALIZE VALUES
waypoints = []      # [(0, 0), (0, 457), (-388, 457), (-385, 573), (-385, 583)]
start_pos = (0,0)   # (600,760)
mouse_down = False

# READ CONFIG FILE
config = configparser.ConfigParser()
config.read("config.ini")

# READ & SHOW IMAGE, SET OPENCV PROPERTIES
img = cv2.imread(config["FIELD_IMAGE"]["FILE_LOCATION"])
cv2.imshow("Field", img)
cv2.setMouseCallback("Field", click)
cv2.waitKey()

# MAKE SURE AT LEAST 2 POINTS SELECTED
if len(waypoints) < 2:
    sys.exit(0)

# CONVERT PIXELS TO INCHES, CALCULATE WAYPOINT DISTANCE, INJECT MID-WAYPOINTS
total_waypoints = []
waypoints[0] = tuple(x/float(config["FIELD_IMAGE"]["PIXELS_PER_INCH"]) for x in waypoints[0])
for i in range(len(waypoints)-1):
    waypoints[i+1] = tuple(x/float(config["FIELD_IMAGE"]["PIXELS_PER_INCH"]) for x in waypoints[i+1])
    dist = math.sqrt((waypoints[i+1][0]-waypoints[i][0])**2 + (waypoints[i+1][1]-waypoints[i][1])**2)
    j=0
    while j < dist:
        total_waypoints.append(tuple(a+j/dist*(b-a) for a,b in zip(waypoints[i], waypoints[i+1])))
        j += float(config["POINT_INJECTION"]["POINT_DIST_INCH"])
total_waypoints.append(waypoints[-1])

# SMOOTH WAYPOINTS - W[0]=X, W[1]=Y
smooth_waypoints = [[w[0], w[1]] for w in total_waypoints]
weight_data = float(config["POINT_INJECTION"]["WEIGHT_DATA"])
weight_smooth = float(config["POINT_INJECTION"]["WEIGHT_SMOOTH"])
tolerance = float(config["POINT_INJECTION"]["TOLERANCE"])
change = tolerance
while change >= tolerance:
    change = 0
    for i in range(1, len(total_waypoints)-1):
        for j in range(len(total_waypoints[i])):
            aux = smooth_waypoints[i][j]
            smooth_waypoints[i][j] += weight_data*(total_waypoints[i][j]-smooth_waypoints[i][j]) + \
                weight_smooth*(smooth_waypoints[i-1][j]+smooth_waypoints[i+1][j]-2*smooth_waypoints[i][j])
            change += abs(aux - smooth_waypoints[i][j])

# CALCULATE PATH DISTANCE - W[2]
smooth_waypoints[0].append(0)
for i, w in enumerate(smooth_waypoints[1:], start=1):
    w.append(smooth_waypoints[i-1][2] + math.sqrt((w[0]-smooth_waypoints[i-1][0])**2 + (w[1]-smooth_waypoints[i-1][1])**2))

# CALCULATE CURVATURE - W[3]
smooth_waypoints[0].append(0.0001)
smooth_waypoints[-1].append(0.0001)
for i, w in enumerate(smooth_waypoints[1:-1], start=1):
    w[0] += 0.0001
    w[1] += 0.0001
    k1 = .5*(w[0]**2 + w[1]**2 - smooth_waypoints[i-1][0]**2 - smooth_waypoints[i-1][1]**2) / (w[0] - smooth_waypoints[i-1][0])
    k2 = (w[1] - smooth_waypoints[i-1][1]) / (w[0] - smooth_waypoints[i-1][0])
    b = .5*(smooth_waypoints[i-1][0]**2 - 2*smooth_waypoints[i-1][0]*k1 + smooth_waypoints[i-1][1]**2 - smooth_waypoints[i+1][0]**2 + 2*smooth_waypoints[i+1][0]*k1 - smooth_waypoints[i+1][1]**2) / (smooth_waypoints[i+1][0]*k2 - smooth_waypoints[i+1][1] + smooth_waypoints[i-1][1] - smooth_waypoints[i-1][0]*k2)
    a = k1 - k2*b
    r = math.sqrt((w[0]-a)**2 + (w[1]-b)**2)
    w.append(1/r)

# CALCULATE DESIRED VELOCITY - W[4]
for w in smooth_waypoints:
    w.append(min(float(config["VELOCITY"]["MAX_VEL"]), float(config["VELOCITY"]["TURNING_CONST"])/w[3]))

# ADD ACCELERATION LIMITS - W[5]
smooth_waypoints[-1].append(0)
for i, w in enumerate(reversed(smooth_waypoints[:-1]), start=1):
    w.append(min(w[4], math.sqrt(smooth_waypoints[-i][5]**2+2*float(config["VELOCITY"]["MAX_ACCEL"])* \
                                  math.sqrt((w[0]-smooth_waypoints[-i][0])**2 + (w[1]-smooth_waypoints[-i][1])**2))))

# WRITE RESULTS TO FILE
with open(config["PATH"]["FILE_LOACTION"], "w+") as file:
    for w in smooth_waypoints:
        file.write(str(w[0]) + "," + str(w[1]) + "," + str(w[5]) + "\n")

# DISPLAY COLOR-CODED IMAGE OF PATH
cv2.circle(img, (int(start_pos[0]+smooth_waypoints[0][0]*float(config["FIELD_IMAGE"]["PIXELS_PER_INCH"])),
                     int(start_pos[1]-smooth_waypoints[0][1]*float(config["FIELD_IMAGE"]["PIXELS_PER_INCH"]))),
               2, (255*(1-smooth_waypoints[0][5]/float(config["VELOCITY"]["MAX_VEL"])), 0,
                   255*smooth_waypoints[0][5]/float(config["VELOCITY"]["MAX_VEL"])), -1)
for i in range(1, len(smooth_waypoints)):
    cv2.circle(img, (int(start_pos[0]+smooth_waypoints[i][0]*float(config["FIELD_IMAGE"]["PIXELS_PER_INCH"])),
                     int(start_pos[1]-smooth_waypoints[i][1]*float(config["FIELD_IMAGE"]["PIXELS_PER_INCH"]))),
               2, (255*(1-smooth_waypoints[i-1][5]/float(config["VELOCITY"]["MAX_VEL"])), 0,
                   255*smooth_waypoints[i-1][5]/float(config["VELOCITY"]["MAX_VEL"])), -1)
    cv2.line(img, (int(start_pos[0]+smooth_waypoints[i][0]*float(config["FIELD_IMAGE"]["PIXELS_PER_INCH"])),
                     int(start_pos[1]-smooth_waypoints[i][1]*float(config["FIELD_IMAGE"]["PIXELS_PER_INCH"]))),
             (int(start_pos[0] + smooth_waypoints[i-1][0] * float(config["FIELD_IMAGE"]["PIXELS_PER_INCH"])),
              int(start_pos[1] - smooth_waypoints[i-1][1] * float(config["FIELD_IMAGE"]["PIXELS_PER_INCH"]))),
             (255 * (1 - smooth_waypoints[i - 1][5] / float(config["VELOCITY"]["MAX_VEL"])), 0,
              255*smooth_waypoints[i - 1][5] / float(config["VELOCITY"]["MAX_VEL"])), 1)
cv2.imshow("Field", img)
cv2.waitKey()