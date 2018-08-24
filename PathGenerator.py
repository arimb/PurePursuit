import cv2
import configparser
import math
import sys

# HANDLE MOUSE EVENTS FOR SELECTION
def click(event, x, y, flags, param):
    global waypoints, img, start_pos, mouse_down
    try:
        if mouse_down:
            tmp = img.copy()
            place_point(tmp, x, y, flags&cv2.EVENT_FLAG_SHIFTKEY, False)
            cv2.imshow("Field", tmp)
        if event == cv2.EVENT_LBUTTONDOWN:
            mouse_down = True
        if event == cv2.EVENT_LBUTTONUP:
            mouse_down = False
            if len(waypoints) == 0:
                start_pos = (x,y)
            place_point(img, x, y, flags&cv2.EVENT_FLAG_SHIFTKEY, True)
            cv2.imshow("Field", img)
    except:
        pass

def place_point(img, x, y, shift, add):
    if shift and len(waypoints)>0:
        if abs(x-(start_pos[0]+waypoints[-1][0])) < abs(y-(start_pos[1]-waypoints[-1][1])):
            x = start_pos[0] + waypoints[-1][0]
        else:
            y = start_pos[1] - waypoints[-1][1]
    cv2.circle(img, (x, y), 3, (0, 255, 255), -1)
    if len(waypoints)>0:
        cv2.line(img, (x, y), (start_pos[0]+waypoints[-1][0], start_pos[1]-waypoints[-1][1]), (0, 255, 255), 2)
    if add:
        waypoints.append((x - start_pos[0], start_pos[1] - y))

# INITIALIZE VALUES
waypoints = []
start_pos = (0,0)
mouse_down = False

# READ CONFIG FILE
config = configparser.ConfigParser()
config.read("config.ini")
scaler = float(config["FIELD_IMAGE"]["PIXELS_PER_UNIT"])

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
waypoints[0] = tuple(x/scaler for x in waypoints[0])
for i in range(len(waypoints)-1):
    waypoints[i+1] = tuple(x/scaler for x in waypoints[i+1])
    dist = math.sqrt((waypoints[i+1][0]-waypoints[i][0])**2 + (waypoints[i+1][1]-waypoints[i][1])**2)
    j=0
    while j < dist:
        total_waypoints.append(tuple(a+j/dist*(b-a) for a,b in zip(waypoints[i], waypoints[i+1])))
        j += float(config["POINT_INJECTION"]["POINT_DIST"])
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

smooth_waypoints[0][5] = float(config["VELOCITY"]["STARTING_VEL"])
for i, w in enumerate(smooth_waypoints[1:], start=1):
    test = math.sqrt(smooth_waypoints[i-1][5]**2 + 2*float(config["VELOCITY"]["MAX_ACCEL"])* \
                     math.sqrt((w[0] - smooth_waypoints[i-1][0]) ** 2 + (w[1] - smooth_waypoints[i-1][1]) ** 2))
    print(test)
    if test < w[5]:
        w[5] = test
    else:
        break

# WRITE RESULTS TO FILE
with open(config["PATH"]["FILE_LOCATION"], "w+") as file:
    for w in smooth_waypoints:
        file.write(str(w[0]) + "," + str(w[1]) + "," + str(w[5]) + "\n")

# DISPLAY COLOR-CODED IMAGE OF PATH
cv2.circle(img, (int(start_pos[0]+smooth_waypoints[0][0]*scaler),
                     int(start_pos[1]-smooth_waypoints[0][1]*scaler)),
               2, (255*(1-smooth_waypoints[0][5]/float(config["VELOCITY"]["MAX_VEL"])), 0,
                   255*smooth_waypoints[0][5]/float(config["VELOCITY"]["MAX_VEL"])), -1)
for i in range(1, len(smooth_waypoints)):
    cv2.circle(img, (int(start_pos[0]+smooth_waypoints[i][0]*scaler),
                     int(start_pos[1]-smooth_waypoints[i][1]*scaler)),
               2, (255*(1-smooth_waypoints[i-1][5]/float(config["VELOCITY"]["MAX_VEL"])), 0,
                   255*smooth_waypoints[i-1][5]/float(config["VELOCITY"]["MAX_VEL"])), -1)
    cv2.line(img, (int(start_pos[0]+smooth_waypoints[i][0]*scaler),
                     int(start_pos[1]-smooth_waypoints[i][1]*scaler)),
             (int(start_pos[0] + smooth_waypoints[i-1][0] * scaler),
              int(start_pos[1] - smooth_waypoints[i-1][1] * scaler)),
             (255 * (1 - smooth_waypoints[i - 1][5] / float(config["VELOCITY"]["MAX_VEL"])), 0,
              255*smooth_waypoints[i - 1][5] / float(config["VELOCITY"]["MAX_VEL"])), 1)
cv2.imshow("Field", img)
cv2.waitKey()