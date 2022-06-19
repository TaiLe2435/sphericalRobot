# imports
import cv2
import numpy as np
import math
import time

# function looking for clicks
def clickEvent(event, x, y, flags, params):
    global flag, point
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
        flag = True
        # displaying the coordinates
        print(x, ' ', y) 
        point = (x,y)
 
    # checking for right mouse clicks    
    if event==cv2.EVENT_RBUTTONDOWN:
        flag = True
        # displaying the coordinates
        print(x, ' ', y)
        point = (x,y)

# capturing video
cap = cv2.VideoCapture(1) # change this value until you get the correct cam
flag = False
prevCircle = None
dist = lambda x1, y1, x2, y2: (x1-x2)**2+(y1-y2)**2 # mini function

while True:
    _, img = cap.read()
    # resizing for faster processing
    img = cv2.resize(img, (320,240), interpolation = cv2.INTER_LINEAR)
    
    # creating window for mouse input
    cv2.namedWindow("Robot Tracking")
    cv2.setMouseCallback('Robot Tracking', clickEvent)
    if flag == True:
        cv2.circle(img, point, 10, (255, 0, 0), -1)

    # converting to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # HSV range for color and defining mask 
    # change these for respective colors
    # link for HSV stuff
    # https://www.google.com/imgres?imgurl=https%3A%2F%2Fi.stack.imgur.com%2FTSKh8.png&imgrefurl=https%3A%2F%2Fstackoverflow.com%2Fquestions%2F47483951%2Fhow-to-define-a-threshold-value-to-detect-only-green-colour-objects-in-an-image&tbnid=Jx2H1bjYvu6n_M&vet=12ahUKEwiA6p3xp-z3AhXUXM0KHcAtDh4QMygBegUIARDDAQ..i&docid=d4AswGhN6lbYWM&w=720&h=355&q=hsv%20range&ved=2ahUKEwiA6p3xp-z3AhXUXM0KHcAtDh4QMygBegUIARDDAQ
    # https://www.google.com/imgres?imgurl=https%3A%2F%2Fanswers.opencv.org%2Fupfiles%2F15181560142151344.png&imgrefurl=https%3A%2F%2Fanswers.opencv.org%2Fquestion%2F184281%2Fhow-are-hsv-values-interpreted-in-python%2F&tbnid=mpa5ObAswr1QPM&vet=12ahUKEwi2qKKx8oL4AhVaookEHZVJDLQQxiAoAXoECAAQGQ..i&docid=06ORPhgZpk_9yM&w=743&h=477&itg=1&q=hsv%20range&ved=2ahUKEwi2qKKx8oL4AhVaookEHZVJDLQQxiAoAXoECAAQGQ
    lower = np.array([0, 0, 0])             # change upper and lower values for colors
    upper = np.array([180, 20, 70])
    mask = cv2.inRange(hsv, lower, upper)
    
    # tracking color
    contours, hierarchy=cv2.findContours(mask,cv2.RETR_EXTERNAL ,cv2.CHAIN_APPROX_TC89_L1)
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour) 
        maxArea = max(contours, key = cv2.contourArea)
        
        # only taking largest area
        if(area > 10): 
            x, y, w, h = cv2.boundingRect(maxArea) 
            # finding center of target
            xC = int(x + w/2)
            yC = int(y + h/2)
            center = (xC, yC)
            
            # plotting bounding box and center
            img = cv2.rectangle(img, (x, y), 
                                    (x + w, y + h), 
                                    (0, 255, 0), 2) 
            cv2.circle(img, center, 10, (0, 255, 0), -1)
            cv2.putText(img, "Target", (x, y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
                        (0, 255, 0))
    
      
    cv2.imshow("Robot Tracking", img) 
    
    # 'Esc' key to exit video streaming
    key = cv2.waitKey(1)
    if key ==27:
        break
    
cap.release()
cv2.destroyAllWindows()