import numpy as np                      #necessary imports
import cv2
import time 
import math
import serial


color=(255,0,0)                         #variable for contour color and thickness
thickness=2
cX = cY = 0                             #centroid of ball contour
cap = cv2.VideoCapture(1)               #capture from video camera 
j=0
int_x,int_y,prev_x,prev_y = 0,0,0,0     #previous co-ordinates of ball contour centriod
x_cor,y_cor,i = 0,0,0                   #x,y co-ordinate of edge of platform initialize
s = serial.Serial("COM3",9600)          #Establish Serial Communication
s.baudrate = 9600


def Platform(c):

    global x_cor,y_cor,img2,Left,Right,Top,Bottom,frame,Q
    
    Left = tuple(c[c[:, :, 0].argmin()][0])     #This is creating a tuple of x,y cordinates of extreme points
    Right = tuple(c[c[:, :, 0].argmax()][0])    #Minimum along X-Axis is Left and similar logic for others
    Top = tuple(c[c[:, :, 1].argmin()][0])
    Bottom = tuple(c[c[:, :, 1].argmax()][0])

    x_cor = int(((Right[0] - Left[0])**2 + (Right[1] - Left[1])**2 )**0.5)  #Sides of the platform (dynamically)
    y_cor = int(((Bottom[0] - Top[0])**2 + (Bottom[1] - Top[1])**2 )**0.5)
    
    pts1 = np.float32([(list(Top),list(Right),list(Bottom),list(Left))])    #List of all 4 corners
    pts2 = np.float32([[0,0],[x_cor,0],[x_cor,y_cor],[0,y_cor]])            #List of 4 points we want to map it to
    Q = cv2.getPerspectiveTransform(pts1,pts2)                              #Get the Transformation Matrix
    

        

def Ball_Track():

    global dst,x_cor,y_cor,thresh1,frame,Q,i

    dst = cv2.warpPerspective(frame,Q,(x_cor,y_cor))              #Trsansform and view in orthogonal perspective
    
    gray1 = cv2.cvtColor(dst,cv2.COLOR_BGR2GRAY)
    ret,thresh1 = cv2.threshold(gray1,170,255,cv2.THRESH_BINARY) 
    (_,cont_bw,hierarchy)=cv2.findContours(thresh1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)   #contours of ball


    if len(cont_bw) != 0:
        #l = max(cont_bw, key = cv2.contourArea)
        for q in range(len(cont_bw)):
            peri = cv2.arcLength(cont_bw[q], True)
            approx = cv2.approxPolyDP(cont_bw[q], 0.01 * peri, True)
            area = cv2.contourArea(cont_bw[q])
            #print(len(approx))
            if peri != 0 :
                #print(area/peri)
                if (len(approx)>=7 and area/peri > 8):      # circle will have more than 7 sides and also area/peri is Radius/2
                    print(area/peri)
                    dst=cv2.drawContours(dst, cont_bw[q], -1, [0,255,0], thickness) #Draw contours of the ball
                    M = cv2.moments(cont_bw[q])
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])           #Centroid of ball
                        cY = int(M["m01"] / M["m00"])
                        i = [cX,cY]                             #List of centroid
                        print(i)
                        data = PID()                            #Get Servo Angles to send by PID
                        Serial_C(data)                          #Send data to Arduino

def PID():

    global x_cor,y_cor,i
    global int_x,int_y,prev_x,prev_y
    
    Ball_x = 25*(i[0]-x_cor/2)//x_cor                           #Co-ordinates of Ball maped in cm
    Ball_y = 25*(i[1]-y_cor/2)//y_cor

    #Ball_x = 25*(600-400//2)//200
    #Ball_y = 25*(600-400//2)//200

    Kp = 1       #1
    Kd = -40     #-35   
    Ki = 0.015   #-0.01                                          #PID co-efficients

    
    angle_x = (90+int(Kp*(Ball_x) + Kd*(prev_x-(Ball_x)) + Ki*(Ball_x + int_x)))    #X-Angle to send 
    angle_y = (90+int(Kp*Ball_y + Kd*(prev_y-(Ball_y))+ Ki*(y_cor + Ball_y)))       #Y-Angle to send
    
    int_x = Ball_x                              #Storing x,y co-ordinates
    int_y = Ball_y

    prev_x = Ball_x
    prev_y = Ball_y

    angle_x = max(60,angle_x)                   #Min Angle to send is 60 deg and max 120 deg
    angle_x = min(120,angle_x)

    angle_y = max(60,angle_y)
    angle_y = min(120,angle_y)
    
    ard_x = str(angle_x)                       #Making is as 6digit like 087098 for 87 and 98 degrees
    if(len(ard_x)==2):
        ard_x = "0"+ard_x
        
    ard_y = str(angle_y)
    if(len(ard_y)==2):
        ard_y = "0"+ard_y

    arduino = ard_x + ard_y + "*"              #End of command character

    return arduino

    

def Serial_C(data):

    global s
    s.write(data.encode())          #Send Data to Arduino
    
    
if __name__ == "__main__":
    global j,img2,Left,Right,Top,Bottom,dst,thresh1,frame
    
    while(True):
        j=j+1
        #print(j)
        # Capture frame-by-frame
        ret, frame = cap.read()  # ret = 1 if the video is captured; frame is the image

        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,150,255,cv2.THRESH_BINARY_INV)
        (_,contour,hierarchy)=cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)  #Contours for Platform  
        
        if len(contour) != 0:
                c = max(contour, key = cv2.contourArea) # find the largest contour
                
                img2=cv2.drawContours(frame, c, -1, color, thickness) # draw largest contour

                if(j>=25):          #From 25th Frame for settling the image
        
                    Platform(c)     #Make Platform Contours
    
                if(j>=25):

                    Ball_Track()    #Make Ball Track Contours
                    
                    cv2.circle(img2, Left, 8, (0, 0, 255), -1)        #Points (Extreme Display)
                    cv2.circle(img2, Right, 8, (0, 255, 0), -1)
                    cv2.circle(img2, Top, 8, (255, 0, 0), -1)
                    cv2.circle(img2, Bottom, 8, (255, 255, 0), -1)
                        
                    cv2.imshow('Original View',img2)                  #Display all 3 views
                    cv2.imshow('B&W',thresh1)
                    cv2.imshow('Tracking',dst)

                    
        # Display the resulting image
        #cv2.imshow('Contour',img3)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # press q to quit    
           break
            
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

