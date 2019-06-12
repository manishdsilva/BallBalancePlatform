import numpy as np
import cv2
import time 
import math
import serial


color=(255,0,0)
thickness=2
cX = cY = 0
cap = cv2.VideoCapture(1)
j=0
int_x,int_y,prev_x,prev_y = 0,0,0,0
x_cor,y_cor,i = 0,0,0
s = serial.Serial("COM3",9600)
s.baudrate = 9600


def Platform(c):

    global x_cor,y_cor,img2,Left,Right,Top,Bottom,frame,Q
    
    Left = tuple(c[c[:, :, 0].argmin()][0])
    Right = tuple(c[c[:, :, 0].argmax()][0])
    Top = tuple(c[c[:, :, 1].argmin()][0])
    Bottom = tuple(c[c[:, :, 1].argmax()][0])

    x_cor = int(((Right[0] - Left[0])**2 + (Right[1] - Left[1])**2 )**0.5)
    y_cor = int(((Bottom[0] - Top[0])**2 + (Bottom[1] - Top[1])**2 )**0.5)
    
    pts1 = np.float32([(list(Top),list(Right),list(Bottom),list(Left))])
    pts2 = np.float32([[0,0],[x_cor,0],[x_cor,y_cor],[0,y_cor]])
    Q = cv2.getPerspectiveTransform(pts1,pts2)
    

        

def Ball_Track():

    global dst,x_cor,y_cor,thresh1,frame,Q,i

    dst = cv2.warpPerspective(frame,Q,(x_cor,y_cor))
    
    gray1 = cv2.cvtColor(dst,cv2.COLOR_BGR2GRAY)
    ret,thresh1 = cv2.threshold(gray1,170,255,cv2.THRESH_BINARY) 
    (_,cont_bw,hierarchy)=cv2.findContours(thresh1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

##    circles = cv2.HoughCircles(thresh1,cv2.HOUGH_GRADIENT,1,600,
##                    param1=20,param2=10,minRadius=10,maxRadius=25)
##
##    if(circles is not None):
##        circles = np.uint16(np.around(circles))
##        
##        for i in circles[0,:]:
##            # draw the outer circle
##            cv2.circle(dst,(i[0],i[1]),i[2],(0,255,0),2)
##            # draw the center of the circle
##            cv2.circle(dst,(i[0],i[1]),2,(0,0,255),3)
##
##            cv2.circle(dst, (x_cor//2,y_cor//2), 8, (255, 255, 0), -1)
##
##            data = PID()
##
##            #print(data)/;;;
##            
##            Serial_C(data)
    if len(cont_bw) != 0:
        #l = max(cont_bw, key = cv2.contourArea)
        for q in range(len(cont_bw)):
            peri = cv2.arcLength(cont_bw[q], True)
            approx = cv2.approxPolyDP(cont_bw[q], 0.01 * peri, True)
            area = cv2.contourArea(cont_bw[q])
            #print(len(approx))
            if peri != 0 :
                #print(area/peri)
                if (len(approx)>=7 and area/peri > 8):
                    print(area/peri)
                    dst=cv2.drawContours(dst, cont_bw[q], -1, [0,255,0], thickness)
                    M = cv2.moments(cont_bw[q])
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        i = [cX,cY]
                        print(i)
                        data = PID()
                        Serial_C(data)

def PID():

    global x_cor,y_cor,i
    global int_x,int_y,prev_x,prev_y
    
    Ball_x = 25*(i[0]-x_cor/2)//x_cor
    Ball_y = 25*(i[1]-y_cor/2)//y_cor

    #Ball_x = 25*(600-400//2)//200
    #Ball_y = 25*(600-400//2)//200

    Kp = 1       #1
    Kd = -40     #-35   
    Ki = 0.015   #-0.01      

    
    angle_x = (90+int(Kp*(Ball_x) + Kd*(prev_x-(Ball_x)) + Ki*(Ball_x + int_x)))
    angle_y = (90+int(Kp*Ball_y + Kd*(prev_y-(Ball_y))+ Ki*(y_cor + Ball_y)))
    
    int_x = Ball_x
    int_y = Ball_y

    prev_x = Ball_x
    prev_y = Ball_y

    angle_x = max(60,angle_x)
    angle_x = min(120,angle_x)

    angle_y = max(60,angle_y)
    angle_y = min(120,angle_y)
    
    ard_x = str(angle_x)
    if(len(ard_x)==2):
        ard_x = "0"+ard_x
        
    ard_y = str(angle_y)
    if(len(ard_y)==2):
        ard_y = "0"+ard_y

    arduino = ard_x + ard_y + "*"

    return arduino

    

def Serial_C(data):

    global s
    s.write(data.encode())
    
    
if __name__ == "__main__":
    global j,img2,Left,Right,Top,Bottom,dst,thresh1,frame
    
    while(True):
        j=j+1
        #print(j)
        # Capture frame-by-frame
        ret, frame = cap.read()  # ret = 1 if the video is captured; frame is the image

        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        

        lower_4=np.array([68,0,45])
        upper_4=np.array([255,289,145])
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_4 = cv2.inRange(hsv, lower_4, upper_4)
        blur_mask_4 = cv2.medianBlur(mask_4,5)
        _, contours_4, hierarchy = cv2.findContours(blur_mask_4, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    

        ret,thresh = cv2.threshold(gray,150,255,cv2.THRESH_BINARY_INV)
        
        (_,contour,hierarchy)=cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        

        if len(contour) != 0:
                c = max(contour, key = cv2.contourArea) # find the largest contour
                
                img2=cv2.drawContours(frame, c, -1, color, thickness) # draw largest contour

                if(j>=25):
        
                    Platform(c)
    
                if(j>=25):

                    Ball_Track()
                    
                    cv2.circle(img2, Left, 8, (0, 0, 255), -1)
                    cv2.circle(img2, Right, 8, (0, 255, 0), -1)
                    cv2.circle(img2, Top, 8, (255, 0, 0), -1)
                    cv2.circle(img2, Bottom, 8, (255, 255, 0), -1)
                        
                    cv2.imshow('Original View',img2)
                    cv2.imshow('B&W',thresh1)
                    cv2.imshow('Tracking',dst)

                    
        # Display the resulting image
        #cv2.imshow('Contour',img3)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # press q to quit
           break
            
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

