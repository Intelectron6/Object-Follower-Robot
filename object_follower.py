 from picamera.array import PiRGBArray     
from picamera import PiCamera
from gpiozero import Servo
from gpiozero import Motor
import time
import cv2
import numpy as np
    
def segment_colour(frame):
    hsv_roi =  cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_roi, np.array([160, 160,10]), np.array([190,255,255]))
    kern_dilate = np.ones((8,8),np.uint8)
    kern_erode  = np.ones((3,3),np.uint8)
    mask= cv2.erode(mask,kern_erode) 
    mask=cv2.dilate(mask,kern_dilate)  
    cv2.imshow('mask',mask)
    return mask

def find_blob(blob):
    largest_contour=0
    cont_index=0
    _, contours, hierarchy = cv2.findContours(blob, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for idx, contour in enumerate(contours):
        area=cv2.contourArea(contour)
        if (area >largest_contour) :
            largest_contour=area
            cont_index=idx                              
    r=(0,0,2,2)
    if len(contours) > 0:
        r = cv2.boundingRect(contours[cont_index])       
    return r,largest_contour

def target_hist(frame):
    hsv_img=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)   
    hist=cv2.calcHist([hsv_img],[0],None,[50],[0,255])
    return hist

camera = PiCamera()
camera.rotation = 270
camera.brightness = 51
camera.resolution = (480, 368)
camera.framerate = 16
rawCapture = PiRGBArray(camera, size=(480, 368))

motorR = Motor(23,18)
motorL = Motor(25,24)
last = 0

time.sleep(0.01)

for image in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):     
      frame = image.array
      global centre_x
      global centre_y
      centre_x=0.
      centre_y=0.
      hsv1 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      mask_red=segment_colour(frame)     
      loct,area=find_blob(mask_red)
      x,y,w,h=loct
             
      if (w*h) < 10:
            found = 0
            if last==0:
                print('pivot left')
                motorR.forward(0.75)
                motorL.backward(0.75)
            else:
                print('pivot right')
                motorL.forward(0.75)
                motorR.backward(0.75)
      else:
            found = 1
            simg2 = cv2.rectangle(frame, (x,y), (x+w,y+h), 255,2)
            cv2.imshow('simg2',simg2)
            centre_x=x+((w)/2)
            centre_y=y+((h)/2)
            cv2.circle(frame,(int(centre_x),int(centre_y)),3,(0,110,255),-1)
            print (centre_x,centre_y)
            print(w*h)
            value = centre_x/160
            if (w*h) < 24000:
                if centre_x < 160:
                    motorR.forward(1)
                    motorL.forward(value)
                    print('left:',value)
                    last = 0
                elif centre_x < 320:
                    motorR.forward(1)
                    motorL.forward(1)
                    print('front')
                else:
                    motorR.forward((2.999 - value))
                    motorL.forward(1)
                    print('right:',2.999 - value)
                    last = 1
            elif (w*h) < 28000:
                if centre_x < 160:
                    print('left')
                    motorR.forward(0.5)
                    motorL.stop()
                    last = 0
                elif centre_x < 320:
                    print('stop')
                    motorR.stop()
                    motorL.stop()
                else:
                    print('right')
                    motorL.forward(0.5)
                    motorR.stop()
                    last = 1
            else:
                print('reverse')
                motorR.backward(0.5)
                motorL.backward(0.5)
                    
      rawCapture.truncate(0)
      cv2.imshow('stuff',frame)
      if(cv2.waitKey(1) & 0xff == ord('q')):
            break
cv2.destroyAllWindows()

