import RPi.GPIO as GPIO
import time
import cv2
from PIL import ImageFont, Image, ImageDraw
import numpy as np

#motor_initial_setting
servo_pin = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin,GPIO.OUT) 
pwm = GPIO.PWM(servo_pin,50) #50Hz(servo pwm hz)
#initial state
a = 180
pwm.start(3+a*4.5/90) #inital_closing_setting
a = 0
timer = [0]*10

face = False


#cam_setting
CAM_ID = 0

cap = cv2.VideoCapture(CAM_ID)
cv2.namedWindow('Face')
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
face_cascade = cv2.CascadeClassifier()
face_cascade.load('/home/pi/opencv/opencv-4.1.2/data/haarcascades/haarcascade_frontalface_default.xml')

#resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH,400)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,200)



#display
b,g,r = 255,0,0
fontpath = '/home/pi/Documents/Fonts/gulim.ttc'
font = ImageFont.truetype(fontpath,30)


while(True):
	ret, frame = cap.read()
	frame = cv2.flip(frame,0)
	grayframe = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
	grayframe = cv2.equalizeHist(grayframe)
	faces = face_cascade.detectMultiScale(grayframe,1.1,3,0,(30,30))
	
	if faces is():
		print("0")
		'''
		timer[0] = False
		timer[0],timer[1],timer[2],timer[3],timer[4],timer[5],timer[6],timer[7],timer[8],timer[9] = timer[1],timer[2],timer[3],timer[4],timer[5],timer[6],timer[7],timer[8],timer[9],timer[0]
		'''
		timer[0] = False
		timer[0:9],timer[9] = timer[1:10],timer[0]
	else:
		print("1")
		face = True
		'''
		timer[0] = True
		timer[0],timer[1],timer[2],timer[3],timer[4],timer[5],timer[6],timer[7],timer[8],timer[9] = timer[1],timer[2],timer[3],timer[4],timer[5],timer[6],timer[7],timer[8],timer[9],timer[0]
		'''
		timer[0] = True
		timer[0:9],timer[9] = timer[1:10],timer[0]
		if sum(timer) == 9:
			a =1150
			pwm.ChangeDutyCycle(3+a*4.5/90)
			time.sleep(1)
			pwm.ChangeDutyCycle(3+140*4.5/90)
			pwm.stop()
			break
		'''
		a = 1
		pwm.ChangeDutyCycle(3+a*4.5/90)
		time.sleep(2.0)
		pwm.stop()
		GPIO.cleanup()
		break
		'''
		
		
	
	for (x,y,w,h) in faces:
		cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),3,4,0)
		img_pil = Image.fromarray(frame)
		draw = ImageDraw.Draw(img_pil)
		draw.text((30,30),"face",font=font, fill = (b,g,r))
		frame= np.array(img_pil)
	cv2.imshow('Face',frame)
	
	if cv2.waitKey(10) >=0:
		break
GPIO.cleanup()
cap.release()
cv2.destroyWindow('Face')
