import cv2

cam = cv2.VideoCapture('/dev/video0')

while True:
	ret, image = cam.read()
	break
cv2.imwrite('usb_image.jpeg', image)
cam.release()
