import cv2
# 192.168.0.18:8554/live
cam_url = "http://lcj005:woaistorm5@192.168.0.12:8080/video"
cap = cv2.VideoCapture(url)
def rescale(frame,scale=0.7):
	w=int(frame.shape[1]*scale)
	h=int(frame.shape[0]*scale)
	d=(w,h)
	return cv.resize(frame,d,interpolation=cv.INTER_AREA)
while(cap.isOpened()):

    ret, frame = cap.read()
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()