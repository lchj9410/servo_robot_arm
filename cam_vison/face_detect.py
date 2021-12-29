import cv2 as cv

img=cv.imread("image0.jpeg")

gray=cv.cvtColor(img,cv.COLOR_BGR2GRAY)


haar_cas = cv.CascadeClassifier('face.xml')
face = haar_cas.detectMultiScale(gray,scaleFactor=1.1,minNeighbors=4)
for (x,y,w,h) in face:
    cv.rectangle(img,(x,y),(x+w,y+h),(0,255,0),thickness=1)
cv.imshow('dg',img)
cv.waitKey(0)