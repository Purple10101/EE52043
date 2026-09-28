import numpy as np
import cv2 as cv
 
def nothing(x):
    pass
 
# Create a black image, a window
img = np.zeros((300,512,3), np.uint8)
cv.namedWindow('image')
 
# create trackbars for color change
cv.createTrackbar('H','image',0,179,nothing)
cv.createTrackbar('S','image',0,255,nothing)
cv.createTrackbar('V','image',0,255,nothing)
 

while(1):
    cv.imshow('image',img)
    k = cv.waitKey(1) & 0xFF
    if k == 27:
        break
 
    # get current positions of four trackbars
    h = cv.getTrackbarPos('H','image')
    s = cv.getTrackbarPos('S','image')
    v = cv.getTrackbarPos('V','image')
     

    im = np.copy(img)
    im[:] = [h,s,v] 
    img = cv.cvtColor(np.asarray(im), cv.COLOR_HSV2BGR)

cv.destroyAllWindows()