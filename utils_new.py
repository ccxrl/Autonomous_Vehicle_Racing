# new utils for trackbars to work withtout display

import cv2
import numpy as np


# FOLLOWING BLACK LANE WITH WHITE LINES
# def thresholding(img):
#     imgHsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
#     lowerWhite = np.array([0, 0, 201])  # HUE Min, SAT Min, VALUE Min
#     upperWhite = np.array([179, 255, 255])  # HUE Max, SAT Max, VALUE Max
#     maskWhite = cv2.inRange(imgHsv,lowerWhite,upperWhite)
#     return maskWhite

def thresholding(img):
    imgHsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    lowerBlack = np.array([86, 58, 0])
    upperBlack = np.array([176, 255, 255])
    maskBlack = cv2.inRange(imgHsv,lowerBlack,upperBlack)
    return maskBlack

def warpImg(img,points,w,h,inv = False):
    pts1 = np.float32(points)
    pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])
    if inv:
        matrix = cv2.getPerspectiveTransform(pts2, pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1,pts2)
    imgWarp = cv2.warpPerspective(img,matrix,(w,h))
    return imgWarp

def nothing(a):
    pass

# Default values for when not using trackbars
DEFAULT_VALUES = {
    "Width Top": 0,
    "Height Top": 0,
    "Width Bottom": 0,
    "Height Bottom": 240
}

def initializeTrackbars(initialTrackbarVals,wT=480, hT=240):
    try:
        cv2.namedWindow("Trackbars")
        cv2.resizeWindow("Trackbars", 360, 240)
        cv2.createTrackbar("Width Top", "Trackbars", initialTrackbarVals[0],wT//2, nothing)
        cv2.createTrackbar("Height Top", "Trackbars", initialTrackbarVals[1], hT, nothing)
        cv2.createTrackbar("Width Bottom", "Trackbars", initialTrackbarVals[2],wT//2, nothing)
        cv2.createTrackbar("Height Bottom", "Trackbars", initialTrackbarVals[3], hT, nothing)
    except:
        print("Running without trackbars")

def valTrackbars(wT=480, hT=240):
    try:
        widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
        heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
        widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
        heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
    except:
        # Use default values if trackbars are not available
        widthTop = DEFAULT_VALUES["Width Top"]
        heightTop = DEFAULT_VALUES["Height Top"]
        widthBottom = DEFAULT_VALUES["Width Bottom"]
        heightBottom = DEFAULT_VALUES["Height Bottom"]

    points = np.float32([(widthTop, heightTop), (wT-widthTop, heightTop),
                      (widthBottom , heightBottom ), (wT-widthBottom, heightBottom)])
    return points

def drawPoints(img,points):
    for x in range(4):
        cv2.circle(img,(int(points[x][0]),int(points[x][1])),15,(0,0,255),cv2.FILLED)
    return img

def getHistogram(img,minPer=0.1,display= False,region=1):
    if region ==1:
        histValues = np.sum(img, axis=0)
    else:
        histValues = np.sum(img[img.shape[0]//region:,:], axis=0)

    maxValue = np.max(histValues)
    minValue = minPer*maxValue

    indexArray = np.where(histValues >= minValue)
    basePoint = int(np.average(indexArray))

    if display:
        imgHist = np.zeros((img.shape[0],img.shape[1],3),np.uint8)
        for x,intensity in enumerate(histValues):
            cv2.line(imgHist,(x,img.shape[0]),(x,int(img.shape[0]-intensity//255//region)),(255,0,255),1)
            cv2.circle(imgHist,(basePoint,img.shape[0]),20,(0,255,255),cv2.FILLED)
        return basePoint,imgHist

    return basePoint

def stackImages(scale,imgArray):
    # (keep this function as is)
    pass