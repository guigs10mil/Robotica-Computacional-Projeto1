import numpy as np
import cv2 as cv
import math

def detectCamShift(frame):
    if 'roi_hist' in locals():
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        dst = cv.calcBackProject([hsv],[0],roi_hist,[0,180],1)
        # apply meanshift to get the new location
        ret, track_window = cv.CamShift(dst, track_window, term_crit)
        # Draw it on image
        pts = cv.boxPoints(ret)
        pts = np.int0(pts)
        img2 = cv.polylines(frame,[pts],True, 255,2)
        cv.rectangle(frame, (c, r), (c+w, r+h), (255,255,255), 1)

        media = (np.int32(np.mean(pts[:,0])), np.int32(np.mean(pts[:, 1])))
        centro = (frame.shape[1]//2, frame.shape[0]//2)

        cv.circle(frame,centro,10,150,3, cv.LINE_AA)
        cv.circle(frame,media,10,255,3, cv.LINE_AA)

        cv.imshow('img2',img2)

        x1 = np.int32(pts)[0][0]
        y1 = np.int32(pts)[0][1]

        x2 = np.int32(pts)[1][0]
        y2 = np.int32(pts)[1][1]

        x3 = np.int32(pts)[2][0]
        y3 = np.int32(pts)[2][1]

        xdist1=x1-x2
        ydist1=y1-y2

        xdist2=x1-x3
        ydist2=y1-y3

        lado1=math.sqrt((xdist1)**2+(ydist1)**2)
        lado2=math.sqrt((xdist2)**2+(ydist2)**2)

        area=lado1*lado2

        cv.imshow('img2',img2)
        k = cv.waitKey(60) & 0xff
        if k == 32:
            roi = frame[r:r+h, c:c+w]
            hsv_roi =  cv.cvtColor(roi, cv.COLOR_BGR2HSV)
            mask = cv.inRange(hsv_roi, np.array((0., 60.,32.)), np.array((180.,255.,255.)))
            roi_hist = cv.calcHist([hsv_roi],[0],mask,[180],[0,180])
            cv.normalize(roi_hist,roi_hist,0,255,cv.NORM_MINMAX)

        return media, centro, area

    else:
        centro = (frame.shape[1]//2, frame.shape[0]//2)
        r,h,c,w = centro[1]-50,100,centro[0]-50,100  # simply hardcoded the values
        track_window = (c,r,w,h)
        # set up the ROI for tracking
        roi = frame[r:r+h, c:c+w]
        hsv_roi =  cv.cvtColor(roi, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv_roi, np.array((0., 60.,32.)), np.array((180.,255.,255.)))
        roi_hist = cv.calcHist([hsv_roi],[0],mask,[180],[0,180])
        cv.normalize(roi_hist,roi_hist,0,255,cv.NORM_MINMAX)
        term_crit = ( cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 1 )
        media = 0
        centro = 0
        area = 0
        return media, centro, area
