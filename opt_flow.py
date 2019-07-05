#!/usr/bin/env python

# Python 2/3 compatibility
from __future__ import print_function
import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv
import math
import video


def draw_flow(img, flow, step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1).astype(int)
    fx, fy = flow[y,x].T
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    cv.polylines(vis, lines, 0, (0, 255, 0))
    for (x1, y1), (_x2, _y2) in lines:
        cv.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    cv.circle(vis, (130, 130), 3, (255, 0, 0), -1)
    return vis

def calc_mag_phase(flow):
    magArr = np.zeros((flow.shape[0],flow.shape[1]))
    phaseArr = np.zeros((flow.shape[0],flow.shape[1]))
    for a in range(flow.shape[0]):
        for b in range(flow.shape[1]):
            mag = math.sqrt(math.pow(flow[a,b,0],2) + math.pow(flow[a,b,0],2))
            phase = math.atan2(flow[a,b,1],flow[a,b,0])
            magArr[a,b] = (mag)
            phaseArr[a,b] = (phase)
    return magArr,phaseArr

def main():
    import sys


    cam = video.create_capture("/home/alex/Downloads/tap-test-120fps-da-2019-04-08_193122_500.avi")
    #cam = video.create_capture("/home/alex/Downloads/goodData/2019-07-02_191322_883.avi")
    ret, prev = cam.read()

    prev1 = prev[150:400,460:640]
    prev2 = prev[137:330,0:185]
    prevgray1 = cv.cvtColor(prev1, cv.COLOR_BGR2GRAY)
    prevgray2 = cv.cvtColor(prev2, cv.COLOR_BGR2GRAY)
    show_hsv = False
    show_glitch = False
    cur_glitch = prev.copy()
    magTot = []
    phaseTot = []
    count = 0
    while True:
        ret, img = cam.read()
        #sec1 = img[442:630,155:350]
        sec1 = img[150:400,460:640]
        #sec2 = img[0:185,137:330]
        sec2 = img[137:330,0:185]
        gray1 = cv.cvtColor(sec1, cv.COLOR_BGR2GRAY)
        gray2 = cv.cvtColor(sec2, cv.COLOR_BGR2GRAY)


        flow1 = cv.calcOpticalFlowFarneback(prevgray1, gray1, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        flow2 = cv.calcOpticalFlowFarneback(prevgray2, gray2, None, 0.5, 3, 15, 3, 5, 1.2, 0)

        prevgray1 = gray1
        prevgray2 = gray2

        mag,phase = calc_mag_phase(flow1)
        magTot.append(mag)
        phaseTot.append(phase)
        if(count > 1200):
            break


        #cv.mshow('Dan', draw_flow(gray1, flow1))
        #if(count > 268):
        #    print("frame: ",count)
        #    cv.imshow('Dan', draw_flow(gray1, flow1))
        #    cv.waitKey(0)
        count+=1
        #else:
        cv.waitKey(5)
        #cv.imshow('AL', draw_flow(gray2, flow2))
        #ch = cv.waitKey(5)
        #cv.imshow('AL',)
    tempMag,tempPhase = [],[]
    for count,a in enumerate(magTot):
	tempMag.append(a[130,130])
	#if(a[227,148] < 0.3):
	#	tempPhase.append(0)
	#else:
	tempPhase.append(phaseTot[count][130,130])
    #for a in (magTot):
    #    tempMag.append(a[227,148])
    #for b in phaseTot:
    #    tempPhase.append(b[227,148])
    
    plt.figure(1)
    plt.plot(tempMag)
    plt.title('Magnitude')
    plt.xlabel('Time(frames)')

    plt.figure(2)
    plt.plot(tempPhase)
    plt.title('Phase')
    plt.xlabel('Time(frames)')
    plt.figure(3)
    line1, = plt.plot(tempMag[270:300],label="Magnitude")
    line2, = plt.plot(tempPhase[270:300],label="Phase(radians)")
    plt.title('Phase and Magnitude of Human')
    plt.xlabel('Time(frames)')
    plt.legend(handles=[line1,line2],loc=2)
    #plt.legend(handles=[line2],loc=4)
    plt.show()


if __name__ == '__main__':
    print(__doc__)
    main()
    cv.destroyAllWindows()
