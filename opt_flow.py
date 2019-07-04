#!/usr/bin/env python

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import cv2 as cv


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
    return vis

def calc_mag_phase(flow):
    magArr = []
    phaseArrr = []
    for a in range(flow.shape[0]):
        for b in range(flow.shape[1]):
            mag = math.sqrt(math.pow(flow[a,b,0],2) + math.pow(flow[a,b,0]))
            phase = math.atan2(flow[a,b,1],flow[a,b,0])
            magArr.append(mag)
            phaseArr.append(phase)
    return magArr,phaseArr

def main():
    import sys
    try:
        fn = sys.argv[1]
    except IndexError:
        fn = 0

    cam = cv.VideoCapture("home/alex/Downloads/tap-test-120fps-da-2019-04-08_193122_500.avi")
    ret, prev = cam.read()
    ret, prev = cam.read()
    ret, prev = cam.read()
    ret, prev = cam.read()
    ret, prev = cam.read()
    
    print(prev.shape)
    prevgray = cv.cvtColor(prev, cv.COLOR_BGR2GRAY)
    show_hsv = False
    show_glitch = False
    cur_glitch = prev.copy()
    magTot = []
    phaseTot = []
    while True:
        ret, img = cam.read()
        sec1 = img[442,155:630,350]
        sec2 = img[0,137:185,330]
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        flow = cv.calcOpticalFlowFarneback(prevgray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        prevgray = gray
        #mag,phase = calc_mag_phase(flow)
        #magTot.append(mag)
        #phaseTot.append(phase)
        cv.imshow('flow', draw_flow(gray, flow))


        ch = cv.waitKey(5)


    print('Done')


if __name__ == '__main__':
    print(__doc__)
    main()
    cv.destroyAllWindows()
