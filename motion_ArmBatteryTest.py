# -*- encoding: UTF-8 -*- 

''' PoseZero: Set all the motors of the body to zero. '''

import sys
from naoqi import ALProxy
import time
import math
import forwardKinematics
import qi

def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

def moveArm(motionProxy,pNames):
    numBodies = len(motionProxy.getBodyNames(pNames))
    pTargetAngles = [0.0] * numBodies
    pTargetAngles[0] = 80.0
    pTargetAngles[1] = -9
    pTargetAngles[2] = 60
    pTargetAngles[3] = 80
    #resultingT = forwardKinematics.createTransforms(pTargetAngles,pNames)

    # We set the fraction of max speed
    pMaxSpeedFraction = 0.3
    pTargetAnglesRad = [0.0] * numBodies
    for a in range(len(pTargetAngles)):
	pTargetAnglesRad[a] = math.radians(pTargetAngles[a])

    motionProxy.setAngles(pNames, pTargetAnglesRad, pMaxSpeedFraction)

    time.sleep(1.0)
    pTargetAngles = [0.0] * numBodies
    pTargetAngles[0] = 80.0
    pTargetAngles[1] = -9
    pTargetAngles[2] = 60
    pTargetAngles[3] = 5
    #resultingT = forwardKinematics.createTransforms(pTargetAngles,pNames)

    # We set the fraction of max speed
    pMaxSpeedFraction = 0.3
    pTargetAnglesRad = [0.0] * numBodies
    for a in range(len(pTargetAngles)):
	pTargetAnglesRad[a] = math.radians(pTargetAngles[a])
    motionProxy.setAngles(pNames, pTargetAnglesRad, pMaxSpeedFraction)
    time.sleep(1.0)

def main(robotIP):
    # Init proxies.
    try:
        motionProxy = ALProxy("ALMotion", robotIP, 9559)
	session = qi.Session()
	session.connect("tcp://" + robotIP + ":" + str(9559))

        memory_service = session.service("ALMemory")

    except Exception, e:
        print "Could not create proxy to ALRobotPosture"
        print "Error was: ", e

    try:
        postureProxy = ALProxy("ALRobotPosture", robotIP, 9559)
    except Exception, e:
        print "Could not create proxy to ALRobotPosture"
        print "Error was: ", e

    # Set NAO in Stiffness On
    StiffnessOn(motionProxy)
    csvTiming = open("testTiming.csv","w")
    # Send NAO to Pose Init
    #postureProxy.goToPosture("Stand", 0.5)
    #postureProxy.goToPosture("StandZero", 0.5)
    #time.sleep(2.0)

    # We use the "Body" name to signify the collection of all joints and actuators
    pNames = "RArm"

    # Get the Number of Joints
    
    #print motionProxy.getBodyNames(pNames)
    batPerc = memory_service.getData("Device/SubDeviceList/Battery/Charge/Sensor/Value")
    print("battery%",batPerc)
    startingTime = time.time()
    count = 0
    print("starting time: ",startingTime)
    print("starting bat: ",batPerc)
    while batPerc > .05:
    	moveArm(motionProxy,pNames)
	count = count + 1
	if(count > 10):
        	batPerc = memory_service.getData("Device/SubDeviceList/Battery/Charge/Sensor/Value")
        	print("battery%",batPerc)
		counts = 0 
		csvTiming.write(str(time.time()-startingTime)+","+str(batPerc) + "\n")

    csvTiming.write(str(time.time()-startingTime)+","+str(batPerc)+"\n")
    print("Finishing time: ",time.time()-startingTime)
    #print "Angle values assigned in radians",pTargetAnglesRad

    # Ask motion to do this with a blocking call

    #encoderAngles = motionProxy.getAngles(pNames,True)
    #print(encoderAngles)
    #rotationMatrixExample.createTransforms(encoderAngles,pNames)
    time.sleep(3.0)

if __name__ == "__main__":
    robotIp = "127.0.0.1"

    if len(sys.argv) <= 1:
        print "Usage python motion_poseZero.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)
