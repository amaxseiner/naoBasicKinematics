import numpy
import math
import forwardKinematics

UpperArmLength  =105.0
ElbowOffsetY    =15.0
LowerArmLength  =55.95
HandOffsetX 	=57.75
ShoulderOffsetY =98.0
HandOffsetZ     =12.31

incrementVal = .001
l0,l1,l2=1,1.2,1.5
theta0,theta1,theta2=45,30,40 # starting theta
thetaOld = numpy.array([theta0,theta1,theta2])
startingPoint = 0,0,0
desiredPoint = 1.4,1.2,1.2 # meters
b=0.0 # just place holder

def find2dSlope(des,cur):
	slope = (des[1]-cur[1])/(des[0]-cur[0])
	return slope

def find3dSlope(des,cur):
	xDiff = math.sqrt(math.pow(des[0]-cur[0],2) + math.pow(des[1]-cur[0],2))
	yDiff = des[2]-cur[2]
	return math.sqrt(math.pow(xDiff,2)+math.pow(yDiff,2))

def find3dDistance(des,cur):
	xPrime = math.sqrt(math.pow(des[0]-cur[0],2)+math.pow(des[1]-cur[1],2))
	res = math.sqrt(math.pow(xPrime,2)+math.pow(desiredPoint[2]-cur[2],2))
	print res
	return res

#print(find3dSlope(desiredPoint,startingPoint))

def createTransformation(lengths,theta,axis):
	# different rotation matrices for each axis
	if(axis=='z'):
		Rtemp = numpy.array([[math.cos(math.radians(theta)),-math.sin(math.radians(theta)),0,0],
							[math.sin(math.radians(theta)),math.cos(math.radians(theta)),0,0],
							[0,0,1,0],
							[0,0,0,1]])
		Ptemp = numpy.array([[1,0,0,lengths[0]],[0,1,0,lengths[1]],[0,0,1,lengths[2]],[0,0,0,1]])
	elif(axis=='y'):
		Rtemp = numpy.array([[math.cos(math.radians(theta)),0,math.sin(math.radians(theta)),0],
							[0,1,0,0],
							[-math.sin(math.radians(theta)),0,math.cos(math.radians(theta)),0],
							[0,0,0,1]])
		Ptemp = numpy.array([[1,0,0,lengths[0]],[0,1,0,lengths[1]],[0,0,1,lengths[2]],[0,0,0,1]])
	elif(axis=='x'):
		Rtemp = numpy.array([[1,0,0,0],
							[0,math.cos(math.radians(theta)),-math.sin(math.radians(theta)),0],
							[0,math.sin(math.radians(theta)),math.cos(math.radians(theta)),0],
							[0,0,0,1]])
		Ptemp = numpy.array([[1,0,0,lengths[0]],[0,1,0,lengths[1]],[0,0,1,lengths[2]],[0,0,0,1]])

	return numpy.matmul(Rtemp,Ptemp)

# assumes: listOTs is order starting with Translation matrix 0:1 up to Translation matrix n-1:n
def matMullTransformation(listOTs):
	# takes in list of transformation matrices and computes the Cumulative transformation matrix
	TRes = []
	for a in range(len(listOTs)-1):
		if(a == 0):
			# need this first case to give TRes a first value to use
			TRes = numpy.matmul(listOTs[a],listOTs[a+1])
		else:
			TRes = numpy.matmul(TRes,listOTs[a+1])
	return TRes

joint0 = createTransformation([l0,0,0],theta0,'z')
joint1 = createTransformation([l1,0,0],theta1,'z')
joint2 = createTransformation([0,0,l2],theta2,'y')
base = matMullTransformation([joint0,joint1,joint2])
#print base
startingPoint = numpy.zeros(3)
startingPoint[0] = base[0,3]
startingPoint[1] = base[1,3]
startingPoint[2] = base[2,3]
print "starting",startingPoint
print "desired",desiredPoint
def createJacobian():
	global joint0,joint1,joint2
	dTheta = .001 # degrees
	theta0Prime = theta0 + dTheta
	theta1Prime = theta1 + dTheta
	theta2Prime = theta2 + dTheta
	#forwardKinematics.createTransforms([],"RArm")
	PPrime0 = createTransformation([l0,0,0],(theta0Prime),'z')
	PPrime1 = createTransformation([l1,0,0],theta1,'z')
	PPrime2 = createTransformation([0,0,l2],theta2,'y')

	basePrime = matMullTransformation([PPrime0,PPrime1,PPrime2])
	#print("theta0Prime:")
	#print(basePrime)
	dexdTheta0 = (basePrime[0,3] - base[0,3])/dTheta
	deydTheta0 = (basePrime[1,3] - base[1,3])/dTheta
	dezdTheta0 = (basePrime[2,3] - base[2,3])/dTheta

	PPrime0 = createTransformation([l0,0,0],theta0,'z')
	PPrime1 = createTransformation([l1,0,0],(theta1Prime),'z')
	PPrime2 = createTransformation([0,0,l2],theta2,'y')

	basePrime = matMullTransformation([PPrime0,PPrime1,PPrime2])
	#print("theta1Prime:")
	#print(basePrime)
	dexdTheta1 = (basePrime[0,3] - base[0,3])/dTheta
	deydTheta1 = (basePrime[1,3] - base[1,3])/dTheta
	dezdTheta1 = (basePrime[2,3] - base[2,3])/dTheta


	PPrime0 = createTransformation([l0,0,0],theta0,'z')
	PPrime1 = createTransformation([l1,0,0],theta1,'z')
	PPrime2 = createTransformation([0,0,l2],(theta2Prime),'y')

	basePrime = matMullTransformation([PPrime0,PPrime1,PPrime2])
	#print("theta2Prime:")
	#print(basePrime)
	dexdTheta2 = (basePrime[0,3] - base[0,3])/dTheta
	deydTheta2 = (basePrime[1,3] - base[1,3])/dTheta
	dezdTheta2 = (basePrime[2,3] - base[2,3])/dTheta

	#J = numpy.zeros((3,3))
	#J[0,3] = dexdTheta0
	J = numpy.array([[dexdTheta0,dexdTheta1,dexdTheta2],
					[deydTheta0,deydTheta1,deydTheta2],
					[dezdTheta0,dezdTheta1,dezdTheta2]])


	#J = numpy.array([[dexdTheta0,dexdTheta1]#,dexdTheta2],
	#		[deydTheta0, deydTheta1]])#,deydTheta2]])

	#print(J)
	return J

J = createJacobian()
Jinverse = numpy.linalg.inv(J)
#print Jinverse
newPos = startingPoint
for a in range(200):
	diffY,diffX = (desiredPoint[1]-newPos[1]),(desiredPoint[0]-newPos[0])
	slope = abs(find2dSlope(desiredPoint,newPos))#(desiredPoint[1]-newPos[1])/(desiredPoint[0]-newPos[0])
	diffZ = desiredPoint[2]-newPos[2]
	print "2dslope",slope
	if(max([abs(diffX),abs(diffY),abs(diffZ)]) == abs(diffY)):
		if(diffY >0):
			newY = .001
		else:
			newY = -.001
		if(diffX >0):
			newX = (newY-b)/(slope)
		else:
			newX = -(newY-b)/(slope)
		print diffX
		distTemp = math.sqrt(math.pow(newX,2)+math.pow(newY,2))
		if(diffZ>0):
			newZ = math.sqrt(math.pow(distTemp,2) + math.pow(diffZ,2))
		else:
			newZ = -math.sqrt(math.pow(distTemp,2) + math.pow(diffZ,2))

	elif(max([abs(diffX),abs(diffY),abs(diffZ)]) == abs(diffX)):
		if(diffX >0):
			newX = .001
		else:
			newX = -.001
		if(diffY > 0):
			newY = (slope)*newX + b
		else:
			newY = -(slope)*newX + b
		distTemp = math.sqrt(math.pow(newX,2)+math.pow(newY,2))
		if(diffZ>0):
			newZ = math.sqrt(math.pow(distTemp,2) + math.pow(diffZ,2))
		else:
			newZ = -math.sqrt(math.pow(distTemp,2) + math.pow(diffZ,2))
	else:
		if(diffZ > 0):
			newZ = .001
		else:
			newZ = -.001
		if(diffX >0):
			newX = .001
		else:
			newX = -.001
		if(diffY > 0):
			newY = (slope)*newX + b
		else:
			newY = -(slope)*newX + b


	"""if(diffZ >0.0):
		newZ = .001
	else:
		newZ = -.001"""

	# need to figure out newZ

	newPos[0] = newX # - newPos[0]
	newPos[1] = newY # - newPos[1]
	newPos[2] = newZ
	print "movement",newPos
	dThetaJ = numpy.matmul(Jinverse,newPos)
	#print "change in theta", dThetaJ
	thetaNew = thetaOld + dThetaJ
	#print thetaNew
	joint0 = createTransformation([l0,0,0],thetaNew[0],'z')
	joint1 = createTransformation([l1,0,0],thetaNew[1],'z')
	joint2 = createTransformation([0,0,l2],thetaNew[2],'y')
	baseP = matMullTransformation([joint0,joint1,joint2])
	#print(baseP)
	newP = numpy.zeros(3)
	newP[0] = baseP[0,3] # set the x
	newP[1] = baseP[1,3] # set the y
	newP[2] = baseP[2,3] # set the z
	print "new points",newP
	if(find3dDistance(desiredPoint,newP)<.01):
		print "found it at: ",a
		print thetaNew
		break
	newPos = newP
	thetaOld = thetaNew
