import numpy
import math

incrementVal = .001
l0,l1,l2=1,1.2,1.5
theta0,theta1,theta2=45,30,80 # starting theta
thetaOld = numpy.array([theta0,theta1,theta2])
startingPoint = 0,0,0
desiredPoint = 1.2,1.6,1.1 # meters
b=0.0 # just place holder

# need to update the slope
if((desiredPoint[1]/desiredPoint[0]) > 1):
	newY = .001
	newX = (newY-b)/(desiredPoint[1]/desiredPoint[0])
else:
	newX = .001
	newY = (desiredPoint[1]/desiredPoint[0])*newX + b

newPos = numpy.array([newX,newY,newZ])


print "Next Position incrementally going to: ",newPos

def find2dSlope(des,cur):
	slope = (des[1]-cur[1])/(des[0]-cur[0])
	return slope

def find3dSlope(des,cur):
	xDiff = math.sqrt(math.pow(des[0]-cur[0],2) + math.pow(des[1]-cur[0],2))
	yDiff = des[2]-cur[2]
	return math.sqrt(math.pow(xDiff,2)+math.pow(yDiff,2))

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
joint2 = createTransformation([l2,0,0],theta2,'x')
base = matMullTransformation([joint0,joint1,joint2])
print base
startingPoint = base[0,3],base[1,3]
print startingPoint
def createJacobian():
	global joint0,joint1,joint2
	dTheta = .001 # degrees
	theta0Prime = theta0 + dTheta
	theta1Prime = theta1 + dTheta
	theta2Prime = theta2 + dTheta

	PPrime0 = createTransformation([l0,0,0],(theta0Prime),'z')
	PPrime1 = createTransformation([l1,0,0],theta1,'z')
	PPrime2 = createTransformation([l2,0,0],theta2,'x')

	basePrime = matMullTransformation([PPrime0,PPrime1])
	print("theta0Prime:")
	print(basePrime)
	dexdTheta0 = (basePrime[0,3] - base[0,3])/dTheta
	deydTheta0 = (basePrime[1,3] - base[1,3])/dTheta
	dezdTheta0 = (basePrime[2,3] - base[2,3])/dTheta

	PPrime0 = createTransformation([l0,0,0],theta0,'z')
	PPrime1 = createTransformation([l1,0,0],(theta1Prime),'z')
	PPrime2 = createTransformation([l2,0,0],theta2,'x')

	basePrime = matMullTransformation([PPrime0,PPrime1])
	print("theta1Prime:")
	print(basePrime)
	dexdTheta1 = (basePrime[0,3] - base[0,3])/dTheta
	deydTheta1 = (basePrime[1,3] - base[1,3])/dTheta
	dezdTheta1 = (basePrime[2,3] - base[2,3])/dTheta


	PPrime0 = createTransformation([l0,0,0],theta0,'z')
	PPrime1 = createTransformation([l1,0,0],theta1,'z')
	PPrime2 = createTransformation([0,0,l1],(theta2Prime),'x')

	basePrime = matMullTransformation([PPrime0,PPrime1,PPrime2])
	print("theta2Prime:")
	print(basePrime)
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

	print(J)
	return J

J = createJacobian()
Jinverse = numpy.linalg.inv(J)
print Jinverse
#print(newPos)
for a in range(2000):
	slope = find2dSlope(desiredPoint,newPos)#(desiredPoint[1]-newPos[1])/(desiredPoint[0]-newPos[0])

	if(slope > 1):
		newY = .001
		newX = (newY-b)/(slope)
	elif(slope <= 1 and slope >0):
		newX = .001
		newY = (slope)*newX + b
	elif(slope < 0 and slope > -1):
		newX = -.001
		newY = (slope)*newX + b
	elif(slope < -1):
		newY = -.001
		newX = (newY-b)/(slope)

	# need to figure out newZ

	print newX, newY
	newPos[0] = newX # - newPos[0]
	newPos[1] = newY # - newPos[1]
	newPos[2] = newZ
	print newPos
	dThetaJ = numpy.matmul(Jinverse,newPos)
	print "change in theta", dThetaJ
	thetaNew = thetaOld + dThetaJ
	print thetaNew
	joint0 = createTransformation([l0,0,0],thetaNew[0],'z')
	joint1 = createTransformation([l1,0,0],thetaNew[1],'z')
	#joint2 = createTransformation([l2,0,0],theta2,'z')
	baseP = matMullTransformation([joint0,joint1])

	newP = numpy.zeros(2)
	newP[0] = baseP[0,3] # set the x
	newP[1] = baseP[1,3] # set the y
	print "new points",newP
	if(math.sqrt(math.pow(desiredPoint[0]-newP[0],2)+math.pow(desiredPoint[1]-newP[1],2))<.01):
		print "found it at: ",a
		break
	newPos = newP
	thetaOld = thetaNew
