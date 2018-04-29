#!/usr/bin/env python
import sys
import math


def plane(size, fileName):
	file = open(fileName,"w")
	file.write("{:.10f};0;{:.10f}\n".format(size/2.0,size/2.0))
	file.write("{:.10f};0;{:.10f}\n".format(size/2,-size/2))
	file.write("{:.10f};0;{:.10f}\n".format(-size/2,size/2))

	file.write("{:.10f};0;{:.10f}\n".format(size/2,-size/2))
	file.write("{:.10f};0;{:.10f}\n".format(-size/2,-size/2))
	file.write("{:.10f};0;{:.10f}\n".format(-size/2,size/2))
	file.close()
	return




def box(dim_X, dim_Y, dim_Z, divisions, fileName):
	file = open(fileName,"w")
	deltax = dim_X / divisions
	deltay = dim_Y / divisions
	deltaz = dim_Z / divisions
	dx = dim_X/2.0
	dy = dim_Y/2.0
	dz = dim_Z/2.0
	
	# faces da frente e de tras
	for y in range(0, int(divisions)):
		for x in range (0, int(divisions)):
			file.write("{};{};{}\n".format(-dx + x * deltax, -dy  + y * deltay, dz))
			file.write("{};{};{}\n".format(-dx + (x+1) * deltax, -dy + y * deltay, dz))
			file.write("{};{};{}\n".format(-dx + (x+1) * deltax, -dy + (y+1) * deltay, dz))

			file.write("{};{};{}\n".format(-dx + (x+1) * deltax, -dy + (y+1) * deltay, dz))
			file.write("{};{};{}\n".format(-dx + x * deltax, -dy + (y+1) * deltay, dz))
			file.write("{};{};{}\n".format(-dx + x * deltax, -dy + y * deltay, dz))
			
			file.write("{};{};{}\n".format(-dx + (x+1) * deltax, -dy + (y+1) * deltay, -dz))
			file.write("{};{};{}\n".format(-dx + (x+1) * deltax, -dy  + y * deltay, -dz))
			file.write("{};{};{}\n".format(-dx + x * deltax, -dy + y * deltay, -dz))
			
			file.write("{};{};{}\n".format(-dx + x * deltax, -dy  + y * deltay, -dz))
			file.write("{};{};{}\n".format(-dx + x * deltax, -dy  + (y+1) * deltay, -dz))
			file.write("{};{};{}\n".format(-dx + (x+1) * deltax, -dy + (y+1) * deltay, -dz))

	# faces da direita e da esquerda
	for y in range(0,int(divisions)):
		for z in range (0, int(divisions)):
			file.write("{};{};{}\n".format(dx, -dy + y * deltay, dz - z * deltaz))
			file.write("{};{};{}\n".format(dx, -dy + y * deltay, dz - (z+1) * deltaz))
			file.write("{};{};{}\n".format(dx, -dy + (y+1) * deltay, dz - (z+1) * deltaz))

			file.write("{};{};{}\n".format(dx, -dy + (y+1) * deltay, dz - (z+1) * deltaz))
			file.write("{};{};{}\n".format(dx, -dy + (y+1) * deltay, dz - z * deltaz))
			file.write("{};{};{}\n".format(dx, -dy + y * deltay, dz - z * deltaz))

			file.write("{};{};{}\n".format(-dx, -dy + y * deltay, -dz + z * deltaz))
			file.write("{};{};{}\n".format(-dx, -dy + y * deltay, -dz + (z+1) * deltaz))
			file.write("{};{};{}\n".format(-dx, -dy + (y+1) * deltay, -dz + z * deltaz))

			file.write("{};{};{}\n".format(-dx, -dy + (y+1) * deltay, -dz + z * deltaz))
			file.write("{};{};{}\n".format(-dx, -dy + y * deltay, -dz + (z+1) * deltaz))
			file.write("{};{};{}\n".format(-dx, -dy + (y+1) * deltay, -dz + (z+1) * deltaz))
			
	# faces de cima e de baixo
	for z in range(0,int(divisions)):
		for x in range (0, int(divisions)):
			file.write("{};{};{}\n".format(-dx + x * deltax, dy, -dz + z * deltaz))
			file.write("{};{};{}\n".format(-dx + (x+1) * deltax, dy, -dz + (z+1) * deltaz))
			file.write("{};{};{}\n".format(-dx + (x+1) * deltax, dy, -dz + z * deltaz))

			file.write("{};{};{}\n".format(-dx + (x+1) * deltax, dy, -dz + (z+1) * deltaz))
			file.write("{};{};{}\n".format(-dx + x * deltax, dy, -dz + z * deltaz))
			file.write("{};{};{}\n".format(-dx + x * deltax, dy, -dz + (z+1) * deltaz))

			file.write("{};{};{}\n".format(-dx + x * deltax, -dy, -dz + z * deltaz))
			file.write("{};{};{}\n".format(-dx + (x+1) * deltax, -dy, -dz + z * deltaz))
			file.write("{};{};{}\n".format(-dx + x * deltax, -dy, -dz + (z+1) * deltaz))

			file.write("{};{};{}\n".format(-dx + x * deltax, -dy, -dz + (z+1) * deltaz))
			file.write("{};{};{}\n".format(-dx + (x+1) * deltax, -dy, -dz + z * deltaz))
			file.write("{};{};{}\n".format(-dx + (x+1) * deltax, -dy, -dz + (z+1) * deltaz))
	file.close()
	return









def sphere(radius, slices, stacks, fileName):

	#Criar ficheiro
	file = open(fileName,"w")

	# angle of slice
	alfa = (2.0 * math.pi)/(slices)
	# angle of stack
	beta = (math.pi)/(stacks)

	#Entre polo y = -radius aka stack 0 e stack 1
	stackR = radius*math.cos( ( -math.pi/2 ) + beta)
	stackh = radius*math.sin( ( -math.pi/2 ) + beta)
	for x in range(0,int(slices)):

		if x+1 != slices:
			file.write("0.0;{:.10f};0.0\n".format(-radius))
			file.write("{:.10f};{:.10f};{:.10f}\n".format( stackR*math.cos(x*alfa)    , stackh , stackR*math.sin(x*alfa)  ))
			file.write("{:.10f};{:.10f};{:.10f}\n".format( stackR*math.cos((x+1)*alfa)    , stackh , stackR*math.sin((x+1)*alfa)  ))
		else :
			file.write("0.0;{:.10f};0.0\n".format(-radius))
			file.write("{:.10f};{:.10f};{:.10f}\n".format( stackR*math.cos(x*alfa)    , stackh , stackR*math.sin(x*alfa)  ))
			file.write("{:.10f};{:.10f};0.0\n".format( stackR    , stackh ))

	#Nested Loops para os calculos dos triangulos entre stacks em que nenhuma e um dos polos da esfera
	if stacks > 2:
		for istack in range(1, int(stacks)):
			lowStackR  = radius * math.cos(( -math.pi/2 ) + (istack * beta))     # radius of lower stack
			upStackR   = radius * math.cos(( -math.pi/2 ) + ((istack+1) * beta)) # radius of upper stack
			nextStackH = radius * math.sin(( -math.pi/2 ) + ((istack+1) * beta)) # height of upper stack
			curStackH  = radius * math.sin(( -math.pi/2 ) + (istack * beta))     # height of lower stack

			for islice in range(0, int(slices)):

				if islice+1 == slices:
					nextalfa = 0
				else :
					nextalfa = (islice +1) * alfa
				#1st triangle
				file.write("{:.10f};{:.10f};{:.10f}\n".format(lowStackR*math.cos(islice*alfa) , curStackH , lowStackR*math.sin(islice*alfa) ))
				file.write("{:.10f};{:.10f};{:.10f}\n".format(upStackR*math.cos(nextalfa) , nextStackH , upStackR*math.sin(nextalfa) ))
				file.write("{:.10f};{:.10f};{:.10f}\n".format(lowStackR*math.cos(nextalfa) , curStackH , lowStackR*math.sin(nextalfa) ))
				#2nd triangle
				file.write("{:.10f};{:.10f};{:.10f}\n".format(lowStackR*math.cos(islice*alfa) , curStackH , lowStackR*math.sin(islice*alfa) ))
				file.write("{:.10f};{:.10f};{:.10f}\n".format(upStackR*math.cos(islice*alfa) , nextStackH , upStackR*math.sin(islice*alfa) ))
				file.write("{:.10f};{:.10f};{:.10f}\n".format(upStackR*math.cos(nextalfa) , nextStackH , upStackR*math.sin(nextalfa) ))

	#Entre ultima stack e polo y = +radius
	stackh = radius*math.sin( ( math.pi/2 ) - beta) #Como e igual em todas as iteracoes podemos calcular fora do ciclo
	stackR = radius*math.cos( ( math.pi/2 ) - beta)
	for y in range(0, int(slices)):
		if y+1 != slices:
			file.write("0.0;{:.10f};0.0\n".format(radius))
			file.write("{:.10f};{:.10f};{:.10f}\n".format( stackR*math.cos((y+1)*alfa)    , stackh , stackR*math.sin((y+1)*alfa)  ))
			file.write("{:.10f};{:.10f};{:.10f}\n".format( stackR*math.cos(y*alfa)    , stackh , stackR*math.sin(y*alfa)  ))
		else :
			file.write("0.0;{:.10f};0.0\n".format(radius))
			file.write("{:.10f};{:.10f};0.0\n".format( stackR    , stackh ))
			file.write("{:.10f};{:.10f};{:.10f}\n".format( stackR*math.cos(y*alfa)    , stackh , stackR*math.sin(y*alfa)  ))
	
	file.close()
	return








def cone(radius, height, slices, stacks, fileName):
	#Criar ficheiro
	file = open(fileName,"w")

	# angle of slice
	alfa = (2.0 * math.pi)/(slices)
	#top angle
	beta = math.atan(radius/height)
	#height per stack
	hps = height / stacks
	#Base of the cone
	for x in range(0,int(slices)):
		if x+1 != slices:
			file.write("0.0;0.0;0.0\n")
			file.write("{:.10f};0.0;{:.10f}\n".format( radius*math.cos(x*alfa)     , radius*math.sin(x*alfa) ))
			file.write("{:.10f};0.0;{:.10f}\n".format( radius*math.cos((x+1)*alfa) , radius*math.sin((x+1)*alfa) ))
		else :
			file.write("0.0;0.0;0.0\n")
			file.write( "{:.10f};0.0;{:.10f}\n".format( radius*math.cos(x*alfa) , radius*math.sin(x*alfa) ))
			file.write("{:.10f};0.0;0.0\n".format( radius ))

	for istack in range(0, int(stacks-1)):
		curStackH = istack * hps  # height of lower stack
		nextStackH = (istack+1) * hps  # height of upper stack
		lowStackR =  (height - curStackH) * math.tan(beta) #radius of the bottom stack
		upStackR = (height - nextStackH) * math.tan(beta) #radius of upper stack

		for islice in range(0, int(slices)):
			curalfa = islice*alfa
			if islice+1 == slices:
				nextalfa = 0
			else :
				nextalfa = (islice +1) * alfa

			#1st triangle
			file.write("{:.10f};{:.10f};{:.10f}\n".format( lowStackR * math.cos(curalfa)  , curStackH  , lowStackR * math.sin(curalfa) ))
			file.write("{:.10f};{:.10f};{:.10f}\n".format( upStackR  * math.cos(nextalfa) , nextStackH , upStackR  * math.sin(nextalfa) ))
			file.write("{:.10f};{:.10f};{:.10f}\n".format( lowStackR * math.cos(nextalfa) , curStackH  , lowStackR * math.sin(nextalfa) ))
			#2nd triangle
			file.write("{:.10f};{:.10f};{:.10f}\n".format( upStackR  * math.cos(curalfa)  , nextStackH , upStackR * math.sin(curalfa) ))
			file.write("{:.10f};{:.10f};{:.10f}\n".format( upStackR  * math.cos(nextalfa) , nextStackH , upStackR * math.sin(nextalfa) ))
			file.write("{:.10f};{:.10f};{:.10f}\n".format( lowStackR * math.cos(curalfa)  , curStackH  , lowStackR   * math.sin(curalfa) ))
	
	stackh = height - hps
	stackR = (height - stackh) * math.tan(beta)
	for x in range(0,int(slices)):
		if x+1 != slices :
			file.write("0.0;{:.10f};0.0\n".format(height))
			file.write("{:.10f};{:.10f};{:.10f}\n".format( stackR * math.cos( (x+1) * alfa)    , stackh , stackR * math.sin( (x+1) * alfa )  ))
			file.write("{:.10f};{:.10f};{:.10f}\n".format( stackR * math.cos( x * alfa )       , stackh , stackR * math.sin( x * alfa )  ))
		else :
			file.write("0.0;{:.10f};0.0\n".format(height))
			file.write("{:.10f};{:.10f};0.0\n".format( stackR    , stackh  ))
			file.write( "{:.10f};{:.10f};{:.10f}\n".format( stackR*math.cos(x*alfa)    , stackh , stackR*math.sin(x*alfa)  ))


	file.close()
	return


def disc(radiusIn, radiusOut, slices, fileName):

	#Criar ficheiro
	file = open(fileName,"w")
	
	# angle of slice
	alfa = (2.0 * math.pi)/(slices)
	
	for i in range( 0,int(slices) ):
		
		#1st triangle UP
		file.write("{:.10f};0.0;{:.10f}\n".format( radiusIn * math.cos( alfa * i ) , radiusIn * math.sin( alfa * i )))
		file.write("{:.10f};0.0;{:.10f}\n".format( radiusOut * math.cos( alfa * i ) , radiusOut * math.sin( alfa * i )))
		file.write("{:.10f};0.0;{:.10f}\n".format( radiusOut * math.cos( alfa * (i+1) ) , radiusOut * math.sin( alfa * (i+1) )))
			
		#1st triangle DOWN
		file.write("{:.10f};0.0;{:.10f}\n".format( radiusIn * math.cos( alfa * i ) , radiusIn * math.sin( alfa * i )))
		file.write("{:.10f};0.0;{:.10f}\n".format( radiusOut * math.cos( alfa * (i+1) ) , radiusOut * math.sin( alfa * (i+1) )))
		file.write("{:.10f};0.0;{:.10f}\n".format( radiusOut * math.cos( alfa * i ) , radiusOut * math.sin( alfa * i )))

		#2nd triangle UP
		file.write("{:.10f};0.0;{:.10f}\n".format( radiusIn * math.cos( alfa * i ) , radiusIn * math.sin( alfa * i )))
		file.write("{:.10f};0.0;{:.10f}\n".format( radiusOut * math.cos( alfa * (i+1) ) , radiusOut * math.sin( alfa * (i+1) )))
		file.write("{:.10f};0.0;{:.10f}\n".format( radiusIn * math.cos( alfa * (i+1)) , radiusIn * math.sin( alfa * (i+1) )))

		#2nd triangle UP
		file.write("{:.10f};0.0;{:.10f}\n".format( radiusIn * math.cos( alfa * i ) , radiusIn * math.sin( alfa * i )))
		file.write("{:.10f};0.0;{:.10f}\n".format( radiusIn * math.cos( alfa * (i+1)) , radiusIn * math.sin( alfa * (i+1) )))
		file.write("{:.10f};0.0;{:.10f}\n".format( radiusOut * math.cos( alfa * (i+1) ) , radiusOut * math.sin( alfa * (i+1) )))
	file.close()
	return


def bezier(inFileName,tesselation,outFileName):
	patchesReferencePoints = list()
	points = list()
	referencePoints = list()

	outFile = open(outFileName,"w")

	with open(inFileName) as inFile:
		lines = inFile.readlines()
	if len(lines) < 7:
		print "Invalid patch file\n"
	else:
		lines = [x.strip() for x in lines]
		for i in range(1,int(lines[0])+1):
			patchesReferencePoints.append(lines[i])
		for i in range(int(lines[0])+2,len(lines)):
			points.append(lines[i])
		for p in patchesReferencePoints:
			writePatchPointsToFile(p,points,tesselation,outFile)

	inFile.close()
	outFile.close()
	return

def writePatchPointsToFile(patchReferencePoints,points,tesselation,outFile):
	curveReferencePoints = list()
	curvePoints = list()
	previousCurvePoints = list()
	for i in range(0,tesselation+1):
		t = float(i) / float(tesselation)
		if i == 0:
			curveReferencePoints = interpolateCurveReferencePoints(patchReferencePoints,points,t)
			previousCurvePoints = determineCurvePoints(curveReferencePoints,tesselation)
		else:
			curveReferencePoints = interpolateCurveReferencePoints(patchReferencePoints,points,t)
			curvePoints = determineCurvePoints(curveReferencePoints,tesselation)
			writeTrianglesToFile(previousCurvePoints,curvePoints,outFile)
			previousCurvePoints = curvePoints

	return

def writeTrianglesToFile(line1,line2,outFile):
	previousPoint1 = list()
	previousPoint2 = list()
	point1 = list()
	point2 = list()
	i = 0

	for p1,p2 in zip(line1,line2):
		if i == 0:
			previousPoint1 = p1
			previousPoint2 = p2
			i = 1
		else:
			point1 = p1
			point2 = p2
			outFile.write("{:.10f};{:.10f};{:.10f}\n".format(previousPoint1[0],previousPoint1[1],previousPoint1[2]))
			outFile.write("{:.10f};{:.10f};{:.10f}\n".format(previousPoint2[0],previousPoint2[1],previousPoint2[2]))
			outFile.write("{:.10f};{:.10f};{:.10f}\n".format(point1[0],point1[1],point1[2]))
			outFile.write("{:.10f};{:.10f};{:.10f}\n".format(point1[0],point1[1],point1[2]))
			outFile.write("{:.10f};{:.10f};{:.10f}\n".format(previousPoint2[0],previousPoint2[1],previousPoint2[2]))
			outFile.write("{:.10f};{:.10f};{:.10f}\n".format(point2[0],point2[1],point2[2]))

	return



def interpolateCurveReferencePoints(patchReferencePoints,points,t):
	interpolatedCurveReferencePoints = list()
	curveReferencePoints = [None] * 4
	for i in range(0,4):
		referencePointsIndexes = patchReferencePoints.split(",")
		curveReferencePoints[0] = points[int(referencePointsIndexes[0+i*4])]
		curveReferencePoints[1] = points[int(referencePointsIndexes[1+i*4])]
		curveReferencePoints[2] = points[int(referencePointsIndexes[2+i*4])]
		curveReferencePoints[3] = points[int(referencePointsIndexes[3+i*4])]
		interpolatedCurveReferencePoints.append(pointInBezierCurveFromStrings(curveReferencePoints,t))

	return interpolatedCurveReferencePoints;

def determineCurvePoints(curveReferencePoints,tesselation):
	points = list()

	for i in range(0,tesselation+1):
		t = float(i) / float(tesselation)
		points.append(pointInBezierCurve(curveReferencePoints,t))
	return points

def pointInBezierCurve(referencePoints,t):
	P1 = list()
	P2 = list()
	P3 = list()
	P4 = list()
	point = list()

	k1 = (1-t)**3
	k2 = 3*(1-t)**(2)*t
	k3 = 3 * (1-t)*t**2
	k4 = t**3
	
	P1.append(referencePoints[0][0])
	P1.append(referencePoints[0][1])
	P1.append(referencePoints[0][2])

	P2.append(referencePoints[1][0])
	P2.append(referencePoints[1][1])
	P2.append(referencePoints[1][2])

	P3.append(referencePoints[2][0])
	P3.append(referencePoints[2][1])
	P3.append(referencePoints[2][2])

	P4.append(referencePoints[3][0])
	P4.append(referencePoints[3][1])
	P4.append(referencePoints[3][2])

	for p1Coord,p2Coord,p3Coord,p4Coord in zip(P1,P2,P3,P4):
		point.append(k1*p1Coord + k2*p2Coord + k3 * p3Coord + k4 * p4Coord)

	return point

def pointInBezierCurveFromStrings(referencePoints,t):
	P1 = list()
	P2 = list()
	P3 = list()
	P4 = list()
	point = list()

	k1 = (1-t)**3
	k2 = 3*(1-t)**(2)*t
	k3 = 3 * (1-t)*t**2
	k4 = t**3
	
	values = referencePoints[0].split(",")
	P1.append(float(values[0]))
	P1.append(float(values[1]))
	P1.append(float(values[2]))

	values = referencePoints[1].split(",")
	P2.append(float(values[0]))
	P2.append(float(values[1]))
	P2.append(float(values[2]))

	values = referencePoints[2].split(",")
	P3.append(float(values[0]))
	P3.append(float(values[1]))
	P3.append(float(values[2]))

	values = referencePoints[3].split(",")
	P4.append(float(values[0]))
	P4.append(float(values[1]))
	P4.append(float(values[2]))

	for p1Coord,p2Coord,p3Coord,p4Coord in zip(P1,P2,P3,P4):
		point.append(k1*p1Coord + k2*p2Coord + k3 * p3Coord + k4 * p4Coord)

	return point



#TODO: usar switch no argv[1]
def main(argv):
	if len(argv) > 2:
		if argv[1] == "plane":
			if len(argv) == 4:
				plane(float(argv[2]),argv[3])
			else:
				print("ERROR: plane should have 1 additional arguments and output file.\nArguments: size outputFile\n")
		elif argv[1] == "box":
			if len(argv) == 7:
				box(float(argv[2]),float(argv[3]),float(argv[4]),float(argv[5]),argv[6])
			elif len(argv) == 6:
				box(float(argv[2]),float(argv[3]),float(argv[4]),1,argv[5])
			else:
				print("ERROR: box should have at least 3 additional arguments and output file.\nArguments: x y z nrDivisions(optional) outputFile\n")
		elif argv[1] == "sphere":
			if len(argv) == 6:
				sphere(float(argv[2]),float(argv[3]),float(argv[4]),argv[5])
			else:
				print("ERROR: sphere should have 3 additional arguments and output file.\nArguments: radius slices stacks outputFile\n")
		elif argv[1] == "cone":
			if len(argv) == 7:
				cone(float(argv[2]),float(argv[3]),float(argv[4]),float(argv[5]),argv[6])
			else:
				print("ERROR: cone should have 4 additional arguments and output file.\nArguments: radius height slices stacks outputFile\n")
		elif argv[1] == 'disc':
			if len(argv) == 6:
				disc(float(argv[2]),float(argv[3]),float(argv[4]),argv[5])
			else:
				print("ERROR: cone should have 3 additional arguments and output file.\nArguments: radiusIn radiusOut slices  outputFile\n")
		elif argv[1] == 'bezier':	
			if len(argv) == 5:
				bezier(argv[2],int(argv[3]),argv[4]);
			else:
				print("ERROR: bezier should have 1 additional argument and output file.\nArgument: tesselation\n")
		else:
			print("ERROR: Invalid shape\n")
	else:
		print("Usage: ./generator.py shape arguments outputFile\n")
		return

main(sys.argv)
