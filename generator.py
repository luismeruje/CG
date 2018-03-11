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
	return;




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
	return;









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
	return;








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



	return;










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
			else:
				print("ERROR: box should have 4 additional arguments and output file.\nArguments: x y z nrDivisions outputFile\n")
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
		else:
			print("ERROR: Invalid shape\n")
	else:
		print("Usage: ./generator.py shape arguments outputFile\n")
		return;

main(sys.argv)