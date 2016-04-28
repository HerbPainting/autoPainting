#!/usr/bin/env python
import numpy


def DrawRandomPoints(no_points):
	import random
	radius = 2
	rangeX = (0, 20)
	rangeY = (0, 20)
	qty = no_points  # or however many points you want

	# Generate a set of all points within 2 of the origin, to be used as offsets later
	# There's probably a more efficient way to do this.
	deltas = set()
	for x in range(-radius, radius+1):
	    for y in range(-radius, radius+1):
	        if x*x + y*y <= radius*radius:
	            deltas.add((x,y))

	randPoints = []
	excluded = set()
	i = 0
	while i<qty:
	    x = random.randrange(*rangeX)
	    y = random.randrange(*rangeY)
	    if (x,y) in excluded: continue
	    randPoints.append((x,y))
	    i += 1
	    excluded.update((x+dx, y+dy) for (dx,dy) in deltas)
	# print randPoints
	return randPoints



def DrawPoints():
	# dm.GetColor("BLue");
	first_points = DrawRandomPoints(10);
	for x,y in first_points:
		# Draw([numpy.array([x/100,y/100])]) 
		a = numpy.array([float(x)/100,float(y)/100])
		print [a]

	# dm.GetColor("Red");
	second_points = DrawRandomPoints(10);
	for x,y in first_points:
		# Draw([numpy.array([x/100,y/100])]) 
		a = numpy.array([float(x)/100,float(y)/100])
		print [a]
	
	import IPython
	IPython.embed()

if __name__ == "__main__":
	DrawPoints();