#!/usr/bin/env python
# -*- coding: utf-8 -*-
#######################################################
# FiveBars robot
import math
import numpy

##################################################
# Definition of a 5R robot with approximate architecture
import robot

nominal_architecture = [-22.5,0,22.5,0,17.8,17.8,17.8,17.8]
r5 = robot.FiveBars(nominal_architecture,seed=3, mode=0)
#~ Suite à la calibration, on a obtenu l'architecture suivante :
calibrated_architecture = [-22.48910557, 0.2487764, 22.31083019, 0.19295762, 17.75206524, 17.75148665, 17.82798783, 18.18320809]

# RRRRR kinematic functions
def f_RRRRR(architecture,pose,command):
    [a11,a12,a21,a22,l1,l2,l3,l4] = architecture
    [x1,x2] = pose
    [q1,q2] = numpy.radians(command)
    f1 = (a11 + l1*math.cos(q1) - x1)**2 + (a12 + l1*math.sin(q1) - x2)**2 - l2**2
    f2 = (a21 + l4*math.cos(q2) - x1)**2 + (a22 + l4*math.sin(q2) - x2)**2 - l3**2
    return [f1,f2]

###############################################################
# PATH PLANNING 

# define and display the obstacles
from matplotlib import pyplot

def put_obstacles(rob):
    obstacles = [(-12,-6,1.5),(-12,0,1.1),(-12,6,1.8),(-6,-18,2.8),(-6,-12,2.9),(-6,-6,0.6),(-6,0,1.6) ,(-6,6,0.4),(-6,12,1.9),(-6,18,0.4),(0,-24,2.4),(0,-12,1.9),(0,-12,1.9),(0,-6,0.5),(0,0,1.8),(0,6,2.9),(0,12,1.7),(0,24,0.5),(6,-18,1.9),(6,-12,2.8),(6,-6,0.4),(6,0,1.9),(6,6,0.3),(6,12,2.3),(6,18,1.2),(12,-6,2.8),(12,0,1.3),(12,6,2.1)]
    for o in obstacles:
        rob.ax.add_artist(pyplot.Circle((o[0],o[1]),o[2],color='.5',fill=False))
    rob.refresh()
    return obstacles

obstacles = put_obstacles(r5)

# Construction of discretized target path
def discretize(r5,trajectory,tmin,tmax,steps):
    target_path = [trajectory(t) for t in numpy.linspace(tmin,tmax,steps)]
    r5.ax.plot([x[0] for x in target_path],[x[1] for x in target_path],color='blue',linestyle=':',marker='+')
    r5.refresh()
    return target_path

import paving

from scipy.sparse.csgraph import dijkstra

# Path planning between two target boxes
def path_planner(pav,nei,Bori,Bdes):
    print('Building shortest path from ',Bori,' to ',Bdes)
    # construction of the shortest path
    path=[]
    dist,pred = dijkstra(nei,return_predecessors=True,indices=Bori)
    # checking existence of a compatible destination box
    if dist[Bdes]!=numpy.inf:
        print('   shortest path found!')
        # collecting the path from the predecessors tree
        path=[Bdes]
        while path[0]!=Bori:
            path.insert(0,pred[path[0]])
    else:
        print('   impossible to connect boxes!')
    return path

def display_boxes(rob, pav):
	for b in [i for i in range(len(pav.boxes)) if l[i]==l[ori[0]]]:
	    pav.boxes[b].draw2D(rob.ax,1,2)
	for b in [i for i in range(len(pav.boxes)) if l[i]!=l[ori[0]]]:
	    pav.boxes[b].draw2D(rob.ax,1,2,ec='magenta')

# Display a box path and actuate the robot along it
def display_path(rob,pav,spath,bcol='yellow',rcol='cyan'):
    def midq(b):
        return numpy.degrees([(b.vec[4]+b.vec[5])/2,(b.vec[6]+b.vec[7])/2])
    if spath!=[]:
        rob.pen_up()
        rob.actuate(midq(pav.boxes[spath[0]]))
        rob.pen_down(rcol)
        for b in spath:
            if bcol!=None:
                pav.boxes[b].draw2D(rob.ax,1,2,ec=bcol,fc=bcol)
            rob.actuate(midq(pav.boxes[b]))
        rob.pen_up()
        rob.go_home()

Xori, Xdes = [0,-15], [0,15] 
r5.ax.plot([Xori[0],Xdes[0]],[Xori[1],Xdes[1]],linestyle=':',marker='*',color='.3')
r5.refresh()
paving_5R = paving.Paving()
paving_5R.load_mnf('5R-obs.mnf')
ori = list(paving_5R.boxes_intersecting(Xori,d=[1,2]))
des = list(paving_5R.boxes_intersecting(Xdes,d=[1,2]))

neighborhood = paving_5R.adjacency_matrix()
shortest_path=path_planner(paving_5R,neighborhood,ori[0],des[0])