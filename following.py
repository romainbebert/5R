#######################################################
# TwoBars robot
import math
import numpy

##################################################
# Definition of a RR robot with approximate architecture
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
# PATH FOLLOWING 

# Definition of the target trajectory
def circle(t):
    r = 5
    d1 = r * numpy.cos(t)
    d2 = r * numpy.sin(t)
    return (d1,-20+d2)

# Construction of discretized target path
def discretize(r5,trajectory,tmin,tmax,steps):
    target_path = [trajectory(t) for t in numpy.linspace(tmin,tmax,steps)]
    r5.ax.plot([x[0] for x in target_path],[x[1] for x in target_path],color='blue',linestyle=':',marker='+')
    r5.refresh()
    return target_path

target_path = discretize(r5,circle,0,2*numpy.pi,50)
x0 = target_path[0]
q0 = [numpy.degrees(-0.357023),numpy.degrees(-1.548630)+360] # IBEX solution 0
#q0 = [numpy.degrees(-0.357023),numpy.degrees(-3.010361)+360] # IBEX solution 1
#q0 = [numpy.degrees(-0.912730),numpy.degrees(-1.548630)+360] # IBEX solution 2
#q0 = [numpy.degrees(-0.912730),numpy.degrees(-3.010361)+360] # IBEX solution 3
r5.actuate(q0)

from scipy.optimize import root

# continuation procedure
def continuation(function_xq,target_path,q0):
    qs=[]
    qk=q0
    for xk in target_path:
        res = root(lambda q: function_xq(xk,q), qk)
        if not res.success:
            print(res.message)
            break
        qk = res.x
        qs.append(qk)
    return qs

#~ commands = continuation(lambda x,q:f_RRRRR(nominal_architecture,x,q),target_path,q0)

def draw_path(r5,commands,col='blue'):
    r5.pen_up()
    r5.go_home()
    r5.actuate(commands[0])
    r5.pen_down(col)
    real_path = []
    for q in commands:
        r5.actuate(q)
        real_path.append(numpy.array(r5.measure_pose()))
    r5.pen_up()
    r5.go_home()
    return real_path

#~ real_path = draw_path(r5,commands,col='red')

# distance function used to measure error
def dist(x,y):
    return numpy.sqrt((x[0]-y[0])**2+(x[1]-y[1])**2)


#~ max([dist(x,rx) for (x,rx) in zip(target_path,real_path)])

#~ commands = continuation(lambda x,q:f_RRRRR(calibrated_architecture,x,q),target_path,q0)
#~ real_path = draw_path(r5,commands)
#~ max([dist(x,rx) for (x,rx) in zip(target_path,real_path)])

###############################################################
# RÉSULTAT

# Pour le mode 0, on obtient 4 commandes possibles pour la pose initiale
# car cette pose est atteignable avec chaque coude en position haute ou basse

# En partant de la pose de la solution 0 (coude gauche en position haute, coude droit en position basse) :

# Erreur avec l'architecture nominale :
# 0.7360216765063133
# Erreur avec l'architecture calibrée :
# 0.026936066940079714

# En partant de la pose de la solution 1 (les deux coudes en position haute) :

# Erreur avec l'architecture nominale :
# 0.9948486422395161
# Erreur avec l'architecture calibrée :
# 0.02691115810478777

# En partant de la pose de la solution 2 (les deux coudes en position basse) :

# Erreur avec l'architecture nominale :
# 5.120909144132331
# Erreur avec l'architecture calibrée :
# 4.201167425834054
# L'erreur est très importante puisque dans cette position le robot ne peut pas atteindre tout le quart haut du cercle

# En partant de la pose de la solution 3 (coude gauche en position basse, coude droit en position haute) :

# Erreur avec l'architecture nominale :
# 0.35800741537239755
# Erreur avec l'architecture calibrée :
# 0.027759310530693714