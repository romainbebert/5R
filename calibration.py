#######################################################
# FiveBars robot
import math
import numpy

##################################################
# Definition of a RR robot with approximate architecture
import robot

#~ nominal_architecture = [-22.5,0,22.5,0,17.8,17.8,17.8,17.8]
#~ r5 = robot.FiveBars(nominal_architecture,seed=3, mode=0,man_var=0,mes_var=0)
#~ r5 = robot.FiveBars(nominal_architecture,seed=3, mode=0, eps_cmd=100)

##################################################
# CALIBRATION

# RRRRR kinematic functions
def f_RRRRR(architecture,pose,command):
    [a11,a12,a21,a22,l1,l2,l3,l4] = architecture
    [x1,x2] = pose
    [q1,q2] = numpy.radians(command)
    f1 = (a11 + l1*math.cos(q1) - x1)**2 + (a12 + l1*math.sin(q1) - x2)**2 - l2**2
    f2 = (a21 + l4*math.cos(q2) - x1)**2 + (a22 + l4*math.sin(q2) - x2)**2 - l3**2
    return [f1,f2]

# Actuation of the robot in order to generate measures for calibration
def make_measurements(r2,commands,col='black',mar='*'):
    r5.actuate(commands[0])
    measures=[]
    print('   Taking measures ...')
    for q in commands:
        print(q)
        r5.actuate(q)
        x = r5.measure_pose()
        r5.ax.plot([x[0]],[x[1]],color=col,marker=mar)
        measures.append((x,q))
    r5.go_home()
    return measures

#~ commands = [[q1,q2] for q1 in range(-90,90,15) for q2 in range(90,270,15)]
#~ puis en supprimant les commandes générant des singularités (singularity met) :
#~ commands = [[-90, 165], [-90, 180], [-90, 195], [-90, 210], [-90, 225], [-75, 165], [-75, 180], [-75, 195], [-75, 210], [-75, 225], [-75, 240], [-75, 255], [-60, 150], [-60, 165], [-60, 180], [-60, 195], [-60, 210], [-60, 225], [-60, 240], [-60, 255], [-45, 135], [-45, 150], [-45, 165], [-45, 180], [-45, 195], [-45, 210], [-45, 225], [-45, 240], [-45, 255], [-30, 120], [-30, 135], [-30, 150], [-30, 165], [-30, 180], [-30, 195], [-30, 210], [-30, 225], [-30, 240], [-30, 255], [-15, 105], [-15, 120], [-15, 135], [-15, 150], [-15, 165], [-15, 180], [-15, 195], [-15, 210], [-15, 225], [-15, 240], [-15, 255], [0, 90], [0, 105], [0, 120], [0, 135], [0, 150], [0, 165], [0, 180], [0, 195], [0, 210], [0, 225], [0, 240], [0, 255], [15, 90], [15, 105], [15, 120], [15, 135], [15, 150], [15, 165], [15, 180], [15, 195], [15, 210], [15, 225], [15, 240], [15, 255], [30, 90], [30, 105], [30, 120], [30, 135], [30, 150], [30, 165], [30, 180], [30, 195], [30, 210], [30, 225], [30, 240], [45, 90], [45, 105], [45, 120], [45, 135], [45, 150], [45, 165], [45, 180], [45, 195], [45, 210], [45, 225], [60, 105], [60, 120], [60, 135], [60, 150], [60, 165], [60, 180], [60, 195], [60, 210], [75, 105], [75, 120], [75, 135], [75, 150], [75, 165], [75, 180], [75, 195]]
#~ measures = make_measurements(r5,commands)

# calibration from measurements
from scipy.optimize import least_squares

def calibrate(kinematic_functions,nominal_architecture,measures):
    # error function
    def errors(a):
        err=[]
        for (x,q) in measures:
            for fi in kinematic_functions(a,x,q):
                err.append(fi)
        return err
    print('   Calibration processing ...')
    sol = least_squares(errors,nominal_architecture)
    print('   status : ',sol.message)
    print('   error : ',sol.cost)
    print('   result : ',sol.x)
    return sol.x

#~ calibrated_architecture = calibrate(f_RRRRR,nominal_architecture,measures)
"""
    a11 = -22.48910557;
    a12 = 0.2487764;
    a21 = 22.31083019;
    a22 = 0.19295762;
    l1 = 17.75206524;
    l2 = 17.75148665;
    l3 = 17.82798783;
    l4 = 18.18320809;
"""

###############################################################
# VÉRIFICATION 

#~ Pose de référence : [-2,-5]
#~ Résolution avec Ibex :
""" test_commands = [[math.degrees(0.681812), math.degrees(2.553460)],
                  [math.degrees(0.681812), math.degrees(-2.132572)+360],
                  [math.degrees(-1.183374), math.degrees(2.553460)],
                  [math.degrees(-1.183374), math.degrees(-2.132572)+360]] """
#~ test_commands[0] : [-1.99957072, -5.0015154 ]
#~ test_commands[1] : [-2.00608548, -5.01530764]
#~ test_commands[2] : [-1.99453215, -4.99840006]
#~ test_commands[3] : [-1.24331322, -26.38964046] Impossible en mode 0

#~ Autre pose : [-10,7]
#~ Résolution avec Ibex :
""" test_commands = [[math.degrees(1.654949), math.degrees(2.527771)],
                     [math.degrees(1.654949), math.degrees(-2.94304635)+360],
                     [math.degrees(-0.663802), math.degrees(2.527771)],
                     [math.degrees(-0.663802), math.degrees(-2.943046)+360]] """
#~ test_commands[0] : [-10.00064325, 6.97691621]
#~ test_commands[1] : [-10.08399857, 6.8707291 ]
#~ test_commands[2] : [ 8.86690442, -7.09312602] Impossible en mode 0
#~ test_commands[3] : [  5.79611211, -21.16148527] Impossible en mode 0
