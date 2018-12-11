#######################################################
# TwoBars robot demo
import math
import numpy

#~ from importlib import reload
#~ import demo,robot

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

#~ commands = [[q1,q2] for q1 in range(-90,90,10) for q2 in range(90,270,10)]
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
#~ Mesure attendue : [-3.5,-15]
#~ test_commands = [[0.13798,4.25566], [0.13798,3.09180], [-1.49167,4.25566],[-1.49167,3.09180]]
##################################################
# Trajectory following

"""
def make_pose(r2,t):
    x1 = 2 + (2.5*sin(t))/(1 + cos(t)^2)
    x2 = 2 + (2*sin(t)*cos(t))/(1 + cos(t)^2)

    return pose

def make_commands(r2, architecture,pose,command):
    [a1,a2,a3,a4] = architecture
    pose = []
    commands = []

    for t in T
        pose.append(make_pose(r2,t))
        
        q = r2.measure_command()
        
        r2.actuate(q)
        r2.ax.plot([x[0]],[x[1]],color=col,marker=mar)
        commands.append(q)



    #f1 = a1+a3*numpy.cos(q1)+a4*numpy.cos(q2) - x1
    #f2 = a2+a3*numpy.sin(q1)+a4*numpy.sin(q2) - x2
    return commands
"""
