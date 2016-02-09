#!/usr/bin/env python

from perspective_taking_python.srv import *
import roslib
roslib.load_manifest('perspective_taking_python')
import rospy
import sympy as sp
from sympy.solvers import solve
import numpy as np
import tf
print 'Quaternion:', tf.transformations.quaternion_from_euler(0,0,90/180.0*np.pi)
threshold=0.00005

def func(th, req, idx):
    n = np.array([req.n.x, req.n.y, req.n.z])
    c = np.array([req.c.x, req.c.y, req.c.z])
    h = np.array([req.h.x, req.h.y, req.h.z])
    u = np.array([req.u.x, req.u.y, req.u.z])

    return req.radius*sp.cos(th*np.pi)*u[idx] + req.radius*sp.sin(th*np.pi)*np.cross(n, u)[idx] + c[idx]

def handle_circle_equation(req):
    th = sp.Symbol('x')

    eq0 = sp.Eq(func(th, req, 0), req.h.x)
    eq1 = sp.Eq(func(th, req, 1), req.h.y)
    eq2 = sp.Eq(func(th, req, 2), req.h.z)

    s0=solve(eq0, th)
    s1=solve(eq1, th)
    s2=solve(eq2, th)

    allsolutions = s0+s1+s2
    print 's0: ', s0
    print 's1: ', s1
    print 's2: ', s2
    noduplicates=[]
    for solution in allsolutions:
        newsolution=True
        for existingsolution in noduplicates:
            if solution>1.0:
                solution-=2
            if solution<-1.0:
                solution+=2
            if abs(existingsolution-solution) < threshold:
                newsolution=False
                break
        if newsolution:
            noduplicates.append(solution);

    print noduplicates
    actualsolutions=[]
    for solution in noduplicates:
         if abs(func(solution, req, 0)-req.h.x) < threshold and abs(func(solution, req, 1)-req.h.y) < threshold and abs(func(solution, req, 2)-req.h.z) < threshold:
             actualsolutions.append(solution)

    print 'actual solutions: ', actualsolutions
    return CircleEquationResponse(actualsolutions[0])

def solve_circle_equation_server():
    s = rospy.Service('solve_circle_equation', CircleEquation, handle_circle_equation)
    print "Ready to solve circle equations."
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('solve_circle_equation_server')
    solve_circle_equation_server()
