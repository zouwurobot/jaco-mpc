#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from std_msgs.msg import String
import tf
from math import *
import numpy as np
from cvxopt import *
from kinova_planning.srv import *



def QP_solver(req):
	P = 2*matrix([[1.0, 0, 0, 0, 0, 0], [0, 1.0, 0, 0, 0, 0], [0, 0, 1.0, 0, 0, 0], [0, 0, 0, 1.0, 0, 0], [0, 0, 0, 0, 1.0, 0], [0, 0, 0, 0, 0, 1.0]])
	q = -2*matrix([req.U01, req.U02, req.U03, req.U04, req.U05, req.U06])
	G = matrix([[req.G1, req.G1, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0], [req.G2, req.G2, 0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0], 
		        [req.G3, req.G3, 0, 0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0], [req.G4, req.G4, 0, 0, 0, 1, 0, 0, 0, 0, 0, -1, 0, 0],
		        [req.G5, req.G5, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, -1, 0], [req.G6, req.G6, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, -1]])

	h = matrix([req.h1, req.h2, req.v_max, req.v_max, req.v_max, req.v_max, req.v_max, req.v_max, req.v_max, req.v_max, req.v_max, req.v_max, req.v_max, req.v_max])

	sol=solvers.qp(P, q, G, h)
	return [sol['x'][0], sol['x'][1], sol['x'][2], sol['x'][3], sol['x'][4], sol['x'][5]] 
    


    

if __name__ == '__main__':

    rospy.init_node('qp_solver_server', anonymous=True)

    s = rospy.Service('solve_qp', SolveQP, QP_solver)
    print "Ready to solve QP problems."
    rospy.spin()

