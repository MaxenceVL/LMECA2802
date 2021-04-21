#
#	MBsysTran - Release 8.1
#
#	Copyright 
#	Universite catholique de Louvain (UCLouvain) 
#	Mechatronic, Electrical Energy, and Dynamic systems (MEED Division) 
#	2, Place du Levant
#	1348 Louvain-la-Neuve 
#	Belgium 
#
#	http://www.robotran.be 
#
#	==> Generation Date: Wed Apr 21 04:51:22 2021
#
#	==> Project name: Piston_Engine_Absolute
#
#	==> Number of joints: 4
#
#	==> Function: F8 - Constraints and Constraints Jacobian(h, J)
#
#	==> Git hash: e4323cc50f813318b5c64b3b53763c941a34ce32
#

from math import sin, cos

def cons_hJ(h, Jac, s):
    q = s.q
 
# Trigonometric functions

    S2 = sin(q[2])
    C2 = cos(q[2])
    S3 = sin(q[3])
    C3 = cos(q[3])
    C2p3 = C2*C3-S2*S3
    S2p3 = C2*S3+S2*C3
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

    RLlp1_12 = s.dpt[1,1]*C2
    RLlp1_22 = s.dpt[1,1]*S2
    RLlp1_13 = s.dpt[1,2]*C2p3
    RLlp1_23 = s.dpt[1,2]*S2p3
    POlp1_13 = RLlp1_12+RLlp1_13
    POlp1_23 = RLlp1_22+RLlp1_23
    JTlp1_13_1 = -RLlp1_22-RLlp1_23
    JTlp1_23_1 = RLlp1_12+RLlp1_13
    h_2 = POlp1_23-q[4]
    h[1] = POlp1_13
    h[2] = h_2
    Jac[1,1] = 0
    Jac[1,2] = JTlp1_13_1
    Jac[1,3] = -RLlp1_23
    Jac[1,4] = 0
    Jac[2,1] = 0
    Jac[2,2] = JTlp1_23_1
    Jac[2,3] = RLlp1_13
    Jac[2,4] = -(1.0)

# Number of continuation lines = 0


