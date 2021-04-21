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
#	==> Function: F18 - Constraints Quadratic Velocity Terms (Jdqd)
#
#	==> Git hash: e4323cc50f813318b5c64b3b53763c941a34ce32
#

from math import sin, cos

def cons_jdqd(Jdqd, s):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S2 = sin(q[2])
    C2 = cos(q[2])
    S3 = sin(q[3])
    C3 = cos(q[3])
    C2p3 = C2*C3-S2*S3
    S2p3 = C2*S3+S2*C3
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

 
# Constraints Quadratic Terms

    RLjdqd1_12 = s.dpt[1,1]*C2
    RLjdqd1_22 = s.dpt[1,1]*S2
    OMjdqd1_32 = qd[2]+qd[3]
    ORjdqd1_12 = -RLjdqd1_22*qd[2]
    ORjdqd1_22 = RLjdqd1_12*qd[2]
    Apqpjdqd1_12 = -ORjdqd1_22*qd[2]
    Apqpjdqd1_22 = ORjdqd1_12*qd[2]
    RLjdqd1_13 = s.dpt[1,2]*C2p3
    RLjdqd1_23 = s.dpt[1,2]*S2p3
    ORjdqd1_13 = -OMjdqd1_32*RLjdqd1_23
    ORjdqd1_23 = OMjdqd1_32*RLjdqd1_13
    Apqpjdqd1_13 = Apqpjdqd1_12-OMjdqd1_32*ORjdqd1_23
    Apqpjdqd1_23 = Apqpjdqd1_22+OMjdqd1_32*ORjdqd1_13
    Jdqd[1] = Apqpjdqd1_13
    Jdqd[2] = Apqpjdqd1_23

# Number of continuation lines = 0


