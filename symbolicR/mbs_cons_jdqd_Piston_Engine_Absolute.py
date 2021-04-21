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
#	==> Generation Date: Wed Apr 21 08:00:53 2021
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
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

 
# Constraints Quadratic Terms

    ROjdqd1_82 = -C2*S3-S2*C3
    ROjdqd1_92 = C2*C3-S2*S3
    RLjdqd1_22 = -s.dpt[3,1]*S2
    RLjdqd1_32 = s.dpt[3,1]*C2
    OMjdqd1_12 = qd[2]+qd[3]
    ORjdqd1_22 = -RLjdqd1_32*qd[2]
    ORjdqd1_32 = RLjdqd1_22*qd[2]
    Apqpjdqd1_22 = -ORjdqd1_32*qd[2]
    Apqpjdqd1_32 = ORjdqd1_22*qd[2]
    RLjdqd1_23 = ROjdqd1_82*s.dpt[3,2]
    RLjdqd1_33 = ROjdqd1_92*s.dpt[3,2]
    ORjdqd1_23 = -OMjdqd1_12*RLjdqd1_33
    ORjdqd1_33 = OMjdqd1_12*RLjdqd1_23
    Apqpjdqd1_23 = Apqpjdqd1_22-OMjdqd1_12*ORjdqd1_33
    Apqpjdqd1_33 = Apqpjdqd1_32+OMjdqd1_12*ORjdqd1_23
    Jdqd[1] = Apqpjdqd1_23
    Jdqd[2] = Apqpjdqd1_33

# Number of continuation lines = 0


