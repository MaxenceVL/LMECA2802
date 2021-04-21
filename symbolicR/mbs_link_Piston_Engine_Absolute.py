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
#	==> Generation Date: Wed Apr 14 18:09:37 2021
#
#	==> Project name: Piston_Engine_Absolute
#
#	==> Number of joints: 3
#
#	==> Function: F7 - Link Forces (1D)
#
#	==> Git hash: e4323cc50f813318b5c64b3b53763c941a34ce32
#

from math import sin, cos, sqrt

def link(frc, trq, Flink, Z, Zd, s, tsim):
    q = s.q
    qd = s.qd
 
# Augmented Joint Position Vectors

 
# Link Kinematics: displacement (Z) , relative velocity (Zd)

    PPlnk1 = q[1]*q[1]
    Z1 = sqrt(PPlnk1)
    e21 = -q[1]/Z1
    Zd1 = -qd[1]*e21

# Link Forces 

    Flink1 = s.user_LinkForces(Z1,Zd1,s,tsim,1)
 
# Link Dynamics: forces projection on body-fixed frames

    fPlnk21 = Flink1*e21
 
# Symbolic model output

    frc[2,1] = s.frc[2,1]+fPlnk21
 
# Symbolic model output

    Z[1] = Z1
    Zd[1] = Zd1
    Flink[1] = Flink1

# Number of continuation lines = 0


