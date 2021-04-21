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
#	==> Function: F19 - External Forces
#
#	==> Git hash: e4323cc50f813318b5c64b3b53763c941a34ce32
#

from math import sin, cos, sqrt
from numpy import zeros

def extforces(frc, trq, s, tsim):
    q = s.q
    qd = s.qd
    qdd = s.qdd
    frc = s.frc
    trq = s.trq
    PxF1 = zeros(4)
    RxF1 = zeros((4, 4))
    VxF1 = zeros(4)
    OMxF1 = zeros(4)
    AxF1 = zeros(4)
    OMPxF1 = zeros(4)

 
# Trigonometric functions

    S2 = sin(q[2])
    C2 = cos(q[2])
    S3 = sin(q[3])
    C3 = cos(q[3])
    C2p3 = C2*C3-S2*S3
    S2p3 = C2*S3+S2*C3
 
# Augmented Joint Position Vectors

 
# Sensor Kinematics

    OMcp1_33 = qd[2]+qd[3]
    OPcp1_33 = qdd[2]+qdd[3]
    PxF1[1] = 0
    PxF1[2] = q[1]
    PxF1[3] = 0
    RxF1[1,1] = C2p3
    RxF1[1,2] = S2p3
    RxF1[2,1] = -S2p3
    RxF1[2,2] = C2p3
    RxF1[3,3] = (1.0)
    VxF1[1] = 0
    VxF1[2] = qd[1]
    VxF1[3] = 0
    OMxF1[1] = 0
    OMxF1[2] = 0
    OMxF1[3] = OMcp1_33
    AxF1[1] = 0
    AxF1[2] = qdd[1]
    AxF1[3] = 0
    OMPxF1[1] = 0
    OMPxF1[2] = 0
    OMPxF1[3] = OPcp1_33
 
# Sensor Forces 

    SWr1 = s.user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1)
    xfrc11 = RxF1[1,1]*SWr1[1]+RxF1[1,2]*SWr1[2]
    xfrc21 = RxF1[2,1]*SWr1[1]+RxF1[2,2]*SWr1[2]
    xfrc31 = RxF1[3,3]*SWr1[3]
    xtrq11 = RxF1[1,1]*SWr1[4]+RxF1[1,2]*SWr1[5]
    xtrq21 = RxF1[2,1]*SWr1[4]+RxF1[2,2]*SWr1[5]
    xtrq31 = RxF1[3,3]*SWr1[6]
    trqext_1_3_0 = xtrq11-xfrc21*SWr1[9]+xfrc31*SWr1[8]
    trqext_2_3_0 = xtrq21+xfrc11*SWr1[9]-xfrc31*SWr1[7]
    trqext_3_3_0 = xtrq31-xfrc11*SWr1[8]+xfrc21*SWr1[7]
 
# Symbolic model output

    frc[1,3] = s.frc[1,3]+xfrc11
    frc[2,3] = s.frc[2,3]+xfrc21
    frc[3,3] = s.frc[3,3]+xfrc31
    trq[1,3] = s.trq[1,3]+trqext_1_3_0
    trq[2,3] = s.trq[2,3]+trqext_2_3_0
    trq[3,3] = s.trq[3,3]+trqext_3_3_0

# Number of continuation lines = 0


