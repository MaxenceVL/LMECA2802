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
#	==> Function: F2 - Recursive Inverse Dynamics of tree-like MBS
#
#	==> Git hash: e4323cc50f813318b5c64b3b53763c941a34ce32
#

from math import sin, cos

def invdyna(phi,s,tsim):
    Qq = phi  # compatibility with symbolic generation
    q = s.q
    qd = s.qd
    qdd = s.qdd
 
# Trigonometric functions

    S2 = sin(q[2])
    C2 = cos(q[2])
    S3 = sin(q[3])
    C3 = cos(q[3])
 
# Augmented Joint Position Vectors

 
# Forward Kinematics

    BS12 = -qd[2]*qd[2]
    ALPHA12 = qdd[1]*S2
    ALPHA22 = qdd[1]*C2
    ALPHA13 = C3*(ALPHA12+BS12*s.dpt[1,1])+S3*(ALPHA22+qdd[2]*s.dpt[1,1])
    ALPHA23 = C3*(ALPHA22+qdd[2]*s.dpt[1,1])-S3*(ALPHA12+BS12*s.dpt[1,1])
    ALPHA24 = qdd[1]+qdd[4]
 
# Backward Dynamics

    Fs24 = -s.frc[2,4]+s.m[4]*ALPHA24
    Fs13 = -s.frc[1,3]+s.m[3]*ALPHA13
    Fs23 = -s.frc[2,3]+s.m[3]*ALPHA23
    Fs12 = -s.frc[1,2]+s.m[2]*ALPHA12
    Fs22 = -s.frc[2,2]+s.m[2]*ALPHA22
    Fq12 = Fs12+Fs13*C3-Fs23*S3
    Fq22 = Fs22+Fs13*S3+Fs23*C3
    Cq32 = -s.trq[3,2]-s.trq[3,3]+s.dpt[1,1]*(Fs13*S3+Fs23*C3)
    Fs21 = -s.frc[2,1]+qdd[1]*s.m[1]
    Fq21 = Fs21+Fs24+Fq12*S2+Fq22*C2
 
# Symbolic model output

    Qq[1] = Fq21
    Qq[2] = Cq32
    Qq[3] = -s.trq[3,3]
    Qq[4] = Fs24

# Number of continuation lines = 0


