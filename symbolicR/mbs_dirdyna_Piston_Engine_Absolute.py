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
#	==> Function: F1 - Recursive Direct Dynamics of tree-like MBS
#
#	==> Git hash: e4323cc50f813318b5c64b3b53763c941a34ce32
#

from math import sin, cos

def dirdyna(M, c, s, tsim):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S2 = sin(q[2])
    C2 = cos(q[2])
    S3 = sin(q[3])
    C3 = cos(q[3])
    S2p3 = C2*S3+S2*C3
    C2p3 = C2*C3-S2*S3
 
# Forward Kinematics

    BS12 = -qd[2]*qd[2]
    AF13 = BS12*s.dpt[1,1]*C3
    AF23 = -BS12*s.dpt[1,1]*S3
    AM13_2 = s.dpt[1,1]*S3
    AM23_2 = s.dpt[1,1]*C3
 
# Backward Dynamics

    FA13 = -s.frc[1,3]+s.m[3]*AF13
    FA23 = -s.frc[2,3]+s.m[3]*AF23
    FB13_1 = s.m[3]*S2p3
    FB23_1 = s.m[3]*C2p3
    FB13_2 = s.m[3]*AM13_2
    FB23_2 = s.m[3]*AM23_2
    FF12 = -s.frc[1,2]+FA13*C3-FA23*S3
    FF22 = -s.frc[2,2]+FA13*S3+FA23*C3
    CF32 = -s.trq[3,2]-s.trq[3,3]+s.dpt[1,1]*(FA13*S3+FA23*C3)
    FB12_1 = s.m[2]*S2
    FB22_1 = s.m[2]*C2
    FM12_1 = FB12_1+FB13_1*C3-FB23_1*S3
    FM22_1 = FB22_1+FB13_1*S3+FB23_1*C3
    CM32_1 = s.dpt[1,1]*(FB13_1*S3+FB23_1*C3)
    CM32_2 = s.dpt[1,1]*(FB13_2*S3+FB23_2*C3)
    FF21 = -s.frc[2,1]-s.frc[2,4]+FF12*S2+FF22*C2
    FM21_1 = s.m[1]+s.m[4]+FM12_1*S2+FM22_1*C2
 
# Symbolic model output

    c[1] = FF21
    c[2] = CF32
    c[3] = -s.trq[3,3]
    c[4] = -s.frc[2,4]
    M[1,1] = FM21_1
    M[1,2] = CM32_1
    M[1,4] = s.m[4]
    M[2,1] = CM32_1
    M[2,2] = CM32_2
    M[4,1] = s.m[4]
    M[4,4] = s.m[4]

# Number of continuation lines = 0


