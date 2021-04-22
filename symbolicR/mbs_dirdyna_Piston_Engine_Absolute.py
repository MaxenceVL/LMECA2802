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
#	==> Generation Date: Wed Apr 21 10:30:49 2021
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
 
# Forward Kinematics

    BS92 = -qd[2]*qd[2]
    AF22 = -s.g[3]*S2
    AF32 = -s.g[3]*C2
    OM13 = qd[2]+qd[3]
    BS93 = -OM13*OM13
    AF23 = AF22*C3+S3*(AF32+BS92*s.dpt[3,1])
    AF33 = -AF22*S3+C3*(AF32+BS92*s.dpt[3,1])
    AM23_1 = C2*S3+S2*C3
    AM33_1 = C2*C3-S2*S3
    AM23_2 = -s.dpt[3,1]*C3
    AM33_2 = s.dpt[3,1]*S3
 
# Backward Dynamics

    FA34 = -s.frc[3,4]-s.m[4]*s.g[3]
    FA23 = -s.frc[2,3]+s.m[3]*AF23
    FA33 = -s.frc[3,3]+s.m[3]*(AF33+BS93*s.l[3,3])
    CF13 = -s.trq[1,3]-FA23*s.l[3,3]
    FB23_1 = s.m[3]*AM23_1
    FB33_1 = s.m[3]*AM33_1
    CM13_1 = -FB23_1*s.l[3,3]
    FB23_2 = s.m[3]*(AM23_2-s.l[3,3])
    FB33_2 = s.m[3]*AM33_2
    CM13_2 = s.In[1,3]-FB23_2*s.l[3,3]
    FB23_3 = -s.m[3]*s.l[3,3]
    CM13_3 = s.In[1,3]-FB23_3*s.l[3,3]
    FA22 = -s.frc[2,2]+s.m[2]*AF22
    FA32 = -s.frc[3,2]+s.m[2]*(AF32+BS92*s.l[3,2])
    FF22 = FA22+FA23*C3-FA33*S3
    FF32 = FA32+FA23*S3+FA33*C3
    CF12 = -s.trq[1,2]+CF13-FA22*s.l[3,2]-s.dpt[3,1]*(FA23*C3-FA33*S3)
    FB22_1 = s.m[2]*S2
    FB32_1 = s.m[2]*C2
    FM22_1 = FB22_1+FB23_1*C3-FB33_1*S3
    FM32_1 = FB32_1+FB23_1*S3+FB33_1*C3
    CM12_1 = CM13_1-FB22_1*s.l[3,2]-s.dpt[3,1]*(FB23_1*C3-FB33_1*S3)
    FB22_2 = -s.m[2]*s.l[3,2]
    CM12_2 = s.In[1,2]+CM13_2-FB22_2*s.l[3,2]-s.dpt[3,1]*(FB23_2*C3-FB33_2*S3)
    FA31 = -s.frc[3,1]-s.m[1]*s.g[3]
    FF31 = FA31+FA34+FF22*S2+FF32*C2
    FM31_1 = s.m[1]+s.m[4]+FM22_1*S2+FM32_1*C2
 
# Symbolic model output

    c[1] = FF31
    c[2] = CF12
    c[3] = CF13
    c[4] = FA34
    M[1,1] = FM31_1
    M[1,2] = CM12_1
    M[1,3] = CM13_1
    M[1,4] = s.m[4]
    M[2,1] = CM12_1
    M[2,2] = CM12_2
    M[2,3] = CM13_2
    M[3,1] = CM13_1
    M[3,2] = CM13_2
    M[3,3] = CM13_3
    M[4,1] = s.m[4]
    M[4,4] = s.m[4]

# Number of continuation lines = 0


