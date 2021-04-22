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

    ALPHA31 = qdd[1]-s.g[3]
    BS92 = -qd[2]*qd[2]
    ALPHA22 = ALPHA31*S2
    ALPHA32 = ALPHA31*C2
    OM13 = qd[2]+qd[3]
    OMp13 = qdd[2]+qdd[3]
    BS93 = -OM13*OM13
    ALPHA23 = C3*(ALPHA22-qdd[2]*s.dpt[3,1])+S3*(ALPHA32+BS92*s.dpt[3,1])
    ALPHA33 = C3*(ALPHA32+BS92*s.dpt[3,1])-S3*(ALPHA22-qdd[2]*s.dpt[3,1])
    ALPHA34 = qdd[4]+ALPHA31
 
# Backward Dynamics

    Fs34 = -s.frc[3,4]+s.m[4]*ALPHA34
    Fs23 = -s.frc[2,3]+s.m[3]*(ALPHA23-OMp13*s.l[3,3])
    Fs33 = -s.frc[3,3]+s.m[3]*(ALPHA33+BS93*s.l[3,3])
    Cq13 = -s.trq[1,3]+s.In[1,3]*OMp13-Fs23*s.l[3,3]
    Fs22 = -s.frc[2,2]+s.m[2]*(ALPHA22-qdd[2]*s.l[3,2])
    Fs32 = -s.frc[3,2]+s.m[2]*(ALPHA32+BS92*s.l[3,2])
    Fq22 = Fs22+Fs23*C3-Fs33*S3
    Fq32 = Fs32+Fs23*S3+Fs33*C3
    Cq12 = -s.trq[1,2]+Cq13+qdd[2]*s.In[1,2]-Fs22*s.l[3,2]-s.dpt[3,1]*(Fs23*C3-Fs33*S3)
    Fs31 = -s.frc[3,1]+s.m[1]*ALPHA31
    Fq31 = Fs31+Fs34+Fq22*S2+Fq32*C2
 
# Symbolic model output

    Qq[1] = Fq31
    Qq[2] = Cq12
    Qq[3] = Cq13
    Qq[4] = Fs34

# Number of continuation lines = 0


