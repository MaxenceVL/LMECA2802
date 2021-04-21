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
#	==> Function: F6 - Sensors Kinematics
#
#	==> Git hash: e4323cc50f813318b5c64b3b53763c941a34ce32
#

from math import sin, cos, sqrt

def sensor(sens, s, isens):
  q = s.q
  qd = s.qd
  qdd = s.qdd

  dpt = s.dpt
 
# Trigonometric functions

  S2 = sin(q[2])
  C2 = cos(q[2])
  S3 = sin(q[3])
  C3 = cos(q[3])
  C2p3 = C2*C3-S2*S3
  S2p3 = C2*S3+S2*C3
 
# Augmented Joint Position Vectors

 
# Sensor Kinematics


  if (isens == 1): 

    OMcp1_33 = qd[2]+qd[3]
    OPcp1_33 = qdd[2]+qdd[3]
    sens.P[1] = 0
    sens.P[2] = q[1]
    sens.P[3] = 0
    sens.R[1,1] = C2p3
    sens.R[1,2] = S2p3
    sens.R[2,1] = -S2p3
    sens.R[2,2] = C2p3
    sens.R[3,3] = (1.0)
    sens.V[1] = 0
    sens.V[2] = qd[1]
    sens.V[3] = 0
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = OMcp1_33
    sens.A[1] = 0
    sens.A[2] = qdd[1]
    sens.A[3] = 0
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = OPcp1_33

 


# Number of continuation lines = 0


