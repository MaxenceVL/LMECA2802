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

    sens.P[1] = 0
    sens.P[2] = q[1]
    sens.P[3] = 0
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = 0
    sens.V[2] = qd[1]
    sens.V[3] = 0
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,1] = (1.0)
    sens.A[1] = 0
    sens.A[2] = qdd[1]
    sens.A[3] = 0
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 2): 

    sens.P[1] = 0
    sens.P[2] = q[1]
    sens.P[3] = 0
    sens.R[1,1] = C2
    sens.R[1,2] = S2
    sens.R[2,1] = -S2
    sens.R[2,2] = C2
    sens.R[3,3] = (1.0)
    sens.V[1] = 0
    sens.V[2] = qd[1]
    sens.V[3] = 0
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = qd[2]
    sens.J[2,1] = (1.0)
    sens.J[6,2] = (1.0)
    sens.A[1] = 0
    sens.A[2] = qdd[1]
    sens.A[3] = 0
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = qdd[2]

  if (isens == 3): 

    RLcp3_13 = s.dpt[1,1]*C2
    RLcp3_23 = s.dpt[1,1]*S2
    POcp3_23 = RLcp3_23+q[1]
    OMcp3_33 = qd[2]+qd[3]
    ORcp3_13 = -RLcp3_23*qd[2]
    ORcp3_23 = RLcp3_13*qd[2]
    VIcp3_23 = ORcp3_23+qd[1]
    OPcp3_33 = qdd[2]+qdd[3]
    ACcp3_13 = -ORcp3_23*qd[2]-RLcp3_23*qdd[2]
    ACcp3_23 = qdd[1]+ORcp3_13*qd[2]+RLcp3_13*qdd[2]
    sens.P[1] = RLcp3_13
    sens.P[2] = POcp3_23
    sens.P[3] = 0
    sens.R[1,1] = C2p3
    sens.R[1,2] = S2p3
    sens.R[2,1] = -S2p3
    sens.R[2,2] = C2p3
    sens.R[3,3] = (1.0)
    sens.V[1] = ORcp3_13
    sens.V[2] = VIcp3_23
    sens.V[3] = 0
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = OMcp3_33
    sens.J[1,2] = -RLcp3_23
    sens.J[2,1] = (1.0)
    sens.J[2,2] = RLcp3_13
    sens.J[6,2] = (1.0)
    sens.J[6,3] = (1.0)
    sens.A[1] = ACcp3_13
    sens.A[2] = ACcp3_23
    sens.A[3] = 0
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = OPcp3_33

  if (isens == 4): 

    POcp4_22 = q[1]+q[4]
    VIcp4_22 = qd[1]+qd[4]
    ACcp4_22 = qdd[1]+qdd[4]
    sens.P[1] = 0
    sens.P[2] = POcp4_22
    sens.P[3] = 0
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = 0
    sens.V[2] = VIcp4_22
    sens.V[3] = 0
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,1] = (1.0)
    sens.J[2,4] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp4_22
    sens.A[3] = 0
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

 


# Number of continuation lines = 0


