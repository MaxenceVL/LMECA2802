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
 
# Augmented Joint Position Vectors

 
# Sensor Kinematics


  if (isens == 1): 

    sens.P[1] = 0
    sens.P[2] = 0
    sens.P[3] = q[1]
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = 0
    sens.V[2] = 0
    sens.V[3] = qd[1]
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[3,1] = (1.0)
    sens.A[1] = 0
    sens.A[2] = 0
    sens.A[3] = qdd[1]
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 2): 

    sens.P[1] = 0
    sens.P[2] = 0
    sens.P[3] = q[1]
    sens.R[1,1] = (1.0)
    sens.R[2,2] = C2
    sens.R[2,3] = S2
    sens.R[3,2] = -S2
    sens.R[3,3] = C2
    sens.V[1] = 0
    sens.V[2] = 0
    sens.V[3] = qd[1]
    sens.OM[1] = qd[2]
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[3,1] = (1.0)
    sens.J[4,2] = (1.0)
    sens.A[1] = 0
    sens.A[2] = 0
    sens.A[3] = qdd[1]
    sens.OMP[1] = qdd[2]
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 3): 

    ROcp3_53 = C2*C3-S2*S3
    ROcp3_63 = C2*S3+S2*C3
    ROcp3_83 = -C2*S3-S2*C3
    ROcp3_93 = C2*C3-S2*S3
    RLcp3_23 = -s.dpt[3,1]*S2
    RLcp3_33 = s.dpt[3,1]*C2
    POcp3_33 = RLcp3_33+q[1]
    OMcp3_13 = qd[2]+qd[3]
    ORcp3_23 = -RLcp3_33*qd[2]
    ORcp3_33 = RLcp3_23*qd[2]
    VIcp3_33 = ORcp3_33+qd[1]
    OPcp3_13 = qdd[2]+qdd[3]
    ACcp3_23 = -ORcp3_33*qd[2]-RLcp3_33*qdd[2]
    ACcp3_33 = qdd[1]+ORcp3_23*qd[2]+RLcp3_23*qdd[2]
    sens.P[1] = 0
    sens.P[2] = RLcp3_23
    sens.P[3] = POcp3_33
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp3_53
    sens.R[2,3] = ROcp3_63
    sens.R[3,2] = ROcp3_83
    sens.R[3,3] = ROcp3_93
    sens.V[1] = 0
    sens.V[2] = ORcp3_23
    sens.V[3] = VIcp3_33
    sens.OM[1] = OMcp3_13
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,2] = -RLcp3_33
    sens.J[3,1] = (1.0)
    sens.J[3,2] = RLcp3_23
    sens.J[4,2] = (1.0)
    sens.J[4,3] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp3_23
    sens.A[3] = ACcp3_33
    sens.OMP[1] = OPcp3_13
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 4): 

    POcp4_32 = q[1]+q[4]
    VIcp4_32 = qd[1]+qd[4]
    ACcp4_32 = qdd[1]+qdd[4]
    sens.P[1] = 0
    sens.P[2] = 0
    sens.P[3] = POcp4_32
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = 0
    sens.V[2] = 0
    sens.V[3] = VIcp4_32
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[3,1] = (1.0)
    sens.J[3,4] = (1.0)
    sens.A[1] = 0
    sens.A[2] = 0
    sens.A[3] = ACcp4_32
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

 


# Number of continuation lines = 0


