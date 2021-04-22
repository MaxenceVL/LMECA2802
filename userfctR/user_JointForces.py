# -*- coding: utf-8 -*-
"""Module for the definition of joint forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020


def user_JointForces(mbs_data, tsim):
    """Compute the force and torques in the joint.

    It fills the MBsysPy.MbsData.Qq array.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.

    Notes
    -----
    The numpy.ndarray MBsysPy.MbsData.Qq is 1D array with index starting at 1.
    The first index (array[0]) must not be modified. The first index to be
    filled is array[1].

    Returns
    -------
    None
    """
    # cleaning previous forces value
    mbs_data.Qq[1:] = 0.

    T3Piston = mbs_data.joint_id['T3Piston']
    T3Blok  = mbs_data.joint_id['T3Blok']
    R1Crankshaft  = mbs_data.joint_id['R1Crankshaft']
    # Example: damping in joint number 5
    # D = 0.5 # N/(m/s)
    # mbs_data.Qq[5] = -D * mbs_data.qd[5]
    tau = 10
    mbs_data.Qq[T3Piston] = - tau * mbs_data.qd[T3Piston]
    K = 30000
    D = 5000
    L0 = 0.2
    mbs_data.Qq[T3Blok] = - ( K*(mbs_data.q[T3Blok]-L0) + D*mbs_data.qd[T3Blok] )
    mbs_data.Qq[R1Crankshaft] = 3
    return
