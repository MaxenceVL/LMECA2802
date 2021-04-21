# -*- coding: utf-8 -*-
"""Module for the definition of user links forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020


def user_LinkForces(Z, Zd, mbs_data, tsim, identity):
    """Compute the force in the given link.

    Parameters
    ----------
    Z : float
        The distance between the two anchor points of the link.
    Zd : float
        The relative velocity between the two anchor points of the link.
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.
    identity : int
        The identity of the computed link.

    Returns
    -------
    Flink : float
        The force in the current link.

    """

    Flink = 0.0

    # Example: linear spring
    # k = 1000 #N/m
    # Z0= 0.1  #m
    # Flink = k*(Z-Z0)
    T2Blok = mbs_data.joint_id['T2Blok']

    K = 30000
    D = 5000
    L0 = 0.2
    mbs_data.Qq[T2Blok] = - ( K*(mbs_data.q[T2Blok]-L0) + D*mbs_data.qd[T2Blok] )
    return Flink
