#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Script to run a direct dynamic analysis on a multibody system.

Summary
-------
This template loads the data file *.mbs and execute:
 - the coordinate partitioning module
 - the direct dynamic module (time integration of equations of motion).
 - if available, plot the time evolution of the first generalized coordinate.

It may have to be adapted and completed by the user.


Universite catholique de Louvain
CEREM : Centre for research in mechatronics

http://www.robotran.eu
Contact : info@robotran.be

(c) Universite catholique de Louvain
"""

# %%============================================================================
# Packages loading
# =============================================================================
try:
    import MBsysPy as Robotran
except:
    raise ImportError("MBsysPy not found/installed."
                      "See: https://www.robotran.eu/download/how-to-install/"
                      )

# %%===========================================================================
# Project loading
# =============================================================================
mbs_data = Robotran.MbsData('../dataR/Piston_Engine_Absolute.mbs')

# %%===========================================================================
# Partitionning
# =============================================================================
mbs_data.process = 1
mbs_part = Robotran.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=1)
mbs_part.run()

#%%==============================================================================
# Static Equilibrium
#==============================================================================
#mbs_data.process = 2
#mbs_equil = Robotran.MbsEquil(mbs_data)
#mbs_equil.set_options(method = 1, senstol = 1e-2, verbose = 1,soft = 0, itermax =1000, mode =1,equitol=1e-10)
#Forcer les roues pour l'equilibre
#mbs_equil.run()
# %%===========================================================================
# Direct Dynamics
# =============================================================================
mbs_data.process = 3
mbs_dirdyn = Robotran.MbsDirdyn(mbs_data)
mbs_dirdyn.set_options(dt0=1e-3, tf=3.0, save2file=1)
results = mbs_dirdyn.run()

# %%===========================================================================
# Plotting results
# =============================================================================
print('passed Simulation')
try:
    import matplotlib.pyplot as plt
except Exception:
    raise RuntimeError('Unable to load matplotlib, plotting results unavailable.')

# Figure creation
fig = plt.figure(num='Example of plot')
axis = fig.gca()

# Plotting data's
axis.plot(results.q[:, 0], results.q[:, 1], label='q[1]')

# Figure enhancement
axis.grid(True)
axis.set_xlim(left=mbs_dirdyn.get_options('t0'), right=mbs_dirdyn.get_options('tf'))
axis.set_xlabel('Time (s)')
axis.set_ylabel('Coordinate value (m or rad)')

plt.show()
print('passed Plotting')
