import numpy as np

"""
-----------------------
Magic Numbers
-----------------------
"""

deg2rad = np.pi / 180
rad2deg = 1 / deg2rad
ksi2psi = 1000
psi2ksi = 1 / ksi2psi
ksi2pa = 6.895E6
pa2ksi = 1 / ksi2pa
psi2pa = 0.000145038
pa2psi = 1 / psi2pa
g2N = 0.009807
N2g = 1 / g2N
N2lbf = 0.2248089431
lbf2N = 1 / N2lbf
m2in = 39.37
in2m = 1 / 39.37
m2mm = 1000
mm2m = 1 / 1000
mm2in = mm2m * m2in
in2mm = 1 / mm2in
g2lbf = g2N * N2lbf
lbf2g = 1 / g2lbf
kgpm32lbpin3 = 27680
lbpin32kgpm3 = 1 / kgpm32lbpin3
kg2lb = 2.205
lb2kg = 1 / kg2lb