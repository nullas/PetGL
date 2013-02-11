from __future__ import division
#from mayavi import mlab as pl
from matplotlib import pylab as pl
import numpy as np
from scipy.optimize import brentq

beta = 0.05
L = 10
p0 = np.r_[-1,0]
p1 = np.r_[+1,0]
M = 300
mL = 100
w = p1 - p0
l = np.linalg.norm(p1 - p0)
w = w / l
v = w.copy()
v[0] = -w[1]
v[1] = w[0]

def f(R, l ,L):
    return 2 * (np.pi - np.arcsin(l / 2 / R)) * R - L

def g(R, l, L):
    return 2 * np.arcsin(l / 2 / R) * R - L

if (L > np.pi * l / 2):
    R = brentq(f, l/2, mL, args=(l,L))
    theta = np.arcsin(l / 2 / R)
    center = R * (v * np.cos(theta) + w * np.sin(theta)) + p0
    t = np.linspace(-np.pi / 2 + theta, np.pi / 2 * 3 - theta, M)
else:
    R = brentq(g, l/2, mL, args=(l,L))
    theta = np.arcsin(l / 2 / R)
    center = R * (-v * np.cos(theta) + w * np.sin(theta)) + p0
    t = np.linspace(np.pi / 2 - theta, np.pi / 2  + theta, M)


x = R * np.cos(t) + center[0]
y = R * np.sin(t) + center[1]
l = np.sqrt((x[0] - x[1])**2 + (y[0] - y[1])**2)

z0 = l * (-w * np.cos(beta) + v * np.sin(beta)) + p0
z1 = l * (w * np.cos(beta) + v * np.sin(beta)) + p1
    

fp = open('arc.ply','w')
fp.write('''ply
format ascii 1.0
comment made by Xin\n''')
n_vertices = M
fp.write('element vertex {0}\n'.format(n_vertices))
fp.write('''property float64 x
property float64 y
property float64 z\n''')
fp.write('element face {0}\n'.format(1))
fp.write('property list uint8 int32 vertex_index\n')
fp.write('end_header\n')
for i in xrange(M):
    fp.write('{0} {1} {2}\n'.format(x[i], y[i], 0))
fp.write('{0}'.format(M))
for i in xrange(M):
    fp.write(' {0}'.format(i))
fp.write('\n')
fp.close()
fp = open('arc.crv','w')
fp.write('{0} {1}\n'.format(M - 1, 0))
fp.close()

pl.figure()
pl.plot(x, y)
pl.axis('equal')
pl.show()

