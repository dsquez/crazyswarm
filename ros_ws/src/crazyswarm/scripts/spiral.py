#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import math

t = np.linspace(-15,15)

a = 0.1
b = 0.3
bt = b*t
x = a*np.exp(bt)*np.cos(t)
y = b*np.exp(bt)*np.sin(t)

zarray = 0.1*np.ones([len(x)])
x = np.reshape(x,[len(x)])
y = np.reshape(y,[len(y)])
exparray = np.array([x,y,zarray])
exparray = np.transpose(exparray)
#exparray = np.reshape(exparray,[len(x),3])
np.savetxt("spiral.csv", exparray, delimiter=",")
#As a path collection

'''from matplotlib.collections import LineCollection

fig, ax = plt.subplots()
r = np.arange(0, .075, 0.00001)
nverts = len(r)
theta = np.array(range(nverts)) * (2*np.pi)/(nverts-1)
theta = 90*np.pi*r
xoffset, yoffset = .5, .5
yy = 1.4*r * np.sin(theta) + yoffset
xx = r * np.cos(theta) + xoffset
spiral = zip(xx,yy)
collection = LineCollection([spiral], colors='k')
ax.add_collection(collection)
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)

# In polar coordinates

fig, ax = plt.subplots(subplot_kw=dict(polar=True, axisbg='none'), figsize=(.5,.5))
r = np.arange(0, 3.4, 0.01)
theta = 2*np.pi*r
ax.set_axis_off()
ax.grid(False)
ax.axes.get_xaxis().set_visible(False)
ax.axes.get_yaxis().set_visible(False)
ax.plot(theta, r, linewidth=2, color='k');
print 'theta',theta
plt.show()'''