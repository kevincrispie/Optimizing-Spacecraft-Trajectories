import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import cvxpy as cp

# --- Parameters --- #
h = 0.5
g = 9.8
m = 6e4
Fmax = 8e6
p0 = np.asarray([1.5e3, 1e3, 3e3])
v0 = np.asarray([-300, 400, -400])
alpha = 0.7
gamma = 4e-4
K = 40

# scale force constraint, reduce dynamic range for ECOS solver
force_scale = 1e-7

# --- Pose Convex Optimization Problem --- #
f = cp.Variable((3, K))
p = cp.Variable((3, K+1))
v = cp.Variable((3, K+1))

fuel_usage  = gamma*cp.sum([cp.norm(f[:, i], 2) for i in range(K)])
objective   = cp.Minimize(fuel_usage)
constraints = [p[:, 0] == p0,
               v[:, 0] == v0,
               p[:, K] == np.asarray([0., 0., 0.]),
               v[:, K] == np.asarray([0., 0., 0.])]
constraints += [p[:, i+1] == p[:, i] + (h/2.)*(v[:, i] + v[:, i+1]) for i in range(K)]
constraints += [v[:, i+1] == v[:, i] + (h/m)*f[:, i] - h*np.asarray([0., 0., g]) for i in range(K)]
constraints += [force_scale*cp.norm(f[:, i], 2) <= force_scale*Fmax for i in range(K)]
constraints += [p[2, i] >= alpha*cp.norm(p[0:2, i], 2) for i in range(K)]
prob = cp.Problem(objective, constraints)
result = prob.solve()

fig = plt.figure(figsize=(10, 8))
ax = fig.gca(projection='3d')

X = np.linspace(-np.min(p0), np.max(p0), num=20)
Y = np.linspace(-np.min(p0), np.max(p0), num=20)
X, Y = np.meshgrid(X, Y)
Z = alpha*np.sqrt(X**2+Y**2)
ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0)
ax.plot(p.value[0,:], p.value[1,:], p.value[2,:], color="blue", linewidth=2)
ax.set_xlabel('x [m]'), ax.set_ylabel('y [m]'), ax.set_zlabel('z [m]')
ax.set_title("Minimum Fuel Descent Trajectory")
ax.view_init(10, -145)
plt.show()