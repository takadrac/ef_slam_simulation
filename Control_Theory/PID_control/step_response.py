from control import *
from control.matlab import *
import matplotlib.pyplot as plt

from model import *


def linestyle_generator():
    linestyle = ['-', '--', '-.', ':']
    lineID = 0
    while True:
        yield linestyle[lineID]
        lineID = (lineID + 1) % len(linestyle)

def plot_set(fig_ax, *args):
    fig_ax.set_xlabel(args[0])
    fig_ax.set_ylabel(args[1])
    fig_ax.grid(ls=':')
    if len(args)==3:
        fig_ax.legend(loc=args[2])

kp = (0.5, 1, 2)

LS = linestyle_generator()
fig, ax = plt.subplots()
for i in range(3):
    K = tf([0, kp[i]], [0, 1])
    Gyr = feedback(P*K, 1)
    y, t = step(Gyr, np.arange(0, 2, 0.01))

    pltargs = {'ls': next(LS), 'label': '$k_P$='+str(kp[i])}
    ax.plot(t, y*ref, **pltargs)

ax.axhline(ref, color='k', linewidth=0.5)
plot_set(ax, 't', 'y', 'best')
# plt.show()