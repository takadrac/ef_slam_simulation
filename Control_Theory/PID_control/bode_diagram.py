import numpy as np

from model import *
from step_response import *


def bodeplot_set(fig_ax, *args):
    fig_ax[0].grid(which="both", ls=':')
    fig_ax[0].set_ylabel('Gain [dB]')

    fig_ax[1].grid(which="both", ls=':')
    fig_ax[1].set_xlabel('$\omega$ [rad/s]')
    fig_ax[1].set_ylabel('Phase [deg]')

    if len(args) > 0:
        fig_ax[1].legend(loc=args[0])
    if len(args) > 1:
        fig_ax[0].legend(loc=args[1])

LS = linestyle_generator()
fig, ax = plt.subplots(2, 1)

for i in range(len(kp)):
    K = tf([0, kp[i]], [0, 1])
    Gyr = feedback(P*K, 1)

    gain, phase, w = bode(Gyr, logspace(-1, 2), Plot=False)

    pltargs = {'ls': next(LS), 'label': '$k_P$='+str(kp[i])}
    ax[0].semilogx(w, 20*np.log10(gain), **pltargs)
    ax[1].semilogx(w, np.degrees(phase), **pltargs)

bodeplot_set(ax, 'lower left')
plt.show()