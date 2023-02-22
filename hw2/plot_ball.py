from matplotlib import pyplot as plt
import numpy as np

eps = np.array([0.2, 0.1, 0.01])
d = np.arange(2,10)

fig = plt.figure()
gs = fig.add_gridspec(3, hspace=0.4)
axs = gs.subplots(sharex=True)

#fig, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
fig.suptitle(r'Volume of $\eta_d(\epsilon)$ as a function of d')
y = (1 - np.power(1-eps[0], d))
axs[0].plot(d, y)
axs[0].set_title(r'$\epsilon$=0.2')

y = (1 - np.power(1-eps[1], d))
axs[1].plot(d, y)
axs[1].set_title(r'$\epsilon$=0.1')

y = (1 - np.power(1-eps[2], d))
axs[2].plot(d, y)
axs[2].set_title(r'$\epsilon$=0.01')

plt.show()
