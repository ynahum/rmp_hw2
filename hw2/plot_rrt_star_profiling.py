from matplotlib import pyplot as plt
import numpy as np

eta = np.array([5, 6, 7, 8, 9, 10, 15, 20])
avg_time = np.array([48.35, 32.25, 40.56, 19.28, 52.47, 11.54, 4.95, 3.23])
avg_cost = np.array([521.68, 519.78, 521.79, 514.78, 525.71, 516.21, 521.88, 513.53])

fig = plt.figure()
gs = fig.add_gridspec(2, hspace=0.4)
axs = gs.subplots(sharex=True)

#fig, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
#fig.suptitle(r'Average time as a function of $\eta$')
axs[0].stem(eta, avg_time)
axs[0].set_title(r'Average time as a function of $\eta$')
axs[0].set_ylabel('Average Time')

axs0_line = axs[0].twinx()  # Create a secondary y-axis
axs0_line.plot(eta, avg_time, 'r-')
axs0_line.set_ylabel('Average Time (Line Plot)', color='r')
axs0_line.tick_params(axis='y', labelcolor='r')

axs[1].stem(eta, avg_cost)
axs[1].set_title(r'Average cost as a function of $\eta$')
axs[1].set_ylabel('Average Cost')
axs1_line = axs[1].twinx()  # Create a secondary y-axis
axs1_line.plot(eta, avg_cost, 'm-')
axs1_line.set_ylabel('Average Cost (Line Plot)', color='m')
axs1_line.tick_params(axis='y', labelcolor='m')

plt.tight_layout()

plt.show()
