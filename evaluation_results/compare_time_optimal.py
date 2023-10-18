import matplotlib.pyplot as plt
import yaml
import numpy as np

# Loading data
with open("spherical_to.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        items = list(data.items())
        current_to_trial_index = 0
        for i in items:
            if i[1]['Success'] == True:
                if i[1]['policy'] == 'time_opt':
                    current_to_trial_index += 1
        time_to = np.zeros(current_to_trial_index)

        current_to_trial_index = 0
        for i in items:
            if i[1]['Success'] == True:
                if i[1]['policy'] == 'time_opt':
                    time_to[current_to_trial_index] = i[1]['time_to_finish']
                    current_to_trial_index += 1
    except yaml.YAMLError as exc:
        print(exc)

with open("spherical_db.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        items = list(data.items())
        current_db_trial_index = 0
        for i in items:
            if i[1]['Success'] == True:
                if i[1]['policy'] == 'depth_based':
                    current_db_trial_index += 1
        time_db = np.zeros(current_db_trial_index)

        current_db_trial_index = 0
        for i in items:
            if i[1]['Success'] == True:
                if i[1]['policy'] == 'depth_based':
                    time_db[current_db_trial_index] = i[1]['time_to_finish']
                    current_db_trial_index += 1
    except yaml.YAMLError as exc:
        print(exc)


# Plot
time_means = (time_db.mean(),
                    time_to.mean())
time_std = (time_db.std(),
                time_to.std())

ind = np.arange(len(time_means))  # the x locations for the groups
width = 0.36  # the width of the bars

fig, ax = plt.subplots()
rects1 = ax.bar(ind - width/2, time_means, width, yerr=time_std,
                label='Spherical obstacles')
rects3 = ax.bar(ind + width/2, time_means, width, yerr=time_std,
                label='Forest')
# Add some text for labels, title and custom x-axis tick labels, etc.
plt.rcParams['font.sans-serif'] = "Times New Roman"
plt.rcParams['font.family'] = "sans-serif"
plt.rcParams['font.size'] = 33
# set default font size for tick labels
# plt.rcParams['xtick.labelsize'] = 33
# plt.rcParams['ytick.labelsize'] = 33

ax.set_ylabel('Time [second]', fontsize=33)
ax.set_title('Time to finish by policy', fontsize=33)
ax.set_xticks(ind)
ax.set_xticklabels(('Minimum-jerk (Nguyen et al)', 'State-to-state Time-optimal (proposed)'), fontsize=33)
ax.legend()


def autolabel(rects):
    """
    Attach a text label above each bar in *rects*, displaying its height.

    *xpos* indicates which side to place the text w.r.t. the center of
    the bar. It can be one of the following {'center', 'right', 'left'}.
    """

    for rect in rects:
        height = rect.get_height()
        ax.annotate('{:.2f}'.format(height),
                    xy=(rect.get_x() + rect.get_width() / 2, 5),
                    textcoords="offset points",  # in both directions
                    ha='center', va='bottom')

autolabel(rects1)
autolabel(rects3)
# plt.xticks(np.arange(0, 450, step=50), fontsize=33)
# plt.yticks(np.arange(-1.2, 0.1, step=0.2), fontsize=33)
plt.xticks(fontsize=33)
plt.yticks(fontsize=33)
plt.ylim([5, 30])
fig.tight_layout()
plt.grid()
plt.show()