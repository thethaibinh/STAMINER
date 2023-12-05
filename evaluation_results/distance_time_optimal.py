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
                    time_to[current_to_trial_index] = i[1]['travelled_distance']
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
                    time_db[current_db_trial_index] = i[1]['travelled_distance']
                    current_db_trial_index += 1
    except yaml.YAMLError as exc:
        print(exc)


# Plot
time_means = (time_db.mean(),
              time_to.mean())
time_std = (time_db.std(),
            time_to.std())
print(time_std)

ind = np.arange(len(time_means))  # the x locations for the groups
width = 0.36  # the width of the bars

fig, ax = plt.subplots()
rects1 = ax.bar(ind - width/2, time_means, width, yerr=time_std, error_kw=dict(lw=5, capsize=5, capthick=3),
                label='Spherical obstacles')
rects3 = ax.bar(ind + width/2, time_means, width, yerr=time_std, error_kw=dict(lw=5, capsize=5, capthick=3),
                label='Forest')
# Add some text for labels, title and custom x-axis tick labels, etc.
plt.rcParams['font.sans-serif'] = "Times New Roman"
plt.rcParams['font.family'] = "sans-serif"
plt.rcParams['font.size'] = 43

ax.set_ylabel('Distance [m]', fontsize=43)
ax.set_title('Distance travelled by policy', fontsize=43)
ax.set_xticks(ind)
ax.set_xticklabels(('DESS (Nguyen et al)', 'Our planner (proposed)'), fontsize=43)
ax.legend(
        # bbox_to_anchor=(1.0, 0.63, 0.0, 0.0),
            loc="upper left",
            ncol=1,
            markerscale=1,
            fontsize=43,
            handlelength=3)
# plt.legend(bbox_to_anchor=(1.0, 0.63, 0.0, 0.0),
#             loc="center right",
#             ncol=1,
#             markerscale=1,
#             fontsize=33,
#             handlelength=3)

def autolabel(rects):
    """
    Attach a text label above each bar in *rects*, displaying its height.

    *xpos* indicates which side to place the text w.r.t. the center of
    the bar. It can be one of the following {'center', 'right', 'left'}.
    """

    for rect in rects:
        height = rect.get_height()
        ax.annotate('{:.2f}'.format(height),
                    xy=(rect.get_x() + rect.get_width() / 2, 19.6),
                    textcoords="offset points",  # in both directions
                    ha='center', va='top')

autolabel(rects1)
autolabel(rects3)
plt.xticks(fontsize=43)
plt.yticks(fontsize=43)

plt.ylim([18.7, 37])
fig.tight_layout()
plt.grid()
plt.show()