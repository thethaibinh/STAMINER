import matplotlib.pyplot as plt
import yaml
import numpy as np

with open("results_easy.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        items = list(data.items())
        number_easy_items_evaluated = 0
        number_easy_items_ny = 0
        number_easy_items_st = 0
        number_easy_items_da = 0
        number_easy_items = 0
        for i in items:
            number_easy_items += 1
            if i[1]['Success'] == True:
                number_easy_items_evaluated += 1
                if i[1]['policy'] == 'naive_yawing':
                    number_easy_items_ny += 1
                elif i[1]['policy'] == 'steering':
                    number_easy_items_st += 1
                elif i[1]['policy'] == 'depth_aware':
                    number_easy_items_da += 1
        # distance_easy_ny = np.zeros(number_easy_items_ny)
        # distance_easy_st = np.zeros(number_easy_items_st)
        # distance_easy_da = np.zeros(number_easy_items_da)
        time_easy_ny = np.zeros(number_easy_items_ny)
        time_easy_st = np.zeros(number_easy_items_st)
        time_easy_da = np.zeros(number_easy_items_da)
        
        number_easy_items_ny = 0
        number_easy_items_st = 0
        number_easy_items_da = 0
        for i in items:
            if i[1]['Success'] == True:
                if i[1]['policy'] == 'naive_yawing':
                    time_easy_ny[number_easy_items_ny] = i[1]['time_to_finish']
                    # distance_easy_ny[number_easy_items_ny] = i[1]['travelled_distance']
                    number_easy_items_ny += 1
                elif i[1]['policy'] == 'steering':
                    time_easy_st[number_easy_items_st] = i[1]['time_to_finish']
                    # distance_easy_st[number_easy_items_st] = i[1]['travelled_distance']
                    number_easy_items_st += 1
                elif i[1]['policy'] == 'depth_aware':
                    time_easy_da[number_easy_items_da] = i[1]['time_to_finish']
                    # distance_easy_da[number_easy_items_da] = i[1]['travelled_distance']
                    number_easy_items_da += 1
    except yaml.YAMLError as exc:
        print(exc)

with open("results_medium.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        items = list(data.items())
        number_medium_items_evaluated = 0
        number_medium_items_ny = 0
        number_medium_items_st = 0
        number_medium_items_da = 0
        number_medium_items = 0
        for i in items:
            number_medium_items += 1
            if i[1]['Success'] == True:
                number_medium_items_evaluated += 1
                if i[1]['policy'] == 'naive_yawing':
                    number_medium_items_ny += 1
                elif i[1]['policy'] == 'steering':
                    number_medium_items_st += 1
                elif i[1]['policy'] == 'depth_aware':
                    number_medium_items_da += 1
        # distance_medium_ny = np.zeros(number_medium_items_ny)
        # distance_medium_st = np.zeros(number_medium_items_st)
        # distance_medium_da = np.zeros(number_medium_items_da)
        time_medium_ny = np.zeros(number_medium_items_ny)
        time_medium_st = np.zeros(number_medium_items_st)
        time_medium_da = np.zeros(number_medium_items_da)
        
        number_medium_items_ny = 0
        number_medium_items_st = 0
        number_medium_items_da = 0
        for i in items:
            if i[1]['Success'] == True:
                if i[1]['policy'] == 'naive_yawing':
                    time_medium_ny[number_medium_items_ny] = i[1]['time_to_finish']
                    # distance_medium_ny[number_medium_items_ny] = i[1]['travelled_distance']
                    number_medium_items_ny += 1
                elif i[1]['policy'] == 'steering':
                    time_medium_st[number_medium_items_st] = i[1]['time_to_finish']
                    # distance_medium_st[number_medium_items_st] = i[1]['travelled_distance']
                    number_medium_items_st += 1
                elif i[1]['policy'] == 'depth_aware':
                    time_medium_da[number_medium_items_da] = i[1]['time_to_finish']
                    # distance_medium_da[number_medium_items_da] = i[1]['travelled_distance']
                    number_medium_items_da += 1
    except yaml.YAMLError as exc:
        print(exc)

with open("results_hard.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        items = list(data.items())
        number_hard_items_evaluated = 0
        number_hard_items_ny = 0
        number_hard_items_st = 0
        number_hard_items_da = 0
        number_hard_items = 0
        for i in items:
            number_hard_items += 1
            if i[1]['Success'] == True:
                number_hard_items_evaluated += 1
                if i[1]['policy'] == 'naive_yawing':
                    number_hard_items_ny += 1
                elif i[1]['policy'] == 'steering':
                    number_hard_items_st += 1
                elif i[1]['policy'] == 'depth_aware':
                    number_hard_items_da += 1
        # distance_hard_ny = np.zeros(number_hard_items_ny)
        # distance_hard_st = np.zeros(number_hard_items_st)
        # distance_hard_da = np.zeros(number_hard_items_da)
        time_hard_ny = np.zeros(number_hard_items_ny)
        time_hard_st = np.zeros(number_hard_items_st)
        time_hard_da = np.zeros(number_hard_items_da)
        
        number_hard_items_ny = 0
        number_hard_items_st = 0
        number_hard_items_da = 0
        for i in items:
            if i[1]['Success'] == True:
                if i[1]['policy'] == 'naive_yawing':
                    time_hard_ny[number_hard_items_ny] = i[1]['time_to_finish']
                    # distance_hard_ny[number_hard_items_ny] = i[1]['travelled_distance']
                    number_hard_items_ny += 1
                elif i[1]['policy'] == 'steering':
                    time_hard_st[number_hard_items_st] = i[1]['time_to_finish']
                    # distance_hard_st[number_hard_items_st] = i[1]['travelled_distance']
                    number_hard_items_st += 1
                elif i[1]['policy'] == 'depth_aware':
                    time_hard_da[number_hard_items_da] = i[1]['time_to_finish']
                    # distance_hard_da[number_hard_items_da] = i[1]['travelled_distance']
                    number_hard_items_da += 1
    except yaml.YAMLError as exc:
        print(exc)

easy_time_means = (time_easy_ny.mean(), 
                    # time_easy_st.mean(), 
                    time_easy_da.mean())
easy_time_std = (time_easy_ny.std(), 
                # time_easy_st.std(),
                time_easy_da.std())

medium_time_means = (time_medium_ny.mean(), 
                    # time_medium_st.mean(), 
                    time_medium_da.mean())
medium_time_std = (time_medium_ny.std(), 
                # time_medium_st.std(),
                time_medium_da.std())

hard_time_means = (time_hard_ny.mean(), 
                    # time_hard_st.mean(), 
                    time_hard_da.mean())
hard_time_std = (time_hard_ny.std(), 
                # time_hard_st.std(),
                time_hard_da.std())

ind = np.arange(len(easy_time_means))  # the x locations for the groups
width = 0.24  # the width of the bars

fig, ax = plt.subplots()
rects1 = ax.bar(ind - width, easy_time_means, width, yerr=easy_time_std,
                label='Easy')
rects2 = ax.bar(ind, medium_time_means, width, yerr=medium_time_std,
                label='Medium')
rects3 = ax.bar(ind + width, hard_time_means, width, yerr=hard_time_std,
                label='Hard')
# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Time [second]')
ax.set_title('Time to finish by policy')
ax.set_xticks(ind)
ax.set_xticklabels(('Naive Yawing (Lee et al)', 'Steering+DAPPer (our)'))
ax.legend()

def autolabel(rects, xpos='center'):
    """
    Attach a text label above each bar in *rects*, displaying its height.

    *xpos* indicates which side to place the text w.r.t. the center of
    the bar. It can be one of the following {'center', 'right', 'left'}.
    """

    ha = {'center': 'center', 'right': 'right', 'left': 'left'}
    offset = {'center': 0, 'right': 1, 'left': -1}

    for rect in rects:
        height = rect.get_height()
        ax.annotate('{:.1f}'.format(height),
                    xy=(rect.get_x() + rect.get_width() / 2, 0),
                    xytext=(offset[xpos]*10, 0),  # use 3 points offset
                    textcoords="offset points",  # in both directions
                    ha=ha[xpos], va='bottom')

autolabel(rects1, "left")
# autolabel(rects2, "center")
autolabel(rects3, "center")

plt.ylim([16, 32])
fig.tight_layout()
plt.grid()
plt.show()