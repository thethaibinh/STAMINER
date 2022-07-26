from matplotlib import pyplot as plt
import textwrap as twp
import yaml
import numpy as np

with open("results_easy.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        items = list(data.items())
        number_easy_items_ny_failed = 0
        number_easy_items_st_failed = 0
        number_easy_items_da_failed = 0
        number_easy_items_ny_success = 0
        number_easy_items_st_success = 0
        number_easy_items_da_success = 0
        for i in items:
            if i[1]['Success'] == True:
                if i[1]['policy'] == 'naive_yawing':
                    number_easy_items_ny_success += 1
                elif i[1]['policy'] == 'steering':
                    number_easy_items_st_success += 1
                elif i[1]['policy'] == 'depth_aware':
                    number_easy_items_da_success += 1
            else:
                if i[1]['policy'] == 'naive_yawing':
                    number_easy_items_ny_failed += 1
                elif i[1]['policy'] == 'steering':
                    number_easy_items_st_failed += 1
                elif i[1]['policy'] == 'depth_aware':
                    number_easy_items_da_failed += 1
        easy_ny_success_rate = number_easy_items_ny_success / (number_easy_items_ny_success + number_easy_items_ny_failed)
        easy_st_success_rate = number_easy_items_st_success / (number_easy_items_st_success + number_easy_items_st_failed)
        easy_da_success_rate = number_easy_items_da_success / (number_easy_items_da_success + number_easy_items_da_failed)
    except yaml.YAMLError as exc:
        print(exc)

with open("results_medium.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        items = list(data.items())
        number_medium_items_ny_failed = 0
        number_medium_items_st_failed = 0
        number_medium_items_da_failed = 0
        number_medium_items_ny_success = 0
        number_medium_items_st_success = 0
        number_medium_items_da_success = 0
        for i in items:
            if i[1]['Success'] == True:
                if i[1]['policy'] == 'naive_yawing':
                    number_medium_items_ny_success += 1
                elif i[1]['policy'] == 'steering':
                    number_medium_items_st_success += 1
                elif i[1]['policy'] == 'depth_aware':
                    number_medium_items_da_success += 1
            else:
                if i[1]['policy'] == 'naive_yawing':
                    number_medium_items_ny_failed += 1
                elif i[1]['policy'] == 'steering':
                    number_medium_items_st_failed += 1
                elif i[1]['policy'] == 'depth_aware':
                    number_medium_items_da_failed += 1
        medium_ny_success_rate = number_medium_items_ny_success / (number_medium_items_ny_success + number_medium_items_ny_failed)
        medium_st_success_rate = number_medium_items_st_success / (number_medium_items_st_success + number_medium_items_st_failed)
        medium_da_success_rate = number_medium_items_da_success / (number_medium_items_da_success + number_medium_items_da_failed)
    except yaml.YAMLError as exc:
        print(exc)

with open("results_hard.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        items = list(data.items())
        number_hard_items_ny_failed = 0
        number_hard_items_st_failed = 0
        number_hard_items_da_failed = 0
        number_hard_items_ny_success = 0
        number_hard_items_st_success = 0
        number_hard_items_da_success = 0
        for i in items:
            if i[1]['Success'] == True:
                if i[1]['policy'] == 'naive_yawing':
                    number_hard_items_ny_success += 1
                elif i[1]['policy'] == 'steering':
                    number_hard_items_st_success += 1
                elif i[1]['policy'] == 'depth_aware':
                    number_hard_items_da_success += 1
            else:
                if i[1]['policy'] == 'naive_yawing':
                    number_hard_items_ny_failed += 1
                elif i[1]['policy'] == 'steering':
                    number_hard_items_st_failed += 1
                elif i[1]['policy'] == 'depth_aware':
                    number_hard_items_da_failed += 1
        hard_ny_success_rate = number_hard_items_ny_success / (number_hard_items_ny_success + number_hard_items_ny_failed)
        hard_st_success_rate = number_hard_items_st_success / (number_hard_items_st_success + number_hard_items_st_failed)
        hard_da_success_rate = number_hard_items_da_success / (number_hard_items_da_success + number_hard_items_da_failed)
    except yaml.YAMLError as exc:
        print(exc)

# float_formatter = "{:.4f}".format
fig = plt.figure()
cells = np.round([[easy_ny_success_rate, easy_st_success_rate, easy_da_success_rate], 
        [medium_ny_success_rate, medium_st_success_rate, medium_da_success_rate], 
        [hard_ny_success_rate, hard_st_success_rate, hard_da_success_rate]],6)
img = plt.imshow(cells, cmap="RdYlGn")
img.set_visible(False)
plt.axis('off')
ax = fig.add_subplot(111, frameon=False, xticks = [], yticks = [])

tb = plt.table(cellText = cells, 
    cellLoc='center',
    rowLabels = ['Easy', 'Medium', 'Hard'], 
    colLabels = [twp.fill('Naive Yawing (Lee et al.)', 16), 
                twp.fill('Steering (our)', 16), 
                twp.fill('Steering + DAPPer (our)', 16)],
    loc = 'center',
    cellColours = img.to_rgba(cells))
for key, cell in tb.get_celld().items():
    cell.set_edgecolor('w')
    # if key[1] != -1 and key[0] != 0:
    #     cell.get_text().set_color('w')

tb.scale(1, 6)
tb.auto_set_font_size(False)
tb.set_fontsize(12)
ax.figure

cbar = plt.colorbar(location='top')
cbar.ax.set_xlabel('Success rate', fontsize=12)
cbar.ax.tick_params(labelsize=12)
cbar.outline.set_linewidth(0)
plt.axis('off')
plt.show()