from matplotlib import pyplot as plt
import textwrap as twp
import yaml
import numpy as np

with open("spherical_to.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        items = list(data.items())
        number_to_items_failed = 0
        number_to_items_success = 0
        for i in items:
            if i[1]['Success'] == True:
                if i[1]['policy'] == 'time_opt':
                    number_to_items_success += 1
            else:
                if i[1]['policy'] == 'time_opt':
                    number_to_items_failed += 1
        bubble_to_success_rate = number_to_items_success / \
            (number_to_items_success + number_to_items_failed)
    except yaml.YAMLError as exc:
        print(exc)

with open("spherical_db.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        items = list(data.items())
        number_db_items_failed = 0
        number_db_items_success = 0
        for i in items:
            if i[1]['Success'] == True:
                if i[1]['policy'] == 'depth_based':
                    number_db_items_success += 1
            else:
                if i[1]['policy'] == 'depth_based':
                    number_db_items_failed += 1
        bubble_db_success_rate = number_db_items_success / (number_db_items_success + number_db_items_failed)
    except yaml.YAMLError as exc:
        print(exc)

# float_formatter = "{:.4f}".format
fig = plt.figure()
cells = np.round([[bubble_to_success_rate, bubble_db_success_rate],
        [bubble_to_success_rate, bubble_db_success_rate]],4)
# print(cells)
print(bubble_to_success_rate)
print(bubble_db_success_rate)
img = plt.imshow(cells, cmap="RdYlGn")
img.set_visible(False)
plt.axis('off')
ax = fig.add_subplot(111, frameon=False, xticks = [], yticks = [])

tb = plt.table(cellText = cells,
    cellLoc='center',
    rowLabels = ['Spherical', 'Forest'],
    colLabels = [twp.fill('Minimum-jerk (Nguyen et al)', 16),
                twp.fill('State-to-state Time-optimal (proposed)', 16)],
    loc = 'center',
    cellColours = img.to_rgba(cells))
for key, cell in tb.get_celld().items():
    cell.set_edgecolor('w')

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