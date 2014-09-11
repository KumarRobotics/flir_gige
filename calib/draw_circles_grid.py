__author__ = 'chao'

import matplotlib.pyplot as plt

fig = plt.gcf()
n_cols = 5
n_rows = 4
r = 0.1
for x in range(0, n_cols):
    for y in range(0, n_rows):
        circle = plt.Circle((x, y), r, color='k')
        fig.gca().add_artist(circle)
fig.gca().set_aspect('equal')
fig.gca().axis([-1.5, n_cols + 0.5, -1.5, n_rows + 0.5])
plt.show()

fig.savefig('circles_grid.png')
