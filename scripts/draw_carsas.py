#%%
import json
import numpy as np
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

#%%
import mpld3

mpld3.enable_notebook()
#%%
def load_edges(filename):
    with open(filename, "r") as f:
        result = [np.round(i, 3) for i in json.load(f)]
    return result


def plot_edge(ax, edge, colors=["C0", "C1"]):
    x, y, z = edge.transpose()
    ax.plot(x, y, z, color=colors[0])
    ax.plot(x, y, z, ".", color=colors[1])


#%%
_filename = "build/bin/out/cyl_edge.json"

data = load_edges(_filename)
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

for i in data:
    plot_edge(ax, i)

#%%
