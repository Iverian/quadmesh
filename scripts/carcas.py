#!/usr/bin/env python3
import os
import sys
import time
import json
import numpy as np
import matplotlib.pyplot as plt
import argparse as ap

from mpl_toolkits.mplot3d import Axes3D


class Vertex(object):
    def __init__(self, point, adjacent):
        self.p = point
        self.adj = adjacent

    def __repr__(self):
        return f'{{"p": {self.p}, "adj": {self.adj}}}'


def draw_vertices(ax, vtxs):
    for k, v in vtxs.items():
        ax.plot([v.p[0]], [v.p[1]], [v.p[2]], ".", color=f"C{k % 10}")


def draw_edges(ax, vtxs, edges):
    for i, j in edges:
        a = vtxs[i].p
        b = vtxs[j].p
        ax.plot([a[0], b[0]], [a[1], b[1]], [a[2], b[2]], "-", color="#555555")


def carcas(ax, data):
    vtxs = {
        i["id"]: Vertex(i["value"], i["adjacent"])
        for i in data["vertices"]
    }
    edges = data["edges"]

    draw_edges(ax, vtxs, edges)
    draw_vertices(ax, vtxs)


# %%
if __name__ == "__main__":
    p = ap.ArgumentParser("carcas")
    p.add_argument("file")
    args = p.parse_args()
    path = os.path.abspath(args.file)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    with open(path, "r") as f:
        carcas(ax, json.load(f)[0])
    plt.show()
