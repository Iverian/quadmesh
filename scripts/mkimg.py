#!/usr/bin/env python3
import os
import json
import matplotlib.pyplot as plt
import argparse as ap

from mpl_toolkits.mplot3d import Axes3D


class Vertex(object):
    def __init__(self, point):
        self.p = point

    def __repr__(self):
        return self.p


def draw_vertices(ax, vtxs):
    for k, v in vtxs.items():
        ax.plot([v.p[0]], [v.p[1]], [v.p[2]], ".", color=f"C{k % 10}")


def draw_edges(ax, vtxs, edges):
    for i, j in edges:
        a = vtxs[i].p
        b = vtxs[j].p
        ax.plot([a[0], b[0]], [a[1], b[1]], [a[2], b[2]], "-", color="#555555")


def carcas(ax, data):
    vtxs = {i["id"]: Vertex(i["value"]) for i in data["vertices"]}
    edges = data["edges"]

    draw_edges(ax, vtxs, edges)
    draw_vertices(ax, vtxs)


#%%
if __name__ == "__main__":
    p = ap.ArgumentParser("mkimg")
    p.add_argument("dir")
    p.add_argument("-o", "--outdir", type=str, default="out")
    p.add_argument("-l", "--lower", type=int, default=0)
    p.add_argument("-u", "--upper", type=int, default=1000)
    p.add_argument("-k", "--keep", action="store_true")

    args = p.parse_args()
    path = os.path.abspath(args.dir)
    if os.path.isabs(args.outdir):
        outdir = args.outdir
    else:
        outdir = os.path.join(path, args.outdir)
    if not os.path.isdir(outdir):
        os.makedirs(outdir)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    count = abs(args.upper - args.lower)
    for f in os.listdir(path):
        if f.endswith(".json"):
            i = int(f.split("-", 1)[0])
            if args.lower <= i and i < args.upper:
                ax.clear()
                with open(os.path.join(path, f), "r") as ff:
                    carcas(ax, json.load(ff)[0])
                name = f.rsplit(".", 1)[0]
                fig.savefig(os.path.join(outdir, f"{name}.png"), dpi=200)

                if not args.keep:
                    os.remove(os.path.join(path, f))
                count -= 1
        if count <= 0:
            break
