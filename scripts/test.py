#%%
import json
import numpy as np
from dataclasses import dataclass
import math as m
import codecs


@dataclass
class Mesh:
    vertices: list
    edges: list
    elems: list


@dataclass
class Vertex:
    value: np.ndarray
    code: int
    adjacent: list


@dataclass
class Plane:
    c: np.ndarray
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray

    def project(self, p):
        c, x, y, z = self.c, self.x, self.y, self.z
        w = p - c
        w = w - z * (w.T @ z)
        u = w.T @ x
        v = w.T @ y
        return c + u * x + v * y


def cross(x, y):
    return np.cross(x.flatten(), y.flatten())


def read_mesh(d):
    vertices = {
        i["id"]: Vertex(
            value=np.array(i["value"]).reshape(3, 1),
            code=i["code"],
            adjacent=i["adjacent"],
        )
        for i in d["vertices"]
    }
    edges = d["edges"]
    elements = d["elements"]
    return Mesh(vertices, edges, elements)


def plane_from_abc(a, b, c):
    x, y = b - a, c - a
    y = y - x * (y.T @ x)
    z = cross(x, y)
    x /= np.linalg.norm(x)
    y /= np.linalg.norm(y)
    z /= np.linalg.norm(z)
    return Plane(c=a, x=x, y=y, z=z)


def angle_abc(a, b, c):
    x, y = a - b, c - b
    p = x.T @ y
    q = np.linalg.norm(cross(x, y))
    return m.atan2(q, p)


def node_regularity(vtxs):
    result = 0
    for i in vtxs.values():
        result += abs((1 if i.code == 2 else 0) + len(i.adjacent) - 4)
    return result


def angle_regularity(a, b, c, d):
    p = m.pi / 2
    return (
        abs(angle_abc(a, b, c) - p)
        + abs(angle_abc(b, c, d) - p)
        + abs(angle_abc(c, d, a) - p)
        + abs(angle_abc(d, a, b) - p)
    )


def mesh_angle_regularity(mesh):
    v = mesh.vertices
    result = 0
    for i in mesh.elems:
        a, b, c = v[i[0]].value, v[i[1]].value, v[i[2]].value
        p = plane_from_abc(a, b, c)
        d = p.project(v[i[3]].value)
        result += angle_regularity(a, b, c, d)
    return result / 4


def elem_min_angle(v, elems):
    result = []
    for i in elems:
        a, b, c, d = v[i[0]].value, v[i[1]].value, v[i[2]].value, v[i[3]].value
        # p = plane_from_abc(a, b, c)
        # d = p.project(v[i[3]].value)
        result.append(
            min(
                angle_abc(a, b, c),
                angle_abc(b, c, d),
                angle_abc(c, d, a),
                angle_abc(d, a, b),
            )
        )
    return np.array(result)


def elem_angle(v, elems):
    result = []
    for i in elems:
        a, b, c, d = v[i[0]].value, v[i[1]].value, v[i[2]].value, v[i[3]].value
        result.append(
            [
                angle_abc(a, b, c),
                angle_abc(b, c, d),
                angle_abc(c, d, a),
                angle_abc(d, a, b),
            ]
        )
    return np.array(result).flatten()


def mesh_stats(mesh):
    return (
        len(mesh.vertices),
        len(mesh.elems),
        node_regularity(mesh.vertices),
        mesh_angle_regularity(mesh),
    )


def tojson(filename, vtxs, elems):
    def vtx_to_dict(i, v):
        v = v.__dict__
        v["value"] = list(v["value"].flatten())
        v["id"] = i
        return v

    vtxs = [vtx_to_dict(i, j) for i, j in enumerate(vtxs)]

    with codecs.open(filename, "w", "ascii") as fou:
        json.dump({"vertices": vtxs, "elements": elems}, fou)


def togeo(filename, vtxs, elems):
    with codecs.open(filename, "w", "ascii") as fou:
        fou.write(
            "$MeshFormat\n4.1 0 {0}\n$EndMeshFormat\n$Nodes\n1 {1} {2} {3}\n2 1 0 {1}\n".format(
                8, len(vtxs), 1, len(vtxs),
            )
        )
        for i in range(len(vtxs)):
            fou.write("{}\n".format(i + 1))
        for i in vtxs:
            fou.write("{:.5f} {:.5f} {:.5f}\n".format(*list(i.value.flatten())))
        fou.write("$EndNodes\n$Elements\n1 {0} 1 {0}\n2 1 3 {0}\n".format(len(elems)))
        for i, j in enumerate(elems):
            fou.write("{} {} {} {} {}\n".format(i + 1, *[k + 1 for k in j]))
        fou.write("$EndElements\n")


#%%
with open("../qcircle.1.json", "r") as fp:
    mesh = read_mesh(json.load(fp)[0])

#%%
print(mesh_stats(mesh))

# %%
# 0
# ranges = [(0, 33), (35, 38), (40, 42)]
# ranges = [(0, 1), (9, 18), (33, 35), (38, 40), (42, 54), (55, 56), (57, 62)]
# ranges = [(0, 10), (33, 35), (38, 40), (42, 46), (54, 55), (56, 57), (62, 76)]
# 1
# ranges = [(0, 33), (34, 38), (39, 42)]
# ranges = [(0, 1), (9, 18), (33, 34), (38, 39), (42, 54), (55, 56), (57, 58), (59, 65)]
ranges = [(0, 10), (33,34),(38,39),(42,48),(54,55),(56,57),(58,59),(65,80)]

def in_ranges(i):
    return any(x[0] <= i < x[1] for x in ranges)


vtxs, elems, vmap = [], [], {}
for k, v in mesh.vertices.items():
    if in_ranges(k):
        vtxs.append(v)
        vmap[k] = len(vtxs) - 1

vi = set(vmap.keys())
for elem in mesh.elems:
    if all(j in vi for j in elem):
        elems.append([vmap[j] for j in elem])

togeo("qcircle_12.geo", vtxs, elems)

# %%
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

plt.style.use("seaborn-notebook")

angles = 180 * elem_min_angle(vtxs, elems) / m.pi
fig, ax = plt.subplots()
n, _, _ = ax.hist(angles, lw=1, color="#BBBBBB", ec="#333333")
fig.savefig("qcircle_12.pdf")
fig.show()
# %%
min(angles), max(angles)

# %%
elem_angle(vtxs, elems)

# %%
n

# %%
