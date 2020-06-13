#!/usr/bin/env python3
import codecs
import argparse
import json


def togeo(file):
    out = ".".join([file.rsplit(".", 1)[0], "geo"])
    with codecs.open(file, "r", "ascii") as fin:
        d = json.load(fin)[0]

    with codecs.open(out, "w", "ascii") as fou:
        fou.write(
            "$MeshFormat\n4.1 0 {0}\n$EndMeshFormat\n$Nodes\n 1 {1} {2} {3}\n2 1 0 {1}\n".format(
                8,
                len(d["vertices"]),
                d["vertices"][0]["id"] + 1,
                d["vertices"][-1]["id"] + 1,
            )
        )
        for i in d["vertices"]:
            fou.write("{}\n".format(i["id"] + 1))
        for i in d["vertices"]:
            fou.write("{:.5f} {:.5f} {:.5f}\n".format(*i["value"]))
        fou.write(
            "$EndNodes\n$Elements\n1 {0} 1 {0}\n2 1 3 {0}\n".format(len(d["elements"]))
        )
        for i, j in enumerate(d["elements"]):
            fou.write("{} {} {} {} {}\n".format(i + 1, *[k + 1 for k in j]))
        fou.write("$EndElements\n")


def main():
    p = argparse.ArgumentParser(prog="togeo")
    p.add_argument("file", nargs="+")
    a = p.parse_args()
    for i in a.file:
        togeo(i)


if __name__ == "__main__":
    main()
