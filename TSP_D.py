from gurobipy import *
import networkx as nx
import random
import matplotlib.pyplot as plt
import numpy as np
import Class
import CostFunc as f

_biasForDist = 1.2
_dSpeed = 70 / 60  # 時速70→分速km
_tSpeed = 45 / 60  # 時速45→分速km
C1 = 25
C2 = 1
dD = {}  # distance by drone
dT = {}  # distance by truck
tD = {}  # time by drone
tT = {}  # time by truck


def solve_tsp_d(S, V):
    model = Model("tsp-d")
    x = {}
    y = {}
    for i in V:
        for j in V:
            x[i.pos, j.pos] = model.addVar(vtype="B")
            for k in V:
                y[i.pos, j.pos, k.pos] = model.addVar(vtype="B")


def make_data(n):
    """make_data: compute matrix distance based on euclidean distance"""
    S = Class.Sequence(n, None, None)
    V = [n]
    for i in range(1, n + 1):
        V[i] = Class.Node(i)
        V[i].coordinate = (10 * random.random(), 10 * random.random())
        V[i].prev = i - 1
        V[i].next = i + 1
        if i == 1:
            V[i].prev = n
        elif i == n:
            V[i].next = 1
        S.nodes[i] = V[i]
    S.first = V[1]
    S.last = V[n]
    global dD, dT, tD, tT
    for i in V:
        for j in V:
            dD[i, j] = distance(i.coordinate, j.coordinate)
            dT[i, j] = dD[i, j] * _biasForDist
            tD[i, j] = dD[i, j] / _dSpeed
            tT[i, j] = dT[i, j] / _tSpeed
    return S, V


def distance(xy1, xy2):
    """distance: euclidean distance between (x1,y1) and (x2,y2)"""
    return math.sqrt((xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[2]) ** 2)


def main():
    n = 10  # 客の総数
    seed = 1
    random.seed(seed)
    S, V = make_data(n)


if __name__ == "__main__":
    main()
