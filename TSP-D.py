from gurobipy import *
import networkx as nx
import random
import matplotlib.pyplot as plt
import numpy as np

biasForDist = 1.2
dSpeed = 70 / 60  # 時速70→分速km
tSpeed = 45 / 60  # 時速45→分速km


def make_data(n):
    """make_data: compute matrix distance based on euclidean distance"""
    V = range(1, n + 1)
    xy = dict([(i, (10 * random.random(), 10 * random.random())) for i in V])
    dd = {}  # トラックでのdist
    dt = {}  # ドローンでのdist
    for i in V:
        for j in V:
            if j > i:
                dd[i, j] = distance(xy[i][0], xy[i][1], xy[j][0], xy[j][1])
                dt[i, j] = biasForDist * dd[i, j]
    return V, dd, dt, xy


def distance(x1, y1, x2, y2):
    """distance: euclidean distance between (x1,y1) and (x2,y2)"""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def main():
    n = 10  # 客の総数
    seed = 1
    random.seed(seed)
    V, dd, dt, xy = make_data(n)
