from gurobipy import *
import networkx as nx
import random
import matplotlib.pyplot as plt
import numpy as np

_biasForDist = 1.2
_dSpeed = 70 / 60  # 時速70→分速km
_tSpeed = 45 / 60  # 時速45→分速km


class Node:
    def __init__(self, num):
        self.pos = num
        self.coordinate = (0, 0)
        self.prev = None
        self.next = None

    def next(self):
        return self.next

    def prev(self):
        return self.prev


class Sequence:
    def __init__(self, size, first, last):
        self.size = size
        self.first = first
        self.last = last
        self.nodes = [size + 1]

    def pos(self, i):
        return self.nodes[i].pos

    def node(self, i):
        return self.nodes[i]

    def subSequence(self, i, j):
        sub = Sequence(j - i + 1, i, j)
        for k in range(i, j + 1):
            sub.nodes[k - i + 1] = self.nodes[k]
        return sub

    def edge(self):
        edges = set()
        i = 1
        while i != self.size + 1:
            edges.add((self.nodes[i].pos, self.nodes[i].next.pos))
            i += 1
        return edges


def make_data(n):
    """make_data: compute matrix distance based on euclidean distance"""
    V = [n]
    for i in range(1, n + 1):
        V[i] = Node(i)
        V[i].pos = (10 * random.random(), 10 * random.random())
        V[i].prev = i - 1
        V[i].next = i + 1
        if i == 1:
            V[i].prev = n
        elif i == n:
            V[i].next = 1
    dd = {}
    dt = {}
    for i in V:
        for j in V:
            if j > i:
                dd[i, j] = distance(xy[i][0], xy[i][1], xy[j][0], xy[j][1])
                dt[i, j] = _biasForDist * dd[i, j]
    return V, dd, dt, xy


def distance(x1, y1, x2, y2):
    """distance: euclidean distance between (x1,y1) and (x2,y2)"""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def main():
    n = 10  # 客の総数
    seed = 1
    random.seed(seed)
    V, dd, dt, xy = make_data(n)
