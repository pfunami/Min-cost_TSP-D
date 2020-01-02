from gurobipy import *
import networkx as nx
import random
import matplotlib.pyplot as plt
import numpy as np
import Class
import CostFunc as f
import copy

_biasForDist = 1.2
_dSpeed = 70 / 60  # 時速70→分速km
_tSpeed = 45 / 60  # 時速45→分速km
C1 = 25
C2 = 1
dD = {}  # distance by drone
dT = {}  # distance by truck
tD = {}  # time by drone
tT = {}  # time by truck
_alpha = 10
_beta = 10
random.seed(1)
model = Model("tsp-d")
M = 1  # ドローンの数？
model.setParam("epsilon", 10)


def solve_tsp_d(S, V, N, n):
    Vl = copy.deepcopy(V[:n])
    Vr = copy.deepcopy(V[1:])
    x = {}
    y = {}
    p = {}
    t = {}
    td = {}
    r = {}
    rd = {}
    w = {}
    wd = {}
    u = {}

    for i in Vl:
        for j in Vr:
            if i.num != j.num:
                x[i.num, j.num] = model.addVar(vtype="B")
            for k in V:
                y[i.num, j.num, k.num] = model.addVar(vtype="B")
        t[i.num] = model.addVar(vtype="C")
        td[i.num] = model.addVar(vtype="C")
        r[i.num] = model.addVar(vtype="C")
        rd[i.num] = model.addVar(vtype="C")
        w[i.num] = model.addVar(vtype="C")
        wd[i.num] = model.addVar(vtype="C")
    for i in V:
        u[i.num] = model.addVar(vtype="I", ub=n + 1)
    for i in N:
        for j in N:
            p[i, j] = model.addVar(vtype="B")
    model.update()

    model.setObjective(
        C1 * quicksum(dT[i.num, j.num] * x[i.num, j.num] for i in Vl for j in Vr if i.num != j.num) +
        C2 * quicksum((dD[i.num, j] + dD[j.num, k.num]) * y[i.num, j, k.num] for i in Vl for j in N for k in Vr) +
        _alpha * quicksum(w[i.num] for i in V) + _beta * quicksum(wd[i.num] for i in V)
        , GRB.MINIMIZE
    )
    for j in N:
        model.addConstr(
            quicksum(x[i.num, j] for i in Vl) +
            quicksum(y[i.num, j, k.num] for i in Vl for k in Vr) == 1
            , name="2"
        )
    model.addConstr(
        quicksum(x[0, j.num] for j in Vr) == 1
        , name="3"
    )
    model.addConstr(
        quicksum(x[i.num, n + 1] for i in Vl) == 1
        , name="4"
    )
    for i, j in Vl, Vr:
        if i.num != j.num:
            model.addConstr(
                u[i.num] - u[j.num] + 1 <= (n + 2) * (1 - x[i.num, j.num])
                , name="5"
            )
    for j in N:
        model.addConstr(
            quicksum(x[i.num, j] for i in Vl if i.num != j) ==
            quicksum(x[j, k.num] for k in Vr if k.num != j)
            , name="6"
        )
    for i, j, k in N, N, Vr:
        if i != j:
            model.addConstr(
                2 * y[i, j, k.num] <=
                quicksum(x[h.num, i] for h in Vl if h.num != i) +
                quicksum(x[l, k] for l in N if l != k)
                , name="7"
            )
    for j, k in N, Vr:
        model.addConstr(
            y[0, j, k.num] <= quicksum(x[h.num, k.num] for h in Vl if h != k & h != j)
            , name="8"
        )
    for i, k in Vl, Vr:
        if i.num != k.num:
            model.addConstr(
                u[k.num] - u[i.num] >= 1 - (n + 2) * (1 - quicksum(y[i.num, j, k.num] for j in N))
                , name="9"
            )
    for i in Vl:
        model.addConstr(
            quicksum(y[i.num, j, k.num] for j in N for k in Vr if j != i.num) <= 1
            , name="10"
        )
    for k in Vr:
        model.addConstr(
            quicksum(y[i.num, j, k.num] for i in Vl for j in N if k.num != i.num) <= 1
            , name="11"
        )
    for i, j in N, Vr:
        if i != j.num:
            model.addConstr(
                u[i] - u[j.num] >= 1 - (n + 2) * p[i, j.num] -
                M * (2 - quicksum(x[h.num, i] for h in Vl if h.num != i) - quicksum(
                    x[k, j.num] for k in N if k != j.num))
                , name="12"
            )
    for i, j in N, Vr:
        if i != j.num:
            model.addConstr(
                u[i] - u[j.num] <=
                -1 + (n + 2) * (1 - p[i, j.num]) +
                M * (2 - quicksum(x[h.num, i] for h in Vl if h.num != i) - quicksum(
                    x[k, j.num] for k in N if k != j.num))
                , name="13"
            )
    for j in Vr:
        model.addConstr(
            u[0] - u[j.num] >= 1 - (n + 2) * p[0, j.num] - M *
            (1 - quicksum(x[k.num, j.num] for k in Vl if k.num != j.num))
            , name="14"
        )
        model.addConstr(
            u[0] - u[j.num] <= -1 + (n + 2) * (1 - p[0, j.num]) +
            M * (1 - quicksum(x[k.num, j.num] for k in Vl if k.num != j.num))
            , name="15"
        )
    for i, k, l in Vl, Vr, N:
        if k.num != i.num & l != i.num & l != k.num:
            model.addConstr(
                u[l] >= u[k.num] - M *
                (3 - quicksum(y[i.num, j, k.num] for j in N if j != i.num) -
                 quicksum(y[l, m, nn.num] - p[i.num, l] for m in N for nn in Vr if
                          m != i.num & m != k.num & m != l & nn.num != i.num & nn.num != k.num))
                , name="16"
            )
    for i, k in Vl, Vr:
        if i.num != k.num:
            model.addConstr(
                t[k.num] >= r[i.num] + tT[i.num, k.num] - M * (1 - x[i.num, k.num])
                , name="17"
            )
            model.addConstr(
                t[k.num] <= r[i.num] + tT[i.num, k.num] + M * (1 - x[i.num, k.num])
                , name="18"
            )
    for j, i in V, Vl:
        if j.num != i.num:
            model.addConstr(
                td[j.num] >= r[i.num] + tD[i.num, j.num] - M * (1 - quicksum(y[i.num, j.num, k.num] for k in Vr))
                , name="19"
            )
            model.addConstr(
                td[j.num] <= r[i.num] + tD[i.num, j.num] + M * (1 - quicksum(y[i.num, j.num, k.num] for k in Vr))
                , name="20"
            )

    def make_data(n):
        """make_data: compute matrix distance based on euclidean distance"""
        S = Class.Sequence(n, None, None)
        V = [n + 2]
        for i in range(0, n + 2):  # インスタンス化
            V[i] = Class.Node(i)
            V[i].coordinate = (10 * random.random(), 10 * random.random())
        for i in range(0, n + 1):  # 0~n n+1はデポ(0)
            if i == 0:
                V[i].next = V[i + 1]
            else:
                V[i].prev = V[i - 1]
                V[i].next = V[i + 1]
            S.nodes[i] = V[i]
        V[n + 1].coordinate = V[0]
        V[n + 1].prev = V[n]
        S.first = V[0]
        S.last = V[n + 1]
        global dD, dT, tD, tT
        for i in V:
            for j in V:
                dD[i.num, j.num] = distance(i.coordinate, j.coordinate)
                dT[i.num, j.num] = dD[i.num, j.num] * _biasForDist
                tD[i.num, j.num] = dD[i.num, j.num] / _dSpeed
                tT[i.num, j.num] = dT[i.num, j.num] / _tSpeed
        return S, V

    def distance(xy1, xy2):
        """distance: euclidean distance between (x1,y1) and (x2,y2)"""
        return math.sqrt((xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[2]) ** 2)

    def main():
        n = 10  # 客の総数
        N = set(range(1, n + 1))
        seed = 1
        random.seed(seed)
        S, V = make_data(n)
        solve_tsp_d(S, V, N, n)

    if __name__ == "__main__":
        main()
