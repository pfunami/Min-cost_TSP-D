from gurobipy import *
import networkx as nx
import random
import matplotlib.pyplot as plt
import numpy as np
import Class
import CostFunc as f
import copy

EPS = 1.e-6
_biasForDist = 1.2
_dSpeed = 40 / 60  # 時速70→分速km
_tSpeed = 40 / 60  # 時速45→分速km
C1 = 25
C2 = 1
Sl = 1
Sr = 1
dD = {}  # distance by drone
dT = {}  # distance by truck
tD = {}  # time by drone
tT = {}  # time by truck
_alpha = 10
_beta = 10
model = Model("tsp-d")
M = GRB.INFINITY  # ドローンの数？
ep = 20.0


def solve_tsp_d(V, N, n):
    def P(i, j, k):
        print(i == j | j == k | i == k)
        if i == j | j == k | i == k:
            return False
        S = set(range(0, n + 2))
        Sd = set(range(1, n + 1))
        if (i in S) & (k in S) & (j in Sd) & (i != j) & (i != k) & (j != k):
            if tD[i, j] + tD[j, k] <= ep:
                return True
        return False

    Vl = copy.deepcopy(V[:n + 1])
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

    for i in V:
        for j in V:
            for k in V:
                if i.num != j.num & i.num != k.num & j.num != k.num:
                    y[i.num, j.num, k.num] = model.addVar(vtype="B")
    for i in Vl:
        for j in Vr:
            if i.num != j.num:
                x[i.num, j.num] = model.addVar(vtype="B")

    for i in Vr:
        t[i.num] = model.addVar(vtype="C")
        td[i.num] = model.addVar(vtype="C")
        r[i.num] = model.addVar(vtype="C")
        rd[i.num] = model.addVar(vtype="C")
        w[i.num] = model.addVar(vtype="C")
        wd[i.num] = model.addVar(vtype="C")
    t[0] = 0
    td[0] = 0
    r[0] = 0
    rd[0] = 0
    w[0] = 0
    wd[0] = 0
    for i in V:
        u[i.num] = model.addVar(vtype="I", ub=n + 1)
    for i in N:
        for j in Vr:
            p[i, j.num] = model.addVar(vtype="B")  # Pのi,jの範囲、怪しい..
            p[0, j.num] = 1
    model.update()

    for j in N:
        model.addConstr(
            quicksum(x[i.num, j] for i in Vl if i.num != j) +
            quicksum(y[i.num, j, k.num] for i in Vl for k in Vr if P(i.num, j, k.num)) == 1
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
    for i in Vl:
        for j in Vr:
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
    for i in N:
        for j in N:
            for k in Vr:
                if P(i, j, k.num):
                    model.addConstr(
                        2 * y[i, j, k.num] <=
                        quicksum(x[h.num, i] for h in Vl if h.num != i) +
                        quicksum(x[l, k.num] for l in N if l != k)
                        , name="7"
                    )
    for j in N:
        for k in Vr:
            if P(0, j, k.num):
                model.addConstr(
                    y[0, j, k.num] <= quicksum(x[h.num, k.num] for h in Vl if h.num != k.num & h.num != j)
                    , name="8"
                )
    for i in Vl:
        for k in Vr:
            if i.num != k.num:
                model.addConstr(
                    u[k.num] - u[i.num] >= 1 - (n + 2) * (
                            1 - quicksum(y[i.num, j, k.num] for j in N if P(i.num, j, k.num)))
                    , name="9"
                )
    for i in Vl:
        model.addConstr(
            quicksum(y[i.num, j, k.num] for j in N for k in Vr if P(i.num, j, k.num)) <= 1
            , name="10"
        )
    for k in Vr:
        model.addConstr(
            quicksum(y[i.num, j, k.num] for i in Vl for j in N if P(i.num, j, k.num)) <= 1
            , name="11"
        )
    for i in N:
        for j in Vr:
            if i != j.num:
                model.addConstr(
                    u[i] - u[j.num] >= 1 - (n + 2) * p[i, j.num] -
                    M * (2 - quicksum(x[h.num, i] for h in Vl if h.num != i) - quicksum(
                        x[k, j.num] for k in N if k != j.num))
                    , name="12"
                )
    for i in N:
        for j in Vr:
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
    for i in Vl:
        for k in Vr:
            for l in N:
                if k.num != i.num & l != i.num & l != k.num:
                    model.addConstr(
                        u[l] >= u[k.num] - M *
                        (3 - quicksum(y[i.num, j, k.num] for j in N if j != i.num & P(i.num, j, k.num)) -
                         quicksum(y[l, m, nn.num] - p[i.num, l] for m in N for nn in Vr if
                                  m != i.num & m != k.num & m != l & nn.num != i.num & nn.num != k.num & P(l, m,
                                                                                                           nn.num)))
                        , name="16"
                    )
    for i in Vl:
        for k in Vr:
            if i.num != k.num:
                model.addConstr(
                    t[k.num] >= r[i.num] + tT[i.num, k.num] - M * (1 - x[i.num, k.num])
                    , name="17"
                )
                model.addConstr(
                    t[k.num] <= r[i.num] + tT[i.num, k.num] + M * (1 - x[i.num, k.num])
                    , name="18"
                )
    for j in V:
        for i in Vl:
            if j.num != i.num:
                model.addConstr(
                    td[j.num] >= r[i.num] + tD[i.num, j.num] - M * (
                            1 - quicksum(y[i.num, j.num, k.num] for k in Vr if P(i.num, j.num, k.num)))
                    , name="19"
                )
                model.addConstr(
                    td[j.num] <= r[i.num] + tD[i.num, j.num] + M * (
                            1 - quicksum(y[i.num, j.num, k.num] for k in Vr if P(i.num, j.num, k.num)))
                    , name="20"
                )
    for j in V:
        for k in Vr:
            if j.num != k.num:
                model.addConstr(
                    td[k.num] >= rd[j.num] + tD[j.num, k.num] - M * (
                            1 - quicksum(y[i.num, j.num, k.num] for i in Vl if P(i.num, j.num, k.num)))
                    , name="21"
                )
                model.addConstr(
                    td[k.num] <= rd[j.num] + tD[j.num, k.num] + M * (
                            1 - quicksum(y[i.num, j.num, k.num] for i in Vl if P(i.num, j.num, k.num)))
                    , name="22"
                )
    for j in N:
        model.addConstr(
            td[j] >= rd[j] - M * (
                    1 - quicksum(y[i.num, j, k.num] for i in Vl for k in Vr if P(i.num, j, k.num)))
            , name="23"
        )
        model.addConstr(
            td[j] <= rd[j] + M * (
                    1 - quicksum(y[i.num, j, k.num] for i in Vl for k in Vr if P(i.num, j, k.num)))
            , name="24"
        )
    for k in Vr:
        model.addConstr(
            r[k.num] >= t[k.num] + Sl * quicksum(y[k.num, l, m.num] for l in N for m in Vr
                                                 if P(k.num, l, m.num)) +
            Sr * quicksum(y[i.num, j, k.num] for i in Vl for j in N if P(i.num, j, k.num)) -
            M * (1 - quicksum(y[i.num, j, k.num] for i in Vl for j in N if P(i.num, j, k.num)))
            , name="25"

        )

        model.addConstr(
            rd[k.num] >= td[k.num] + Sl * 1
            * quicksum(y[k.num, l, m.num] for l in N for m in Vr
                       if  P(k.num, l, m.num)) +
            Sr * quicksum(y[i.num, j, k.num] for i in Vl for j in N if P(i.num, j, k.num)) -
            M * (1 - quicksum(y[i.num, j, k.num] for i in Vl for j in N if P(i.num, j, k.num)))
            , name="26"
        )
    for k in Vr:
        for j in N:
            if j != k.num:
                for i in Vr:
                    if P(i.num, j, k.num):
                        model.addConstr(
                            rd[k.num] - (rd[j] - tD[i.num, j]) - Sl *
                            quicksum(
                                y[k.num, l, m.num] for l in N for m in Vr if
                                l != i.num & l != j & l != k.num & m.num != k.num & m.num != i.num & m.num != l
                                & P(k.num, l, m.num)
                            ) <= ep + M * (1 - y[i.num, j, k.num])
                            , name="27"
                        )
    for k in Vr:
        model.addConstr(
            w[k.num] >= 0, name="28"
        )
        model.addConstr(
            wd[k.num] >= 0, name="29"
        )
        model.addConstr(
            w[k.num] >= td[k.num] - t[k.num], name="30"
        )
        model.addConstr(
            wd[k.num] >= t[k.num] - td[k.num], name="31"
        )
    model.addConstr(
        w[0] == 0, name="32"
    )
    model.addConstr(
        wd[0] == 0, name="33"
    )
    for i in V:
        model.addConstr(
            r[i.num] == rd[i.num], name="34"
        )
    model.addConstr(
        t[0] == 0, name="35"
    )
    model.addConstr(
        td[0] == 0, name="36"
    )
    model.addConstr(
        r[0] == 0, name="37"
    )
    model.addConstr(
        rd[0] == 0, name="38"
    )
    for j in N:
        model.addConstr(
            p[0, j] == 1, name="42"
        )
    for i in V:
        model.addConstr(
            0 <= u[i.num], name="43_1"
        )
        model.addConstr(
            u[i.num] <= n + 1, name="43_2"
        )
        model.addConstr(
            t[i.num] >= 0, name="44"
        )
        model.addConstr(
            td[i.num] >= 0, name="45"
        )
        model.addConstr(
            r[i.num] >= 0, name="46"
        )
        model.addConstr(
            rd[i.num] >= 0, name="46"
        )

    model.setObjective(
        C1 * quicksum(dT[i.num, j.num] * x[i.num, j.num] for i in Vl for j in Vr if i.num != j.num) +
        C2 * quicksum((dD[i.num, j] + dD[j, k.num]) * y[i.num, j, k.num] for i in Vl for j in N for k in Vr if
                      (i.num != j) & P(i.num, j, k.num)) +
        _alpha * quicksum(w[i.num] for i in V) + _beta * quicksum(wd[i.num] for i in V)
        , GRB.MINIMIZE
    )

    # model.write("test2.lp")
    # model.optimize()
    # model.computeIIS()
    # model.write("forum2.sol")
    while True:
        model.optimize()
        edgesT = []
        edgesD = []
        for (i, j) in x:
            if x[i, j].X == 1:
                edgesT.append((i, j))
        for (i, j, k) in y:
            if y[i, j, k].X == 1:
                edgesD.append((i, j, k))
        if model.IsMIP:
            print("break")
            break
        for (i, j) in x:
            x[i, j].VType = "B"
        for (i, j, k) in y:
            y[i, j, k].VType = "B"
        model.update()
    print("Optimal tour:", edgesT)
    print("Optimal tour:", edgesD)


def make_data(n):
    """make_data: compute matrix distance based on euclidean distance"""
    S = Class.Sequence(n, None, None)
    V = []
    for i in range(0, n + 2):  # インスタンス化
        V.append(Class.Node(i))
    for i in range(0, n + 1):  # 0~n n+1はデポ(0)
        V[i].coordinate = [50 * random.random() + 10, 50 * random.random() + 10]
        if i == 0:
            V[i].next = V[i + 1]
        else:
            V[i].prev = V[i - 1]
            V[i].next = V[i + 1]
        S.nodes.append(V[i])
    V[n + 1].coordinate = V[0].coordinate
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
    return math.sqrt((xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2)


def main():
    n = 7  # 客の総数
    N = set(range(1, n + 1))
    seed = 1
    random.seed(seed)
    S, V = make_data(n)
    # for i in V:
    #     print(i.coordinate)
    solve_tsp_d(V, N, n)


if __name__ == "__main__":
    main()
