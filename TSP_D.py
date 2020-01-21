from gurobipy import *
import networkx as nx
import random
import matplotlib

# matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import Class
import CostFunc as f
import copy

EPS = 1.e-6
_biasForDist = 1.5
_dSpeed = 45 / 60  # 時速70→分速km
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
ep = 30.0


def solve_tsp_d(V, N, n, UAV):
    def possible(_uav, _i, _j, _k):
        if _i == (n + 1):
            return False
        if ((_i % (n + 1)) == (_j % (n + 1))) | ((_j % (n + 1)) == (_k % (n + 1))) | ((_i % (n + 1)) == (_k % (n + 1))):
            return False
        else:
            S = set(v.num for v in V)
            if (_i in S) & (_k in S) & (_j in N) & (_i != _j) & (_i != k) & (_j != _k):
                if tD[_uav, _i, _j] + tD[_uav, _j, _k] <= ep:
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

    for uav in UAV:
        for i in Vl:
            for j in N:
                for k in Vr:
                    if (i.num != j) & (i.num != k.num) & (j != k.num):
                        y[uav, i.num, j, k.num] = model.addVar(vtype="B")

    for i in Vl:
        for j in Vr:
            if i.num != j.num:
                x[i.num, j.num] = model.addVar(vtype="B")

    for uav in UAV:
        for i in Vr:
            td[uav, i.num] = model.addVar(vtype="C")
            rd[uav, i.num] = model.addVar(vtype="C")
            wd[uav, i.num] = model.addVar(vtype="C")
        td[uav, 0] = 0
        rd[uav, 0] = 0
        wd[uav, 0] = 0
    for i in Vr:
        t[i.num] = model.addVar(vtype="C")
        r[i.num] = model.addVar(vtype="C")
        w[i.num] = model.addVar(vtype="C")
    t[0] = 0
    r[0] = 0
    w[0] = 0
    for i in V:
        u[i.num] = model.addVar(vtype="I", ub=n + 1)
    for i in N:
        for j in Vr:
            p[i, j.num] = model.addVar(vtype="B")  # Pのi,jの範囲、怪しい..
            p[0, j.num] = 1
    model.update()

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
            if i.num != k.num:
                model.addConstr(
                    t[k.num] >= r[i.num] + tT[i.num, k.num] - M * (1 - x[i.num, k.num])
                    , name="17"
                )
                model.addConstr(
                    t[k.num] <= r[i.num] + tT[i.num, k.num] + M * (1 - x[i.num, k.num])
                    , name="18"
                )

    for k in Vr:
        model.addConstr(
            w[k.num] >= 0, name="28"
        )
        for uav in UAV:
            model.addConstr(
                w[k.num] >= td[uav, k.num] - t[k.num], name="30"
            )
            model.addConstr(
                wd[uav, k.num] >= 0, name="29"
            )
            model.addConstr(
                wd[uav, k.num] >= t[k.num] - td[uav, k.num], name="31"
            )
    model.addConstr(
        w[0] == 0, name="32"
    )
    for uav in UAV:
        model.addConstr(
            wd[uav, 0] == 0, name="33"
        )
        for i in V:
            model.addConstr(
                r[i.num] == rd[uav, i.num], name="34"
            )
    model.addConstr(
        t[0] == 0, name="35"
    )
    for uav in UAV:
        model.addConstr(
            td[uav, 0] == 0, name="36"
        )
    model.addConstr(
        r[0] == 0, name="37"
    )
    for uav in UAV:
        model.addConstr(
            rd[uav, 0] == 0, name="38"
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
            r[i.num] >= 0, name="46"
        )
        for uav in UAV:
            model.addConstr(
                rd[uav, i.num] >= 0, name="46"
            )
            model.addConstr(
                td[uav, i.num] >= 0, name="45"
            )

    for j in N:
        model.addConstr(
            quicksum(x[i.num, j] for i in Vl if i.num != j) + quicksum(
                y[uav, i.num, j, k.num] for uav in UAV for i in Vl for k in Vr if
                possible(uav, i.num, j, k.num)) == 1
            , name="2"
        )
    # for i in Vl:
    #     for j in N:
    #         for k in Vr:
    #             if possible(1, i.num, j, k.num):
    #                 model.addConstr(
    #                     quicksum(y[uav, i.num, j, k.num] for uav in UAV) <= 1
    #                     , name="new constr for D"
    #                 )
    for uav in UAV:
        for i in Vl:
            model.addConstr(
                quicksum(
                    y[uav, i.num, j, k.num] for j in N for k in Vr if possible(uav, i.num, j, k.num)) <= 1
                , name="10"
            )
        for k in Vr:
            model.addConstr(
                quicksum(
                    y[uav, i.num, j, k.num] for i in Vl for j in N if possible(uav, i.num, j, k.num)) <= 1
                , name="11"
            )
    for k in Vr:
        for i in Vl:
            for uav in UAV:
                ii = 0
                if i.num == 0:
                    ii = n + 1
                else:
                    ii = i.num
                model.addConstr(
                    quicksum(y[uav, i.num, j, k.num] + y[uav, k.num % (n + 1), j, ii] for j in N if
                             possible(uav, i.num, j, k.num)) <= 1, name="47 and add for multi"
                )
    for uav in UAV:
        for i in Vl:
            for k in Vr:
                for l in N:
                    if k.num != i.num & l != i.num & l != k.num:
                        model.addConstr(
                            u[l] >= u[k.num] - M *
                            (3 - quicksum(
                                y[uav, i.num, j, k.num] for j in N if j != i.num & possible(uav, i.num, j, k.num)) -
                             quicksum(y[uav, l, m, nn.num] - p[i.num, l] for m in N for nn in Vr if
                                      m != i.num & m != k.num & m != l & nn.num != i.num & nn.num != k.num & possible(
                                          uav, l,
                                          m,
                                          nn.num)))
                            , name="16"
                        )

    for uav in UAV:
        for i in N:
            for j in N:
                for k in Vr:
                    if possible(uav, i, j, k.num):
                        model.addConstr(
                            2 * y[uav, i, j, k.num] <=
                            quicksum(x[h.num, i] for h in Vl if h.num != i) +
                            quicksum(x[l, k.num] for l in N if l != k.num)
                            , name="7"
                        )
        for j in N:
            for k in Vr:
                if possible(uav, 0, j, k.num):
                    model.addConstr(
                        y[uav, 0, j, k.num] <= quicksum(x[h.num, k.num] for h in Vl if h.num != k.num & h.num != j)
                        , name="8"
                    )
        for i in Vl:
            for k in Vr:
                if i.num != k.num:
                    model.addConstr(
                        u[k.num] - u[i.num] >= 1 - (n + 2) * (
                                1 - quicksum(y[uav, i.num, j, k.num] for j in N if possible(uav, i.num, j, k.num)))
                        , name="9"
                    )

        for j in V:
            for i in Vl:
                if j.num != i.num:
                    model.addConstr(
                        td[uav, j.num] >= r[i.num] + tD[uav, i.num, j.num] - M * (
                                1 - quicksum(
                            y[uav, i.num, j.num, k.num] for k in Vr if possible(uav, i.num, j.num, k.num)))
                        , name="19"
                    )
                    model.addConstr(
                        td[uav, j.num] <= r[i.num] + tD[uav, i.num, j.num] + M * (
                                1 - quicksum(
                            y[uav, i.num, j.num, k.num] for k in Vr if possible(uav, i.num, j.num, k.num)))
                        , name="20"
                    )
        for j in V:
            for k in Vr:
                if j.num != k.num:
                    model.addConstr(
                        td[uav, k.num] >= rd[uav, j.num] + tD[uav, j.num, k.num] - M * (
                                1 - quicksum(
                            y[uav, i.num, j.num, k.num] for i in Vl if possible(uav, i.num, j.num, k.num)))
                        , name="21"
                    )
                    model.addConstr(
                        td[uav, k.num] <= rd[uav, j.num] + tD[uav, j.num, k.num] + M * (
                                1 - quicksum(
                            y[uav, i.num, j.num, k.num] for i in Vl if possible(uav, i.num, j.num, k.num)))
                        , name="22"
                    )
        for j in N:
            model.addConstr(
                td[uav, j] >= rd[uav, j] - M * (
                        1 - quicksum(y[uav, i.num, j, k.num] for i in Vl for k in Vr if possible(uav, i.num, j, k.num)))
                , name="23"
            )
            model.addConstr(
                td[uav, j] <= rd[uav, j] + M * (
                        1 - quicksum(y[uav, i.num, j, k.num] for i in Vl for k in Vr if possible(uav, i.num, j, k.num)))
                , name="24"
            )
        for k in Vr:
            model.addConstr(
                r[k.num] >= t[k.num] + Sl * quicksum(y[uav, k.num, l, m.num] for l in N for m in Vr
                                                     if possible(uav, k.num, l, m.num)) +
                Sr * quicksum(y[uav, i.num, j, k.num] for i in Vl for j in N if possible(uav, i.num, j, k.num)) -
                M * (1 - quicksum(y[uav, i.num, j, k.num] for i in Vl for j in N if possible(uav, i.num, j, k.num)))
                , name="25"

            )

            model.addConstr(
                rd[uav, k.num] >= td[uav, k.num] + Sl * 1
                * quicksum(y[uav, k.num, l, m.num] for l in N for m in Vr
                           if possible(uav, k.num, l, m.num)) +
                Sr * quicksum(y[uav, i.num, j, k.num] for i in Vl for j in N if possible(uav, i.num, j, k.num)) -
                M * (1 - quicksum(y[uav, i.num, j, k.num] for i in Vl for j in N if possible(uav, i.num, j, k.num)))
                , name="26"
            )
        for k in Vr:
            for j in N:
                if j != k.num:
                    for i in Vr:
                        if possible(uav, i.num, j, k.num):
                            model.addConstr(
                                rd[uav, k.num] - (rd[uav, j] - tD[uav, i.num, j]) - Sl *
                                quicksum(
                                    y[uav, k.num, l, m.num] for l in N for m in Vr if
                                    l != i.num & l != j & l != k.num & m.num != k.num & m.num != i.num & m.num != l
                                    & possible(uav, k.num, l, m.num)
                                ) <= ep + M * (1 - y[uav, i.num, j, k.num])
                                , name="27"
                            )

    model.setObjective(
        C1 * quicksum(dT[i.num, j.num] * x[i.num, j.num] for i in Vl for j in Vr if i.num != j.num) +
        C2 * quicksum(
            (dD[uav, i.num, j] + dD[uav, j, k.num]) * y[uav, i.num, j, k.num] for uav in UAV for i in Vl for j in N for
            k in Vr if (i.num != j) & possible(uav, i.num, j, k.num)) +
        _alpha * quicksum(w[i.num] for i in V) + _beta * quicksum(wd[uav, i.num] for uav in UAV for i in V)
        , GRB.MINIMIZE
    )
    model.update()

    # model.write("test2.lp")
    # model.optimize()
    # model.computeIIS()
    # model.write("forum2.sol")
    def che(it):
        if it == n + 1:
            return 0
        else:
            return it

    while True:
        model.optimize()
        edgesT = []
        # edgesD = [[]] * len(UAV)
        edgesD = {}
        for uav in UAV:
            edgesD[uav] = []
        edge = []
        for (i, j) in x:
            if x[i, j].X == 1:
                edgesT.append((che(i), che(j)))
        for (uav, i, j, k) in y:
            if y[uav, i, j, k].X == 1:
                edge.append((uav, che(i), che(j), che(k)))
        for ed in edge:
            edgesD[ed[0]].append((ed[1], ed[2], ed[3]))
        if model.IsMIP:
            print("time: ", )
            break
        for (i, j) in x:
            x[i, j].VType = "B"
        for (uav, i, j, k) in y:
            y[uav, i, j, k].VType = "B"
        model.update()
    print("Truck tour:", edgesT)
    print("Drone tour:", edgesD)
    print(model.ObjVal)
    return edgesT, edgesD


def make_data(n, UAV):
    """make_data: compute matrix distance based on euclidean distance"""
    S = Class.Sequence(n, None, None)
    V = []
    for i in range(0, n + 2):  # インスタンス化
        V.append(Class.Node(i))
    for i in range(0, n + 1):  # 0~n n+1はデポ(0)
        V[i].coordinate = [30 * random.random() + 1, 30 * random.random() + 1]
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
            for uav in UAV:
                dD[uav, i.num, j.num] = distance(i.coordinate, j.coordinate)
                tD[uav, i.num, j.num] = dD[uav, i.num, j.num] / _dSpeed
            dT[i.num, j.num] = dD[UAV[0], i.num, j.num] * _biasForDist
            tT[i.num, j.num] = dT[i.num, j.num] / _tSpeed
    return S, V


def distance(xy1, xy2):
    """distance: euclidean distance between (x1,y1) and (x2,y2)"""
    return math.sqrt((xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2)


def visualize_visit_order(edgesT, edgesD, V, UAV):
    plt.close()  # 一旦閉じる！！！
    """Visualize traveling path for given visit order"""
    cus_xy = {}
    n = len(V) - 1

    for v in V:
        if v.num != (n + 1):
            cus_xy[v.num] = v.coordinate
    G1 = nx.Graph()
    G2 = {}
    color1 = []
    color2 = {}
    for uav in UAV:
        color2[uav] = []
    G1.add_edges_from(edgesT)
    i = 0
    for edD in edgesD.values():
        newD = []
        for v in edD:
            newD.append((v[0], v[1]))
            newD.append((v[1], v[2]))
        G2[i] = nx.Graph()
        G2[i].add_edges_from(newD)
        i += 1
    for node in G1:
        if node == 0:
            color1.append("r")
        else:
            color1.append("y")
    i = 1
    for g2 in G2.values():
        print(g2)
        for node in g2:
            if node == 0:
                color2[i].append("r")
            else:
                color2[i].append("y")
        i += 1
    colors = ["firebrick", "r", "darkgreen", "gold", "purple", "orange", "dodgerblue"]
    nx.draw_networkx_nodes(G1, pos=cus_xy, node_color=color1)
    nx.draw_networkx_edges(G1, pos=cus_xy)
    nx.draw_networkx_labels(G1, pos=cus_xy)
    i = 1
    for g2 in G2.values():
        print(colors[i])
        nx.draw_networkx_edges(g2, pos=cus_xy, style="dashdot", edge_color=colors[i])
        nx.draw_networkx_nodes(g2, pos=cus_xy, node_color=color2[i])
        nx.draw_networkx_labels(g2, pos=cus_xy)
        i += 1
    plt.show()
    # plt.savefig('myplot4.png')


def main():
    UAVnum = 1
    if UAVnum == 0:
        global C2
        C2 = GRB.INFINITY
        UAVnum = 1
    n = 25  # 客の総数
    N = set(range(1, n + 1))
    # seed = 222
    seed = 222
    random.seed(seed)
    UAV = [i for i in range(1, UAVnum + 1)]
    S, V = make_data(n, UAV)
    edT, edD = solve_tsp_d(V, N, n, UAV)
    # edT = [(0, 6), (1, 8), (3, 1), (5, 10), (6, 9), (8, 5), (9, 3), (10, 13), (13, 0)]
    # edD = {1: [(3, 11, 1), (5, 7, 10), (6, 4, 9)], 2: [(0, 14, 13), (1, 2, 8), (8, 12, 5), (13, 15, 0)]}
    visualize_visit_order(edT, edD, V, UAV)
    print("UAVnum: ", UAVnum)
    exit(0)


if __name__ == "__main__":
    main()
