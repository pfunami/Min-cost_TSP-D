import TSP_D as m
import Class as c
from gurobipy import *

_alpha = 10
_beta = 10


# cost of drone delivery
def cost(i, j, k):
    return m.C2 * (m.dD[i, j] + m.dD[j, k])


# waiting cost of truck at k
def wait_cost_truck(i, j, k):
    return _alpha * max(0, m.tT[i, k] - (m.tD[i, j] + m.tD[j, k]))


# waiting cost of truck at k
def wait_cost_drone(i, j, k):
    return _beta * max(0, (m.tD[i, j] + m.tD[j, k]) - m.tT[i, k])


# cost(TD), cost(DD)
def tour_cost(tour, series):
    if series == 'TD':
        return quicksum(m.C1 * m.dT[T[0], T[1]] for T in tour.edge)
    elif series == 'DD':
        return quicksum(cost(T.nodes[1], T.nodes[2], T.nodes[3]) for T in tour)


def wait_cost(DD):
    return quicksum(
        wait_cost_truck(T.nodes[1], T.nodes[2], T.nodes[3]) + wait_cost_drone(T.nodes[1], T.nodes[2], T.nodes[3])
        for T in DD)


def total_cost(TD, DD):
    return tour_cost(TD, 'TD') + tour_cost(DD, 'DD') + wait_cost(DD)


def sub_cost(tour):  # sub sequence
    i = tour.nodes[1]
    j = tour.nodes[2]
    k = tour.nodes[3]
    TD = c.Sequence(3, i, k)
    TD.nodes[2] = j
    DD = c.Sequence(2, i, k)
    return total_cost(TD, DD)
