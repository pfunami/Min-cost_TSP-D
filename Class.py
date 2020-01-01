class Node:
    def __init__(self, num):
        self.pos = num
        self.coordinate = (0, 0)
        self.prev = None
        self.next = None
        self.provider = ''

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

    def sub_sequence(self, i, j):  # 要確認
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
