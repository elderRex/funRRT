import sys
import math

class vertex:

    def __init__(self,pt):
        self.vex = pt
        self.adj_vex=[]
        self.dist = sys.maxsize
        self.prev = []

    def __str__(self):
        return str(self.vex)

    def add_adj(self,edges):
        for i in range(0,len(edges)):
            dist = edges[i][2]
            if edges[i][0] == self.vex:
                self.adj_vex.append([edges[i][1],dist])
            elif edges[i][1] == self.vex:
                self.adj_vex.append([edges[i][0],dist])
        return

def mycomp(verA, verB):
    if verA.dist < verB.dist:
        # A,B
        return -1
    elif verA.dist >= verB.dist:
        # B,A
        return 1

    return 0

def find(Q,v):
    for i in range(0,len(Q)):
        if v[0] == Q[i].vex:
            return [Q[i],i]

def find2(Q,v):
    for i in range(0,len(Q)):
        if v == Q[i].vex:
            return [Q[i],i]

def dij(edges,vertices):

    nodes = []
    for i in range(0,len(vertices)):
        #print vertices[i]
        newvex = vertex(vertices[i])
        newvex.add_adj(edges)
        nodes.append(newvex)
    nodes[0].dist = 0
    print nodes[0].vex
    Q = sorted(nodes, cmp=mycomp)

    while Q:
        # print Q
        Q = sorted(Q, cmp=mycomp)
        u = Q[0]
        # print Q[0]
        del Q[0]
        #print u.vex
        #print u.adj_vex
        for v in u.adj_vex:  # Look at all the nodes that this vertex is attached to
            # print '(' + v + ',' + u + ')'
            alt = u.dist + v[1]  # Alternative path distance
            #print v
            nvex = find(Q,v)
            #print nvex
            if nvex is not None:
                if alt < Q[nvex[1]].dist:  # If there is a new shortest path update our priority queue (relax)
                    Q[nvex[1]].dist = alt
                    Q[nvex[1]].prev = u.vex
    path = []
    u = len(vertices)-1
    cur_nd = nodes[u]
    print nodes[u].vex
    print nodes[u].prev
    while cur_nd != nodes[0]:  # Traverse through nodes til we reach the root which is 0
        path.append(cur_nd.prev)
        #for i in range(0,len(nodes)):
        #    print nodes[i]
        cur_nd = find2(nodes,cur_nd.prev)
        #print cur_nd
        cur_nd = nodes[cur_nd[1]]
    print path
    return path