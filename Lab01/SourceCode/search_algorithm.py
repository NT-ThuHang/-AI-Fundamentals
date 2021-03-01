import math
import queue

import graphUI
from node_color import white, yellow, red, blue, purple, green, orange

"""
Feel free print graph, edges to console to get more understand input.
Do not change input parameters
Create new function/file if necessary
"""


def BFS(graph, edges, edge_id, start, goal):
    """
    BFS search
    """
    # TODO: your code
    Close = []  # is a set of considered vertices, traversed
    Pre = [0] * 10000  # Pre[u]=v means v is the previous node of u
    Open = queue.Queue()  # is the set of vertices to be considered in the next step in the queue
    Open.put(start)
    Pre[start] = start
    Close.append(start)
    graph[start][3] = red
    graphUI.updateUI()
    while (Open.qsize() != 0):
        current_Node = Open.get()
        graph[current_Node][3] = yellow
        graphUI.updateUI()
        adjecent_node = graph[current_Node][1]
        for i in adjecent_node:
            if (i == goal):
                edges[edge_id(current_Node, i)][1] = white
                graph[goal][3] = purple
                graphUI.updateUI()
                Pre[i] = current_Node
                while True:
                    edges[edge_id(i, Pre[i])][1] = green
                    i = Pre[i]
                    graphUI.updateUI()
                    if (i == start):
                        graph[start][3] = orange
                        graphUI.updateUI()
                        break
                return
            if (i not in Close):
                Pre[i] = current_Node
                Open.put(i)
                Close.append(i)
                graph[i][3] = red
                edges[edge_id(current_Node, i)][1] = white
                graphUI.updateUI()
        graph[current_Node][3] = blue
        graphUI.updateUI()
    print("Don't Have path from ", start, " to ", goal)
    pass


def DFS(graph, edges, edge_id, start, goal):
    """
    DFS search
    """
    # TODO: your code
    Close = []  # is a set of considered vertices, traversed
    Pre = [0] * 10000  # Pre[u]=v means v is the previous node of u
    Open = []  # is the set of vertices to be considered in the next step in the stack
    # Add start into Open
    Open.append(start)
    Pre[start] = start
    # Update Graph
    graph[start][3] = red
    graphUI.updateUI()

    while (len(Open) != 0):
        current_Node = Open.pop()
        if (current_Node not in Close):
            Close.append(current_Node)
        # Update Graph
        graph[current_Node][3] = yellow
        graphUI.updateUI()
        # Take adjecent_node list
        adjecent_Node = graph[current_Node][1]
        for i in adjecent_Node:
            if (i == goal):
                edges[edge_id(current_Node, i)][1] = white
                graph[goal][3] = purple
                graphUI.updateUI()
                Pre[i] = current_Node
                while True:
                    edges[edge_id(i, Pre[i])][1] = green
                    i = Pre[i]
                    graphUI.updateUI()
                    if (i == start):
                        graph[start][3] = orange
                        graphUI.updateUI()
                        break
                return
            if (i not in Close):
                # add i into Open
                Open.append(i)
                Pre[i] = current_Node
                graph[i][3] = red
                edges[edge_id(i, current_Node)][1] = white
                graphUI.updateUI()
        graph[current_Node][3] = blue
        graphUI.updateUI()
    print("Don't Have path from ", start, " to ", goal)
    pass


def UCS(graph, edges, edge_id, start, goal):
    """
    Uniform Cost Search search
    """
    # TODO: your code

    Close = []
    Pre = [0] * 1000
    Open = queue.PriorityQueue()

    Open.put((0, (start, start)))
    graph[start][3] = red
    graphUI.updateUI()
    while (Open.qsize() != 0):
        x = Open.get()
        cost = x[0]
        u = x[1][0]
        v = x[1][1]
        if (u not in Close):
            Close.append(u)
            Pre[u] = v
            if (u == goal):
                graph[goal][3] = purple
                graphUI.updateUI()
                Pre[u] = v
                while True:
                    edges[edge_id(u, Pre[u])][1] = green
                    graphUI.updateUI()
                    u = Pre[u]
                    if (u == start):
                        graph[start][3] = orange
                        graphUI.updateUI()
                        break
                return
            adjecent_node = graph[u][1]
            for i in adjecent_node:
                if (i not in Close):
                    Pre[i] = u
                    graph[i][3] = red
                    edges[edge_id(i, u)][1] = white
                    graphUI.updateUI()
                    x1 = graph[u][0]
                    x2 = graph[i][0]
                    cost_i = cost + int(math.sqrt(pow((x1[0] - x2[0]), 2) + pow((x1[1] - x2[1]), 2)))
                    Open.put((cost_i, (i, u)))
        graph[u][3] = blue
        graphUI.updateUI()

    print("Don't Have path from ", start, " to ", goal)
    pass


def dist(a, b, graph):
    distance = math.sqrt(pow(graph[a][0][0] - graph[b][0][0], 2) + pow(graph[a][0][1] - graph[b][0][1], 2))
    return int(distance)


def Key_1(a):
    return a[1]


def AStar(graph, edges, edge_id, start, goal):
    """
            A star search
            """
    # TODO: your code
    # hx means the estimated movement cost to move from this vertice to the goal
    # gx means the movemen cost from start to this vertice

    # Create lists for open nodes and closed nodes
    open = []
    close = []
    # Compute  the estimated movement cost
    N = len(graph)  # N is the number of Vertices
    hx = []
    fx = [10e7] * N
    gx = [0] * N

    Pre = [-1] * N
    for i in range(N):
        hx.append(dist(i, goal, graph))

    # Add the start node
    open.append((start, 0, start))
    graph[start][3] = red
    graphUI.updateUI()
    while (len(open) != 0):
        # Sort the open list to get the node with the lowest cost first
        open.sort(key=Key_1)
        # Get the node with the lowest cost
        current_node = open.pop(0)
        graph[current_node[0]][3] = yellow
        graphUI.updateUI()
        # Add the current node to the closed list
        close.append(current_node[0])
        Pre[current_node[0]] = current_node[2]
        # Check if we have reached the goal, return the path
        if (current_node[0] == goal):
            graph[goal][3] = purple
            graphUI.updateUI()
            u = current_node[0]
            v = current_node[2]
            Pre[u] = v
            while True:
                edges[edge_id(u, Pre[u])][1] = green
                graphUI.updateUI()
                u = Pre[u]
                if (u == start):
                    graph[start][3] = orange
                    graphUI.updateUI()
                    break
            return

        adjecent_node = graph[current_node[0]][1]
        for next_node in adjecent_node:
            if (next_node not in close):
                if (next_node in open and fx[next_node] > gx[current_node[0]] + dist(current_node[0], next_node,
                                                                                     graph) + hx[next_node]):
                    gx[next_node] = gx[current_node] + dist(current_node[0], next_node, graph)
                    fx[next_node] = gx[next_node] + hx[next_node]
                    Pre[next_node] = current_node[0]
                    for node in open:
                        if (node[0] == next_node):
                            node[1] = fx[next_node]
                            node[2] = current_node[0]


                else:
                    graph[next_node][3] = red
                    edges[edge_id(current_node[0], next_node)][1] = white
                    graphUI.updateUI()
                    gx[next_node] = gx[current_node[0]] + dist(current_node[0], next_node, graph)
                    fx[next_node] = gx[next_node] + hx[next_node]
                    Pre[next_node] = current_node[0]
                    open.append((next_node, fx[next_node], current_node[0]))
        graph[current_node[0]][3] = blue
        graphUI.updateUI()

    print("Don't Have path from ", start, " to ", goal)
    pass


def GreedySearch(graph, edges, edge_id, start, goal):
    """
    Greedy Search
    """
    # TODO: your code

    Close = []
    Pre = [0] * 1000
    Open = queue.PriorityQueue()

    Open.put((dist(start,goal,graph), (start, start)))
    graph[start][3] = red
    graphUI.updateUI()
    while (Open.qsize() != 0):
        x = Open.get()
        cost = x[0]
        u = x[1][0]
        v = x[1][1]
        if (u not in Close):
            Close.append(u)
            Pre[u] = v
            if (u == goal):
                graph[goal][3] = purple
                graphUI.updateUI()
                Pre[u] = v
                while True:
                    edges[edge_id(u, Pre[u])][1] = green
                    graphUI.updateUI()
                    u = Pre[u]
                    if (u == start):
                        graph[start][3] = orange
                        graphUI.updateUI()
                        break
                return
            adjecent_node = graph[u][1]
            for i in adjecent_node:
                if (i not in Close):
                    Pre[i] = u
                    graph[i][3] = red
                    edges[edge_id(i, u)][1] = white
                    graphUI.updateUI()
                    x1 = graph[u][0]
                    x2 = graph[i][0]
                    cost_i = cost + dist(i,goal,graph)
                    Open.put((cost_i, (i, u)))
        graph[u][3] = blue
        graphUI.updateUI()

    print("Don't Have path from ", start, " to ", goal)
    pass


def BestFirstSearch(graph, edges, edge_id, start, goal):
    """
    Best First Search
    """
    Close = []
    Open = queue.PriorityQueue()
    Pre = [-1] * len(graph)
    Open.put((0,start))
    Pre[start]=start
    Close.append(start)
    graph[start][3]=red
    graphUI.updateUI()
    while (Open.qsize()!=0):
        current_node = Open.get()[1]
        graph[current_node][3]=yellow
        graphUI.updateUI()
        if (current_node==goal):
            graph[current_node][3] = purple
            graphUI.updateUI()
            while True:
                edges[edge_id(current_node, Pre[current_node])][1] = green
                graphUI.updateUI()
                current_node = Pre[current_node]
                if (current_node == start):
                    graph[current_node][3] = orange
                    graphUI.updateUI()
                    break
            return
        adjecent_node = graph[current_node][1]
        for next_node in adjecent_node:
            if (next_node not in Close):
                graph[next_node][3]=red
                edges[edge_id(current_node,next_node)][1]=white
                graphUI.updateUI()
                Open.put((dist(current_node,next_node,graph),next_node))
                Close.append(next_node)
                Pre[next_node]=current_node
        graph[current_node][3]=blue
        graphUI.updateUI()

    print("Don't Have path from ", start, " to ", goal)
    pass



def example_func(graph, edges, edge_id, start, goal):
    """
            This function is just show some basic feature that you can use your project.
            @param graph: list - contain information of graph (same value as global_graph)
                            list of object:
                             [0] : (x,y) coordinate in UI
                             [1] : adjacent node indexes
                             [2] : node edge color
                             [3] : node fill color
                        Ex: graph = [
                                        [
                                            (139, 140),             # position of node when draw on UI
                                            [1, 2],                 # list of adjacent node
                                            (100, 100, 100),        # grey - node edged color
                                            (0, 0, 0)               # black - node fill color
                                        ],
                                        [(312, 224), [0, 4, 2, 3], (100, 100, 100), (0, 0, 0)],
                                        ...
                                    ]
                        It means this graph has Node 0 links to Node 1 and Node 2.
                        Node 1 links to Node 0,2,3 and 4.
            @param edges: dict - dictionary of edge_id: [(n1,n2), color]. Ex: edges[edge_id(0,1)] = [(0,1), (0,0,0)] : set color
                            of edge from Node 0 to Node 1 is black.
            @param edge_id: id of each edge between two nodes. Ex: edge_id(0, 1) : id edge of two Node 0 and Node 1
            @param start: int - start vertices/node
            @param goal: int - vertices/node to search
            @return:
            """

    # Ex1: Set all edge from Node 1 to Adjacency node of Node 1 is green edges.
    node_1 = graph[1]
    for adjacency_node in node_1[1]:
        edges[edge_id(1, adjacency_node)][1] = green
    graphUI.updateUI()

    # Ex2: Set color of Node 2 is Red
    graph[2][3] = red
    graphUI.updateUI()

    # Ex3: Set all edge between node in a array.
    path = [4, 7, 9]  # -> set edge from 4-7, 7-9 is blue
    for i in range(len(path) - 1):
        edges[edge_id(path[i], path[i + 1])][1] = blue
    graphUI.updateUI()
