# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Kelvin Ma (kelvinm2@illinois.edu) on 01/24/2021

"""
This is the main entry point for MP1. You should only modify code
within this file -- the unrevised staff files will be used for all other
files and classes when code is run, so be careful to not modify anything else.
"""
# Search should return the path.
# The path should be a list of tuples in the form (row, col) that correspond
# to the positions of the path taken by your search algorithm.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,astar,astar_multi,fast)


def bfs(maze):
    """
    Runs BFS for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """

    class node:
        def __init__(self, position, parent):
            self.position = position
            self.parent = parent

    explored = []
    tree = []
    frontier = []

    agent = node(maze.start, None)
    goal = maze.waypoints[0]

    while agent.position != goal:
        explored.append(agent.position)
        #print(agent.position, "added to explored list")
        i = agent.position[0]
        j = agent.position[1]

        for cell in maze.neighbors(i, j):
            if cell not in explored and cell not in frontier:
                #print(cell, cell in explored)
                frontier.append(cell)
                tree.append(node(cell, agent))

        agent = tree.pop(0)

    # agent is at goal, reverse up the tree
    path = []
    start = maze.start
    while agent.position != start:
        path.insert(0, agent.position)
        agent = agent.parent
    path.insert(0, start)

    return path


def astar_single(maze):
    """
    Runs A star for part 2 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    class node:
        def __init__(self, position, parent, dist):
            self.position = position
            self.parent = parent
            self.dist = dist

    def h(goal, cell):
        return abs(goal[0]-cell[0]) + abs(goal[1]-cell[1])

    def sorter(node):
        return node[0]

    def findInFrontier(cell, frontier):
        i = 0
        for point in frontier:
            if cell == point[1].position:
                return i
            i += 1
        return -1

    explored = []
    frontier = []

    goal = maze.waypoints[0]
    agent = node(maze.start, None, 0)

    while agent.position != goal:
        explored.append(agent.position)
        #print(agent.position, "added to explored list")
        i = agent.position[0]
        j = agent.position[1]

        for cell in maze.neighbors(i, j):
            if cell not in explored:
                check = h(goal, cell) + agent.dist
                idx = findInFrontier(cell, frontier)
                if idx != -1:
                    if frontier[idx][0] > check:
                        replace = frontier.pop(idx)[1]
                        frontier.append((check, replace))
                else:
                    #print(cell, cell in explored)
                    frontier.append((check,
                                     node(cell, agent, agent.dist + 1)))
        frontier.sort(key=sorter)
        agent = frontier.pop(0)[1]

    # agent is at goal, reverse up the tree
    path = []
    start = maze.start
    while agent.position != start:
        path.insert(0, agent.position)
        agent = agent.parent
    path.insert(0, start)

    return path


def astar_corner(maze):
    """
    Runs A star for part 3 of the assignment in the case where there are four corner objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
        """

    return astar_multiple(maze)


def astar_multiple(maze):
    """
    Runs A star for part 4 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """

    class node:

        def __init__(self, position, parent, distance, goals, mstDist, explored):
            self.position = position
            self.parent = parent
            self.distance = distance
            self.goals = goals
            self.mstDist = mstDist
            self.explored = explored

        """
        def __init__(self, copyNode):
            self.position = copyNode.position
            self.parent = copyNode.parent
            self.distance = copyNode.distance
            self.goals = copyNode.goals
            self.mst = copyNode.mst
            self.paths = copyNode.paths
            """

    def manhattan(loc1, loc2):
        return abs(loc1[0]-loc2[0]) + abs(loc1[1]-loc2[1])

    def mstLen(mst):
        mst_len = 0
        for edge in mst:
            mst_len += edge.cost
        return mst_len

    def nearest_neighbor(cell, goals):
        nearest_neighbor = maze.size[0] + maze.size[1]
        for goal in goals:
            dist = manhattan(cell, goal)
            if dist < nearest_neighbor:
                nearest_neighbor = dist
        return nearest_neighbor

    # used to sort the edges for the mst
    def sortEdges(edge):
        return edge.cost

    def findInFrontier(cell, frontier):
        i = 0
        for point in frontier:
            if cell == point[1].position:
                return i
            i += 1
        return -1

    def findMST(goals):
        class edge:
            def __init__(self, nodes, cost):
                self.nodes = nodes
                self.cost = cost
        # create a tree of all edges based on their manhattan distances
        edges = []
        for i in range(len(goals)):
            j = i + 1
            while j < len(goals):
                newEdge = edge([goals[i], goals[j]],
                               manhattan(goals[i], goals[j]))
                edges.append(newEdge)
                j += 1

        # sort the edges by their cost
        edges.sort(key=sortEdges)

        # add all the edges to a minimum spanning tree
        mst = []
        nodes = []
        i = 0
        while len(nodes) < len(goals) and i < len(edges):
            edge = edges[i]
            if not (edge.nodes[0] in nodes and edge.nodes[1] in nodes):
                mst.append(edge)
                if edge.nodes[0] not in nodes:
                    nodes.append(edge.nodes[0])
                if edge.nodes[1] not in nodes:
                    nodes.append(edge.nodes[1])
            i += 1
        return mst

    def printMST(mst):
        nodes = []
        for edge in mst:
            nodes.append(edge.nodes)
        print(nodes)

    # used to sort the next state tree
    def sortNodes(node):
        return node[0]

    goals = list(maze.waypoints)

    # set up starting state of the search
    explored = [maze.start]
    agent = node(maze.start, None, 0, goals,
                 mstLen(findMST(goals)), explored)
    frontier = []

    while len(agent.goals) != 0:
        agent.explored.append(agent.position)
        #print(agent.position, agent.goals)
        #print(agent.position, agent.goals)
        #print(agent.position, "added to explored list")
        i = agent.position[0]
        j = agent.position[1]

        # update the frontier
        for cell in maze.neighbors(i, j):
            if cell not in agent.explored:
                h = nearest_neighbor(cell, agent.goals) + \
                    agent.mstDist + agent.distance

                idx = findInFrontier(cell, frontier)
                if idx != -1:
                    if frontier[idx][0] > h:
                        replace = frontier.pop(idx)[1]
                        frontier.append((h, replace))
                else:
                    frontier.append((nearest_neighbor(cell, agent.goals) + agent.mstDist + agent.distance,
                                     node(cell, agent, agent.distance + 1, agent.goals.copy(), agent.mstDist, agent.explored)))
        # sort the next state tree
        frontier.sort(key=sortNodes)

        agent = frontier.pop(0)[1]

        if agent.position in agent.goals:
            # print(agent.position)
            currMst = findMST(agent.goals)
            # printMST(currMst)
            # print(mstLen(currMst))
            agent.goals.remove(agent.position)

            #print("Current mstDist", agent.mstDist)
            mst = findMST(agent.goals)
            # printMST(mst)
            # print(mstLen(mst))
            #print("New mstDist", mstLen(mst))
            agent.mstDist = mstLen(mst)
            agent.explored = [agent.position]

    path = []
    reached = []
    print(agent.goals)
    while agent != None:
        path.insert(0, agent.position)
        agent = agent.parent

    print(maze.validate_path(path))
    #path.insert(0, maze.start)
    return path


def fast(maze):
    """
    Runs suboptimal search algorithm for part 5.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    return []
