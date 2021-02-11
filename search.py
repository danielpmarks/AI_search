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
        # print(agent.position, "added to explored list")
        i = agent.position[0]
        j = agent.position[1]

        for cell in maze.neighbors(i, j):
            if cell not in explored and cell not in frontier:
                # print(cell, cell in explored)
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
        # print(agent.position, "added to explored list")
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
                    # print(cell, cell in explored)
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

        def __init__(self, position, parent, distance, mstDist, k, explored, goalCount, goals):
            self.position = position
            self.parent = parent
            self.distance = distance
            self.goals = goals
            self.mstDist = mstDist
            self.k = k
            self.explored = explored
            self.goalCount = goalCount
            self.goals = goals

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
        if len(mst) == 0:
            return 0
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

    def g(node):
        return node.distance + node.mstDist

    def findSet(sets, node):
        i = 0
        for dSet in sets:
            if node in dSet:
                return i
            i += 1
        return -1

    def getGoals(goals, k):
        needs = []
        i = 0
        for g in k:
            if g == 0:
                needs.append(goals[i])
            i += 1
        return needs

    def hashPoint(i, j, k):
        out = ""
        out += str(i)
        out += str(j)
        for num in k:
            out += str(num)
        return out

    def hasAllGoals(goals):
        for i in goals:
            if i != 1:
                return False
        return True

    def findMST(goals):
        class edge:
            def __init__(self, nodes, cost):
                self.nodes = nodes
                self.cost = cost
        # create a tree of all edges based on their manhattan distances
        if len(goals) <= 1:
            return []
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
        sets = []
        i = 0
        while len(nodes) < len(goals) or len(sets) > 1:

            edge = edges[i]
            # print(edge.nodes)
            # neither node is in a set
            if edge.nodes[0] not in nodes and edge.nodes[1] not in nodes:
                mst.append(edge)
                # add nodes to the node list
                nodes.append(edge.nodes[0])
                nodes.append(edge.nodes[1])
                # create new set
                addSet = {edge.nodes[0], edge.nodes[1]}
                sets.append(addSet)
            # one node is in a set, add the other to that set
            elif not (edge.nodes[0] in nodes and edge.nodes[1] in nodes):
                mst.append(edge)
                if edge.nodes[0] not in nodes:
                    nodes.append(edge.nodes[0])
                    # add to the other node's set
                    sets[findSet(sets, edge.nodes[1])].add(edge.nodes[0])
                if edge.nodes[1] not in nodes:
                    nodes.append(edge.nodes[1])
                    # add to the other node's set
                    sets[findSet(sets, edge.nodes[0])].add(edge.nodes[1])
            # both nodes are in a set, merge the sets if they are different
            else:
                set1 = findSet(sets, edge.nodes[0])
                set2 = findSet(sets, edge.nodes[1])
                if set1 != set2:
                    mst.append(edge)
                    # print(sets[set1])
                    sets[set1].update(sets[set2])
                    # print(sets[set1])
                    sets.pop(set2)
            i += 1

        # print(nodes)
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
    numGoals = len(goals)
    k = [0 for goal in goals]
    # set up starting state of the search
    mst = findMST(goals)
    state = {}
    agent = node(maze.start, None, 0, mstLen(mst), k, [], 0, goals)
    frontier = [(0, agent)]

    #start_time = time.time()

    if maze.size[0] > 40 and len(goals) > 5:
        return []

    while len(frontier) != 0:
        agent = frontier.pop(0)[1]
        # print(agent.k)
        # if this state has been explored with a lower cost, then skip
        key = hashPoint(agent.position[0], agent.position[1], agent.k)
        if key in state and state[key].distance <= agent.distance:
            continue
        state[key] = agent
        agent.explored.append(agent.position)
        a_goals = agent.goals
        if agent.position in a_goals:
            # remove the goal from the waypoints list
            new_k = agent.k.copy()
            new_k[goals.index(agent.position)] = 1
            agent.k = new_k
            new_goals = agent.goals.copy()
            new_goals.remove(agent.position)
            # recalculate MST and MST length
            mst = findMST(new_goals)
            agent.mstDist = mstLen(mst)
            # hop to new k plane
            agent.explored = []
            agent.goalCount += 1
            agent.goals = new_goals

        if agent.goalCount == numGoals:
            break

        i = agent.position[0]
        j = agent.position[1]

        # update the frontier
        for cell in maze.neighbors(i, j):
            if cell not in agent.explored:
                newNode = node(cell, agent, agent.distance + 1,
                               agent.mstDist, agent.k, agent.explored, agent.goalCount, agent.goals.copy())
                # print(cell, g(newNode))
                f = nearest_neighbor(cell, agent.goals) + \
                    newNode.mstDist + newNode.distance
                # print(cell, f)
                frontier.append((f, newNode))

        # sort the next state tree
        frontier.sort(key=sortNodes)

    path = []
    # print(agent.goals)
    while agent != None:
        path.insert(0, agent.position)
        agent = agent.parent

    # print(maze.validate_path(path))
    # path.insert(0, maze.start)
    # print(path)
    return path


def fast(maze):
    """
    Runs suboptimal search algorithm for part 5.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    return []
