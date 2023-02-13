from importlib.resources import path
import json

with open(r"/Users/mohammedalmousa/Downloads/Assignment2/AllReg.json", "r", encoding="utf8") as f:
    graphjson = json.loads(f.read())
    graph = {}
    graph1 = {}
    vertices_no = 0



def add_vertex(v):
    global graph
    global vertices_no
    vertices_no = vertices_no + 1
    graph[v["name"]] = []
    graph1[v["name"]] = []


# -----------------------------------------------------------------
def add_edge(v1, v2, e):  # Add an edge between vertex v1 and v2 with edge weight e
    global graph

    # Since this code is not restricted to a directed or an undirected graph, an edge between v1 v2 does not imply that
    # an edge exists between v2 and v1
    temp = [v2, e]
    temp1 = [v2]
    graph[v1["name"]].append(temp)
    graph1[v1["name"]].extend(temp1)


# -----------------------------------------------------------------
for u in graphjson:
    add_vertex(u)

for u1 in graphjson:
    v1 = u1
    for tr in v1["neighbors"]:
        v2 = graphjson[tr["cid"]]["name"]
        e = tr["distance"]
        add_edge(v1, v2, e)


# ----------------------------------------------------------------
 ##def bfs(graph, start, goal):



# ----------------------------------------------------------------
def ucs(graph, start, goal):
    visited = []
    queue = [[(start, 0)]]
    ucs.genNodes = 1
    ucs.maxQueue = 1
    if start == goal:
        return "Start = Goal"
    while queue:
        # queue.sort(key=solcost)
        path = queue.pop(0)  # path=[ , , ,]
        node = path[-1][0]  # put the first element in path

        if node in visited:  # check if visited empty
            continue
        visited.append(node)  # add node to visited
        if node == goal:
            # print(solcost(path))
            return path
        else:

            directed_nodes = graph.get(node, [])
            # new queue shows every related node to current node
            ucs.genNodes += len(directed_nodes)
            if len(directed_nodes) > ucs.maxQueue:
                ucs.maxQueue = len(directed_nodes)
            for (nodein, distance) in directed_nodes:
                npath = path.copy()
                npath.append((nodein, distance))
                # copy current node with related nodes and put them in queue
                queue.append(npath)


def solcost(route):
    if route == "Start = Goal":
        return 0
    Total = 0
    for (n, distance) in route:
        Total += distance
    return Total





def path_cost(path):
    total_cost = 0
    for (node, cost) in path:
        total_cost += cost
    return total_cost, path[-1][0]
path = []
pathCost = []
newpath = []
visited2 = []
maxfringe = []
global Hg
Hg = 0


def ucs1(graph, start, goal):
    visited = []
    queue = [[(start, 0)]]
    ucs.num_node = 1
    ucs.max_fringe = 1
    if start == goal:
        return 'No solution'
    while queue:
        queue.sort(key=path_cost)  # sort by cost
        path = queue.pop(0)
        node = path[-1][0]
        if node not in visited:
            adj_nodes = graph[node]
            ucs.num_node += len(adj_nodes)
            if len(queue) + len(adj_nodes) > ucs.max_fringe:
                ucs.max_fringe = len(queue) + len(adj_nodes)
            visited.append(node)
            if node == goal:
                return path
            else:
                for (node2, cost) in adj_nodes:
                    new_path = path.copy()
                    new_path.append((node2, cost))
                    queue.append(new_path)


path = []
newpath = []
pathCost = []
visited1 = []


def DFS(start, goal, graph, maxDepth, queue, j):
    # print("Checking for destination", start)
    queue.append(start)
    global Hg
    Hg += 0
    if start == goal:
        path.append(queue)
        newpath.append(pathCost)
        DFS.maxfringe = j
        # print("Maximum fringe size: ", j)
        return True
    if maxDepth <= 0:
        path.append(queue)
        newpath.append(pathCost)
        return False
    for node in graph[start]:
        pathCost.append(node[1])
        if len(graph[start]) + len(queue) + len(pathCost) > Hg:
            Hg = len(graph[start]) + len(queue) + len(pathCost)
        if node not in visited2:
            visited2.append(node)
            if len(graph[start]) + len(queue) + len(pathCost) > \
                Hg:
                Hg = len(graph[start]) + len(queue) + len(pathCost)
        if DFS(node[0], goal, graph, maxDepth - 1, queue, j):
            return True
        else:
            pathCost.pop()
            queue.pop()
    return False


def iterativeDDFS(start, goal, graph, maxDepth):
    for i in range(maxDepth):
        queue = []
        if DFS(start, goal, graph, i, queue, 0):
            return True
    return False
#------------------------------------------------------

#############The new BFS##############

import json
from queue import Queue

with open('/Users/mohammedalmousa/Downloads/Assignment2/AllReg.json', encoding='utf-8') as f:  # reading json file
    load1 = json.load(f)


class node:  # node is the city in the queue
    def __init__(self, cid, distance, parent=None):
        self.cid = cid
        self.distance = distance
        self.parent = parent  # The parent of current city

    def getCid(self):
        return self.cid

    def getDistance(self):
        return self.distance

    def __str__(self):  # Just for testing
        if self is not None:
            return f"[{self.cid}]: dicetance: [{self.distance}], parent: {self.parent}"  # if you print __str__ for current city it will print all the parents


def makeNode(cid, distance, parent):  # To make new city
    return node(cid, distance, parent)


def searchId(cityName):  # Take city name and then give you the cid of this cityNme, it take O(n)
    for city in load1:
        if city["name"] == cityName:
            return city["cid"]


def searchName(cid):  # Take city id and return city name, it take O(1)
    return load1[cid]["name"]


def goalFunction(goal, city):  # Take goal: The fiale city, and city: the current city
    return city == goal  # And retuen ture if the current city is the goal, false otherwise


visited = {}  # For every city in visited if it is true it mean, it is already visited, if not visited it will be false
for city in load1:
    visited[city["cid"]] = False

fringe = Queue()
maxFringeSize = fringe.qsize()


##



def expand(id, parent):  # Take id and parent of the current city then it finds the successors and puts them in the fringe
    listOfNeighbors = load1[id]["neighbors"]
    for i in listOfNeighbors:
        if not visited[i['cid']]:
            fringe.put(makeNode(i['cid'], i['distance'], parent))
            global nNodes
            nNodes += 1
    calcMaxFringeSize()


# BFS
def bfs():  # Return the final city as node
    while not fringe.empty():
        temp = fringe.get()
        visited[temp.getCid()] = True
        if goalFunction(goal_state_cid, temp.getCid()):
            return temp
        expand(temp.getCid(), temp)



x: str = input('Enter your start city:')
y: str = input('Enter your goal city:')
def calcMaxFringeSize():
    global maxFringeSize
    if maxFringeSize < fringe.qsize():
        maxFringeSize = fringe.qsize()



initial_state_name = x
initial_state_cid = searchId(initial_state_name)
visited[initial_state_name] = True  # At the start we will put the initial state in the finge
fringe.put(makeNode(initial_state_cid, 0, None))
global nNodes
nNodes = 1

goal_state_name = y
goal_state_cid = searchId(goal_state_name)

def print_path(p):
    print_path.path = []
    print_path.distance = 0
    while p is not None:
        print_path.path.append(searchName(p.getCid()))
        print_path.distance += p.getDistance()
        p = p.parent
    print_path.path.reverse()
    print("-----------------------------------------------------------------------------------------------")
    print("BFS: ")
    print(f"Route : {print_path.path}")
    print(f"Distance = {print_path.distance} km")



xr = bfs()
# testing
print_path(xr)
print(f"number of generated nodes = {nNodes}")
print(f"Maximum fringe size =  {maxFringeSize}")
print("UCS1: \n Route: ", ucs1(graph, x, y), "\n Distance: ", path_cost(ucs1(graph,  x, y)),
      "\n Number of generated nodes: ", ucs.num_node, "\n Maximum fringe size: ", ucs.max_fringe)
if iterativeDDFS( x, y,graph,100):
    c = len(newpath[0])
    s = 0
    for i in range(0, c):
        s += newpath[i][i]
    x1 = path.pop()
    print("IDS: \n Route: ", x1, "\n Distance: ", s, "\n Number of generated nodes: ", len(visited2),
          "\n Maximum fringe size: ", Hg)
else:
    print("IDS: \n Route: Maximum depth reached.")
    # , "\n Distance: ", 0, "\n Number of generated nodes: ", iterativeDDFS.genNodes,"\n Maximum fringe size: ", iterativeDDFS.maxQueue)


