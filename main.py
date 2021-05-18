## Uninformed and informed searches 
## Mustafa Sadiq

################################# Imports ###############################
import random as rand
import numpy as np
from collections import deque
import heapq
from math import sqrt

####################### Make maze ##############################
def make_maze(dim, p):
    maze = np.empty([dim, dim], dtype = str)
    for x in range(dim):
        for y in range(dim):
            if(rand.random() <= p):
                maze[x][y] = 'X'
            else:
                maze[x][y] = ' ' 
    maze[0][0] = 'S'
    maze[dim - 1][dim - 1] = 'G'
    return maze

################### Print ascii maze ##############################
def print_maze(maze):
    print('-'*(len(maze)*4), end="")
    print()
    for x in maze:
        for y in x:
            print(f'| {y} ', end="")
        print("|", end="")
        print()
        print('-'*(len(maze)*4), end="")
        print()

#################### Euclidean distance #######################################
def distance(start, end):
    return sqrt((start[0]-end[0])**2+(start[1]-end[1])**2)


################# Take n steps on maze with given path #################
def take_n_steps(maze, path, steps=None):
    if steps == None:
        for point in path:
            maze[point[0]][point[1]] = '*'
    elif steps == -1:
        for point in path:
            maze[point[0]][point[1]] = ' '
    else:
        for point in path[:steps]:
            maze[point[0]][point[1]] = '*'

##################### Get neighbours of a given point in a maze ####################
def get_neighbours(maze, current):
    x = current[0]
    y = current[1]
    dim = len(maze)
    neighbours = []

    if x-1 >= 0 and maze[x-1][y] != 'X': 
        neighbours.append((x-1, y))
    if y-1 >= 0 and maze[x][y-1] != 'X': 
        neighbours.append((x, y-1))
    if y+1 <= dim - 1 and maze[x][y+1] != 'X': 
        neighbours.append((x, y+1))
    if x+1 <= dim - 1 and maze[x+1][y] != 'X': 
        neighbours.append((x+1, y))   
    
    return neighbours

def get_nonfire_neighbours(maze, current):
    x = current[0]
    y = current[1]
    dim = len(maze)
    neighbours = []

    if x-1 >= 0 and maze[x-1][y] != 'X' and maze[x-1][y] != 'F': 
        neighbours.append((x-1, y))
    if y-1 >= 0 and maze[x][y-1] != 'X' and maze[x][y-1] != 'F':
        neighbours.append((x, y-1))
    if y+1 <= dim - 1 and maze[x][y+1] != 'X' and maze[x][y+1] != 'F':
        neighbours.append((x, y+1))
    if x+1 <= dim - 1 and maze[x+1][y] != 'X' and maze[x+1][y] != 'F':
        neighbours.append((x+1, y))   
    
    return neighbours


###################### DFS = Find a path given a maze, start, goal ###########################
def dfs(maze, start, goal):
    fringe = [start] 
    tree = dict()
    tree[start] = None

    while  fringe:
        current = fringe.pop()
        if current == goal:
            current = goal
            path = []
            while current != start:
                path.append(current)
                current = tree[current]
            path.append(start)
            path.reverse()
            return path
        for neighbour in get_nonfire_neighbours(maze, current):
            if neighbour not in tree:
                fringe.append(neighbour)
                tree[neighbour] = current

    return None

###################### BFS = Find a path given a maze, start, goal ###########################
def bfs(maze, start, goal):
    fringe = deque() 
    fringe.append(start)
    tree = dict()
    tree[start] = None

    while  fringe:
        current = fringe.popleft()
        if current == goal:
            current = goal
            path = []
            while current != start:
                path.append(current)
                current = tree[current]
            path.append(start)
            path.reverse()
            return path
        for neighbour in get_nonfire_neighbours(maze, current):
            if neighbour not in tree:
                fringe.append(neighbour)
                tree[neighbour] = current

    return None


###################### A* = Find a path given a maze, start, goal ###########################
def astar(maze, start, goal):
    fringe = [] 
    heapq.heappush(fringe, (0,start))
    tree = dict()
    cost_tree = dict()
    tree[start] = None
    cost_tree[start] = 0

    while  fringe:
        current = heapq.heappop(fringe)[1]
        if current == goal:
            current = goal
            path = []
            while current != start:
                path.append(current)
                current = tree[current]
            path.append(start)
            path.reverse()
            return path
        for neighbour in get_nonfire_neighbours(maze, current):
            new_cost = cost_tree[current] + 1
            if neighbour not in cost_tree or new_cost < cost_tree[neighbour]:
                cost_tree[neighbour] = new_cost
                priority = new_cost + distance(goal, neighbour)
                heapq.heappush(fringe, (priority, neighbour))
                tree[neighbour] = current

    return None


##################################################

## Make maze
dim = 10
p = 0.2
maze = make_maze(dim,p)
print("\n\nOriginal maze: ")
print_maze(maze)

### DFS
path = dfs(maze, (0,0), (dim-1,dim-1))
take_n_steps(maze, path)
print("\n\nDepth first search: ")
print_maze(maze)

## BFS
take_n_steps(maze, path, -1)
path = bfs(maze, (0,0), (dim-1,dim-1))
take_n_steps(maze, path)
print("\n\nBreadth first search:")
print_maze(maze)

## A*
take_n_steps(maze, path, -1)
path = astar(maze, (0,0), (dim-1,dim-1))
take_n_steps(maze, path)
print("\n\nA* search:")
print_maze(maze)