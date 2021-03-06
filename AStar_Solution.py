# Author: Travis Chamness
# Date: Sept 23, 2021
# Homework 2 for Intro to AI
import sys

#Goal state of puzzle
GOAL = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
ROW = 0
COL = 1
#Min dimension of an object in space
MIN_DIMENSION = 0
#Mex dimension of the puzzle of an n^2 shape
MAX_DIMENSION = 2
#Lookup Table allows for access to a specific tile's goal position
LOOKUP_TABLE = [[0,0], [0,1], [0,2], [1,0], [1,1], [1,2], [2,0], [2,1], [2,2]]
#Global value used to identify which algorithm is being used
SELECTION = 0
ULDR = [[-1,0],[0, -1],[1, 0],[0, 1]]

#Taken from Lab 3
class Node:
    def __init__(self, state, location, parent, movement):
        self.parent = parent
        self.location = location
        self.pastCost = 0 #g(n) in AStar strategy
        self.futureCost = 0 #h(n) in AStar strategy
        self.neighbors = [] # Neighbors from this current state of the graph
        self.state = state # Puzzle at this current state
        self.movement = movement
    #utility for comparing nodes
    def compare_state(self, state):
        same = True
        for i, row in enumerate(self.state):
            if same:
                for j,val in enumerate(row):
                    if val != state[i][j]:
                        same = False
                        break
            else:
                break
        return same

    #Tests current state against the GOAL state
    def goal_test(self):
        return self.compare_state(GOAL)

    def copy(self):
        newNode = Node(self.state, self.location, self.parent, self.movement)
        newNode.pastCost = self.pastCost
        newNode.futureCost = self.futureCost
        newNode.movement = self.movement
        return newNode
    # Compares two nodes for whether or not they are the same
    #   Does not consider the movement, because a movement may not have happened yet, and certainly should not happen again.'
    #   Also will not consider parent because the parent is irrelevant to the same state being found
    def compare(self, o):
        if o == None:
            return False
        elif o is self:
            return True
        else:
            # Comparing location short circuits the if statement so checking full state isn't always necessary
            return self.location == o.location and self.compare_state(o.state)

def not_in_closed(currentNode, closed):
    in_closed = False
    for node in closed:
        if in_closed:
            break
        else:
            in_closed = currentNode.compare(node)
    return in_closed

#Reads puzzle from file or user
def create_puzzle():
    #For user specified puzzle
    # maze_name = input("Enter puzzle name(Example - puzzle.txt): ")
    #For Hardcoded puzzle use
    # maze_name = "puzzle1.txt"
    # maze_name = "puzzle2.txt"
    maze_name = "puzzle3.txt"
    # maze_name = "puzzle4BFS.txt"

    file = open(maze_name, "r")
    lines = file.readlines()
    puzzle = []
    for line in lines:
        arr = []
        for character in line:
            if character != '\n':
                arr.append(int(character))
            else:
                break
        puzzle.append(arr)
    return puzzle

#prints the puzzle and identifies the starting position.
def print_puzzle_id_start(puzzle, find_start = False):
    for i, row in enumerate(puzzle): # Technical Row of the puzzle which is enumerated with identifier i
        print()
        for j, val in enumerate(row): # Technical Col of the puzzle which is enumerated with identifier j
            if val != '\n':
                if val  == 0:
                    print(" ", end=' ') #IDE says branch never entered?
                    start = [i,j] # Utilize the Row Column shape of the puzzle to describe the starting location with i,j
                else:
                    print(str(val), end=' ')
    print('\n')
    # Optionally return the start location, defaults as off
    if find_start:
        return start
    else:
        return ''

def manhattan_heuristics(node):
    global GOAL
    global LOOKUP_TABLE
    #Cumulative distance variable
    cumDist = 0
    #For every row of the GOAL Matrix
    for i,row in enumerate(GOAL):
        #For every value in the row
        for j,val in enumerate(row):
            #If the state tile does not match the goal state tile
            if val != node.state[i][j]:
                #Add to the cumulative distance the manhattan distance from the current position of the tile to where it should be.
                cumDist += abs(LOOKUP_TABLE[node.state[i][j]][ROW] - i) + abs(LOOKUP_TABLE[node.state[i][j]][COL] - j)
    return cumDist

def misplaced_tile_heuristics(node):
    global GOAL
    global LOOKUP_TABLE
    #Cumulative misplaced tile variable
    cumTiles = 0
    #For every row of the GOAL Matrix
    for i,row in enumerate(GOAL):
        #For every value in the row
        for j,val in enumerate(row):
            #If the state tile does not match the goal state tile
            if val != node.state[i][j]:
                #Increment the total misplaced tile count by 1
                cumTiles += 1
    return cumTiles

def successor_helper(currentNode, copyLocation, copyState, move, direction):
    #Swap spaces for state of new node
    temp = copyState[copyLocation[ROW] + move[ROW]][copyLocation[COL] + move[COL]]
    copyState[copyLocation[ROW] + move[ROW]][copyLocation[COL] + move[COL]] = copyState[copyLocation[ROW]][copyLocation[COL]]
    copyState[copyLocation[ROW]][copyLocation[COL]] = temp
    #Populate the new location
    copyLocation[ROW] = copyLocation[ROW] + move[ROW]
    copyLocation[COL] = copyLocation[COL] + move[COL]
    #Initialize new node
    newNode = Node(copyState, copyLocation, currentNode, direction)
    # Assign the new nodes past cost by incrementing the current node's cost by 1
    newNode.pastCost = currentNode.pastCost + 1
    # Assign the new nodes future cost by heuristic
    if SELECTION == 1: #Manhattan
        newNode.futureCost = manhattan_heuristics(newNode)
    else: #Misplaced tile
        newNode.futureCost = misplaced_tile_heuristics(newNode)
    currentNode.neighbors.append(newNode)
    return currentNode

def successor_func(currentNode):
    global SELECTION
    global ULDR

    for direction, move in enumerate(ULDR):
        copyLocation = currentNode.location.copy()
        copyState = [row[:] for row in currentNode.state]
        #Keeping the if statements separated. Prevent stepping into excessive if statements in the case that the copyLocation is out of bounds.
        if direction == 0:
            if copyLocation[ROW] != MIN_DIMENSION:
                currentNode = successor_helper(currentNode, copyLocation, copyState, move, 'U')
        elif direction == 1:
            if copyLocation[COL] != MIN_DIMENSION:
                currentNode = successor_helper(currentNode, copyLocation, copyState, move, 'L')
        elif direction == 2:
            if copyLocation[ROW] != MAX_DIMENSION:
                currentNode = successor_helper(currentNode, copyLocation, copyState, move, 'D')
        elif direction == 3:
            if copyLocation[COL] != MAX_DIMENSION:
                currentNode = successor_helper(currentNode, copyLocation, copyState, move, 'R')
    return currentNode

def populate_path(node):
    path = []
    currentNode = node.copy()
    while currentNode != None:
        path.insert(0, currentNode.movement)
        currentNode = currentNode.parent
    return path

def append_to_fringe(fringe, currentNode):
    for node in currentNode.neighbors:
        fringe.append(node.copy())
    return fringe

def lowest_cost_node(currentNode, fringe):
    smallestNode = Node(None,None,None,None)
    #Allows for any fringe node to replace the 'smallest node', regardless of the size of the fringe node upon first iteration
    smallestNode.pastCost = sys.maxsize
    indexToPop = None
    smallestNode.futureCost = 0
    for index, node in enumerate(fringe):
        # If the node off of the fringe has an f(n) = g(n)'pastcost' + h(n)'futurecost' less than current smallestNode, replace smallestNode
        if (node.pastCost + node.futureCost) < (smallestNode.pastCost + smallestNode.futureCost):
            smallestNode = node
            indexToPop = index

    if smallestNode.pastCost == sys.maxsize:
        return currentNode
    else:
        fringe.pop(indexToPop)
        return smallestNode


def a_star_solution(puzzle):
    global SELECTION

    #Initial Structures
    find_start = True
    goalFound = False
    start = print_puzzle_id_start(puzzle, find_start)
    path = []
    closed = []
    currentNode = None
    head = Node(puzzle, start, None, None)

    #Future cost of starting position determined by manhattan heuristics
    if SELECTION == 1:
        head.futureCost = manhattan_heuristics(head)
    else:
        head.futureCost = misplaced_tile_heuristics(head)

    #Process puzzle via A Star
    fringe = [head]
    while not goalFound and fringe:
        in_closed = False
        currentNode = lowest_cost_node(currentNode, fringe)
        #Test the current node state against the goal state
        goalFound = currentNode.goal_test()
        print(print_puzzle_id_start(currentNode.state))
        #Determine if the current node is in the closed set
        in_closed = not_in_closed(currentNode, closed)
        if not goalFound and not in_closed:
            closed.append(currentNode)
            currentNode = successor_func(currentNode)
            fringe = append_to_fringe(fringe, currentNode)
        elif goalFound:
            path = populate_path(currentNode)
    
    return path, currentNode.pastCost

#Solution host provides the user with oportunity to use manhattan or misplaced tile heuristics
def solution_host():
    global SELECTION

    validSelection = False
    while not validSelection:
        SELECTION = int(input("Heuristics Menu: \n1 - Manhattan Distance \n2 - Misplaced Tiles \n(Enter 1 or 2 for your selection) \nYour Choice?: "))
        print()
        if SELECTION == 1:
            validSelection = True
            print("Manhattan Selected")
            path, cost = a_star_solution(create_puzzle())
        elif SELECTION == 2:
            validSelection = True
            print("Misplaced Tiles Selected")
            path, cost = a_star_solution(create_puzzle())
            
        else:
            print("Invalid Selection\n")
    #Output results from the search
    print("Path: {} \nCost: {}".format(path, cost))


solution_host()
