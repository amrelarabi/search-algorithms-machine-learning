# #!/usr/bin/python
#
### Student Info
# Smith, Christopher, 02386569, 159.302
# Assignment 1: 8 Puzzle.
#
### Language
# This assignment was written in Python. An open source, interpreted language
# with a mix of imperative, OO and functional programming. Syntax is simple
# and easy to learn.
#
# Developed on Ubuntu Linux but this will run on the interpreter available
# from http://python.org. Documentation is also on that site but a good
# tutorial is available for free at http://diveintopython.org.
#
### Data Structures
#
# The state of the board is stored in a list. The list stores values for the
# board in the following positions:
#
# -------------
# | 0 | 3 | 6 |
# -------------
# | 1 | 4 | 7 |
# -------------
# | 2 | 5 | 8 |
# -------------
#
# The goal is defined as:
#
# -------------
# | 1 | 2 | 3 |
# -------------
# | 8 | 0 | 4 |
# -------------
# | 7 | 6 | 5 |
# -------------
#
# Where 0 denotes the blank tile or space.
#
# The code will read state from a file called "state.txt" where the format is
# as above but space seperated. i.e. the content for the goal state would be
# 1 8 7 2 0 6 3 4 5

### Code begins.
import sys

goal_state = [1, 8, 7, 2, 0, 6, 3, 4, 5]

def display_board(state):
    print( "-------------")
    print( "| %i | %i | %i |" % (state[0], state[3], state[6]))
    print( "-------------")
    print( "| %i | %i | %i |" % (state[1], state[4], state[7]))
    print( "-------------")
    print( "| %i | %i | %i |" % (state[2], state[5], state[8]))
    print( "-------------")


def move_up(state):
    ## [1,2,3,4,0,5,6,7,8]
    """Moves the blank tile up on the board. Returns a new state as a list."""
    # Perform an object copy
    new_state = state[:]
    index = new_state.index(0)
    # Sanity check
    if index not in [0, 3, 6]:
        # Swap the values.
        temp = new_state[index - 1]
        new_state[index - 1] = new_state[index]
        new_state[index] = temp
        return new_state
    else:
        # Can't move, return None (Pythons NULL)
        return None

def move_down(state):
    """Moves the blank tile down on the board. Returns a new state as a list."""
    # Perform object copy
    new_state = state[:]
    index = new_state.index(0)
    # Sanity check
    if index not in [2, 5, 8]:
        # Swap the values.
        temp = new_state[index + 1]
        new_state[index + 1] = new_state[index]
        new_state[index] = temp
        return new_state
    else:
        # Can't move, return None.
        return None


def move_left(state):
    """Moves the blank tile left on the board. Returns a new state as a list."""
    new_state = state[:]
    index = new_state.index(0)
    # Sanity check
    if index not in [0, 1, 2]:
        # Swap the values.
        temp = new_state[index - 3]
        new_state[index - 3] = new_state[index]
        new_state[index] = temp
        return new_state
    else:
        # Can't move it, return None
        return None


def move_right(state):
    """Moves the blank tile right on the board. Returns a new state as a list."""
    # Performs an object copy. Python passes by reference.
    new_state = state[:]
    index = new_state.index(0)
    # Sanity check
    if index not in [6, 7, 8]:
        # Swap the values.
        temp = new_state[index + 3]
        new_state[index + 3] = new_state[index]
        new_state[index] = temp
        return new_state
    else:
        # Can't move, return None
        return None


def create_node(state, parent, operator, depth, cost):
    return Node(state, parent, operator, depth, cost)


def expand_node(node):
    """Returns a list of expanded nodes"""
    expanded_nodes = []
    expanded_nodes.append(create_node(move_up(node.state), node, "u", node.depth + 1, 0))
    expanded_nodes.append(create_node(move_down(node.state), node, "d", node.depth + 1, 0))
    expanded_nodes.append(create_node(move_left(node.state), node, "l", node.depth + 1, 0))
    expanded_nodes.append(create_node(move_right(node.state), node, "r", node.depth + 1, 0))
    # Filter the list and remove the nodes that are impossible (move function returned None)
    expanded_nodes = [node for node in expanded_nodes if node.state != None]  # list comprehension!
    return expanded_nodes

def bfs(start, goal):
    """Performs a breadth first search from the start state to the goal"""
    # A list (can act as a queue) for the nodes.

    """

            A
          /    \
        B       C
       / \     / \
      D   E   F   G

    BFS sequence A -> B -> C -> D -> E -> F -> G
    Q used to represent fringe
    Algorithm:
    if current state not goal state
        expand all possible states of current state
        add expanded states to fringe queue
    else
        revisit all the previous parents to the root and collect operation type of each
        stop
    """
    start_node = create_node(start, None, None, 1, 0)
    fringe = []
    fringe.append(start_node)
    while True:
        current_node = fringe.pop(0)
        #display_board(current_node.state)

        if current_node.state != goal:
            expanded_nodes = expand_node(current_node)
            # merge new extended nodes to fringe
            fringe.extend(expanded_nodes)
        else:
            operators = [current_node.operator]
            while current_node.parent:
                current_node = current_node.parent
                if current_node.operator:
                    operators.insert(0, current_node.operator)
            return operators


def dfs(start, goal, depth=10):
    """Performs a depth first search from the start state to the goal. Depth param is optional."""
    # NOTE: This is a limited search or else it keeps repeating moves. This is an infinite search space.
    # I'm not sure if I implemented this right, but I implemented an iterative depth search below
    # too that uses this function and it works fine. Using this function itself will repeat moves until
    # the depth_limit is reached. Iterative depth search solves this problem, though.
    #
    # An attempt of cutting down on repeat moves was made in the expand_node() function.

    """

            A
          /    \
        B       C
       / \     / \
      D   E   F   G

    DFS sequence A -> B -> D -> E -> C -> F -> G

    """
    start_node = create_node(start, None, None, 1, 0)
    fringe = []
    fringe.append(start_node)
    while True:
        current_node = fringe.pop(0)
        if current_node.state != goal:
            expanded_nodes = expand_node(current_node)
            # merge new extended nodes to fringe
            # max depth
            if current_node.depth <= depth:
                fringe = expanded_nodes + fringe
        else:
            operators = [current_node.operator]
            while current_node.parent:
                current_node = current_node.parent
                if current_node.operator:
                    operators.insert(0, current_node.operator)
            return operators


def ids( start, goal, depth=10 ):
	"""Perfoms an iterative depth first search from the start state to the goal. Depth is optional."""
	for i in range( depth ):
		result = dfs( start, goal, i )
		if result != None:
			return result

def a_star(start, goal):
    """Perfoms an A* heuristic search"""
    # ATTEMPTED: does not work :(
    nodes_arr = [create_node(start, None, None, 0, 0)]
    while True:
        arr = []

        for index in range(len(nodes_arr)):
            arr.append(cmp(nodes_arr[index], goal))

        min_index = arr.index(min(arr))
        this_node = nodes_arr.pop(min_index)
        if this_node.state == goal:
            ops = [this_node.operator]
            while this_node.parent != None:
                this_node = this_node.parent
                if this_node.operator != None:
                    ops.insert(0, this_node.operator)
            break
        else:
            nodes_arr.extend(expand_node(this_node))
    return ops

def greedy(start, goal):
    fringe = [create_node(start, None, None, 0, 0)]
    while True:
        heuristic = [h(fringe[i].state, goal) for i in range(len(fringe))]
        heuristicMin = min(heuristic)
        current_node = fringe[heuristic.index(heuristicMin)]
        #print(current_node.state, goal, heuristic, heuristicMin)
        if current_node.state == goal:
            operators = [current_node.operator]
            while current_node.parent:
                current_node = current_node.parent
                if current_node.operator:
                    operators.insert(0, current_node.operator)
            break
        else:
            fringe.pop(fringe.index(current_node))
            fringe.extend(expand_node(current_node))

    return operators

def ucs(start, goal):
    fringe = [create_node(start, None, None, 0, 0)]
    while True:
        cmp_arr = []
        for index in range(len(fringe)):
            cmp_arr.append(cmp_cost(fringe[index]))
        # pop list item with min g(n) + h(n)
        current_index = cmp_arr.index(min(cmp_arr))
        current_node = fringe.pop(current_index)
        #display_board(current.state)
        if current_node.state == goal:
            operators = [current_node.operator]
            while current_node.parent:
                current_node = current_node.parent
                if current_node.operator:
                    operators.insert(0, current_node.operator)
            break
        else:
            fringe.extend(expand_node(current_node))
    return operators


def cmp(x, y):
    # Compare function for A*. f(n) = g(n) + h(n). I use depth (number of moves) for g().
    cost = 0
    current_node = x
    while True:
        if current_node:
            cost += x.depth
            current_node = current_node.parent
        else:
            return cost + h(x.state, y)


def cmp_cost(x):
    # Compare function for UCS.
    cost = 0
    current_node = x
    while True:
        if current_node:
            cost += x.depth
            current_node = current_node.parent
        else:
            return cost

def h(state, goal):
    """Heuristic for the A* search. Returns an integer based on out of place tiles"""
    score = 0
    for i in range(len(state)):
        if state[i] != goal[i]:
            score = score + 1
    return score


# Node data structure
class Node:
    def __init__(self, state, parent, operator, depth, cost):
        # Contains the state of the node
        self.state = state
        # Contains the node that generated this node
        self.parent = parent
        # Contains the operation that generated this node from the parent
        self.operator = operator
        # Contains the depth of this node (parent.depth +1)
        self.depth = depth
        # Contains the path cost of this node from depth 0. Not used for depth/breadth first.
        self.cost = cost

def readfile(filename):
    f = open(filename)
    data = f.read()
    # Get rid of the newlines
    data = data.strip("\n")
    # Break the string into a list using a space as a seperator.
    data = data.split(" ")
    state = []
    for element in data:
        state.append(int(element))
    return state


# Main method
def main():
    starting_state = readfile("state.txt")
    ### CHANGE THIS FUNCTION TO USE bfs, dfs, ids or a_star
    result = bfs(starting_state, goal_state)
    if result == None:
        print( "No solution found")
    elif result == [None]:
        print( "Start node was the goal!")
    else:
        print( result)
        print( len(result), " moves")


# A python-isim. Basically if the file is being run execute the main() function.
if __name__ == "__main__":
    main()
