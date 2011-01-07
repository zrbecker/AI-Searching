#!/usr/bin/python -tt
import sys
import random
from random import randint
from searching import *

class VacuumWorld(Problem):
    """
    Vacuum World

    The world is a grid of some width and height (specified)

    Each location on the grid can have dirt. The goal of the
    problem is to remove all the dirt formt he grid.

    An agent cleaning the grid can move left right up and down and
    it can suck up the dirt.
    """
    LEFT  = 'Left'
    RIGHT = 'Right'
    UP    = 'Up'
    DOWN  = 'Down'
    SUCK  = 'Suck'
    
    actions = [LEFT, RIGHT, UP, DOWN, SUCK]
    
    def __init__(self, width, height):
        """ Initializes the problem with a random state """
        self.init_state = self.randomState(width, height)

    def successorFunction(self, state):
        """ Finds all the possible actions and resulting states
            for a paticular state. """
        return [(action, self.doAction(action, state))
                   for action in self.actions]

    def goalTest(self, state):
        """ Tests if a state is a goal state (All spaces clean) """
        return state[4] == 0
    
    def randomState(self, width, height):
        """ Creates a random state """
        x = randint(0, width - 1)
        y = randint(0, height - 1)
        grid = randint(0, 2 ** (width * height) - 1)
        return (x, y, width, height, grid)
    
    def doAction(self, action, state):
        """ Finds the resulting state from performing an action
            on another state. """
        (x, y, w, h, g) = state
        if   action == self.LEFT  and x > 0:     x -= 1
        elif action == self.RIGHT and x < w - 1: x += 1
        elif action == self.UP    and y > 0:     y -= 1
        elif action == self.DOWN  and y < h - 1: y += 1
        elif action == self.SUCK:
            g &= (2 ** (w * h) - 1) ^ 2 ** (y * w + x)
        return (x, y, w, h, g)

    def heuristic1(self, node):
        (x, y, w, h, g) = node.state
        value = 0
        if g & 2 ** (y * h + x): value -= 1
        while g > 0:
            if g % 2: value += 2
            g /= 2
        return value
    
    def heuristic2(self, node):
        (x, y, w, h, g) = node.state
        h1 = 0
        h2 = 0
        if g & 2 ** (y * h + x): h1 -= 1
        for i in range(w):
            for j in range(h):
                if g & 2 ** (j * h + i):
                    h1 += 2
                    d = abs(x - i) + abs(y - j) + 1
                    if h2 < d: h2 = d
        return max(h1, h2)

    
    def aStarEval(self, node):
        """ Estimates the value of a given state. """
        #print node.cost, self.heuristic(node)
        return node.cost + self.heuristic1(node)

    def greedyEval(self, node):
        """ Estimates the value of a given state. """
        return self.heuristic1(node)

def printState(state):
    (px, py, w, h, g) = state
    result = ''
    for y in range(h):
        for x in range(w):
            if px == x and py == y: result += '('
            else:                   result += ' '
            if g % 2: result += '*'
            else:     result += ' '
            g /= 2
            if px == x and py == y: result += ')'
            else:                   result += ' '
        result += '\n'
    print result


def usage():
    print 'Usage:', sys.argv[0],
    print '[--iter | --breadth | --depth | --greedy | --astar] [--max number]',
    print '[--nograph] [--seed] width height'
    print
    print 'All flags have short version -x where x is their first letter.'

def main():
    # Get arguments from console if they are there.
    try:
        type = ''
        max = None
        graph = True
        seed = None
        
        args = sys.argv[1:]
        if args[0] in ['--iter', '-i', '--breadth', '-b',
                       '--depth', '-b', '--greedy', '-g',
                       '--astar', '-a']:
            if   args[0] == '-i': type = 'iter'
            elif args[0] == '-b': type = 'breadth'
            elif args[0] == '-d': type = 'depth'
            elif args[0] == '-g': type = 'greedy'
            elif args[0] == '-a': type = 'astar'
            else:                 type = args[0][2:]
            del args[0]
        if args[0] in ['--max', '-m']:
            max = int(args[1])
            del args[0:2]
        if args[0] in ['--nograph', '-n']:
            graph = False
            del args[0]
        if args[0] in ['--seed', '-s']:
            seed = int(args[1])
            del args[0:2]
        width = int(args[0])
        height = int(args[1])
    except:
        usage()
        sys.exit()

    if seed is not None:
        random.seed(seed)

    # Create problem and find solution
    problem = VacuumWorld(width, height)

    current_state = problem.init_state
    print 'Initial State:'
    printState(current_state)
    
    if type == 'depth':
        print 'Doing depth first search'
        Node.nodes_created = 0
        solution = depthFirstSearch(problem, max, graph)
        eb_factor = effectiveBranchingFactor(5, Node.depth_found, Node.nodes_created)
        print 'Depth found:', Node.depth_found
        print 'Nodes created:', Node.nodes_created
        print 'Branching factor:', 5
        print 'Effective branching factor:', eb_factor
    elif type == 'breadth':
        print 'Doing breadth first search'
        Node.nodes_created = 0
        solution = breadthFirstSearch(problem, graph)
        eb_factor = effectiveBranchingFactor(5, Node.depth_found, Node.nodes_created)
        print 'Depth found:', Node.depth_found
        print 'Nodes created:', Node.nodes_created
        print 'Branching factor:', 5
        print 'Effective branching factor:', eb_factor
    elif type == 'iter':
        print 'Doing iteretive search'
        Node.nodes_created = 0
        solution = iteretiveDepthFirstSearch(problem, graph)
        eb_factor = effectiveBranchingFactor(5, Node.depth_found, Node.nodes_created)
        print 'Depth found:', Node.depth_found
        print 'Nodes created:', Node.nodes_created
        print 'Branching factor:', 5
        print 'Effective branching factor:', eb_factor
    elif type == 'greedy':
        print 'Doing greedy best first search'
        Node.nodes_created = 0
        solution = greedyBestFirstSearch(problem, graph)
        eb_factor = effectiveBranchingFactor(5, Node.depth_found, Node.nodes_created)
        print 'Depth found:', Node.depth_found
        print 'Nodes created:', Node.nodes_created
        print 'Branching factor:', 5
        print 'Effective branching factor:', eb_factor
    elif type == 'astar':
        print 'Doing A* search'
        Node.nodes_created = 0
        solution = aStarSearch(problem, graph)
        eb_factor = effectiveBranchingFactor(5, Node.depth_found, Node.nodes_created)
        print 'Depth found:', Node.depth_found
        print 'Nodes created:', Node.nodes_created
        print 'Branching factor:', 5
        print 'Effective branching factor:', eb_factor
    print
    
    # Print solution
    if solution is None:
        print 'There was no solution found.'
    elif len(solution.actions) == 0:
        print 'The task is already solved!'
    else:
        print 'Solution Cost:', solution.cost
        for action in solution.actions:
            print action,

if __name__ == '__main__':
    main()
