#!/usr/bin/python -tt

import sys
import getopt
import random

from searching import *


__all__ = ['NPuzzle', 'NPuzzleAgent', 'VacuumWorldState']


class NPuzzle(object):
    """
    N Puzzle

    """
    LEFT  = 'Left'
    RIGHT = 'Right'
    UP    = 'Up'
    DOWN  = 'Down'

    def successors(self, state):
        """
        Finds all the possible actions and resulting states
        for a paticular state.

        """
        result = []
        for action in (self.LEFT, self.RIGHT, self.UP, self.DOWN):
            newstate = self.doaction(action,state)
            result.append((action, newstate))
        return result

    def isgoal(self, state):
        """ Tests if a state is a goal state (All spaces clean) """
        return state.grid == sorted(state.grid)

    def randstate(self, n):
        """ Creates a random state """
        grid = range(n * n)
        random.shuffle(grid)
        return NPuzzleState(n, grid)

    def doaction(self, action, state):
        """
        Finds the resulting state from performing an action
        on another state.

        """
        n = state.n
        grid = state.grid[:]
        pos = grid.index(0)
        x = pos % n
        y = pos // n
        if action == self.LEFT and x > 0:
            grid[pos] = grid[pos - 1]
            grid[pos - 1] = 0
        elif action == self.RIGHT and x < n - 1:
            grid[pos] = grid[pos + 1]
            grid[pos + 1] = 0
        elif action == self.UP and y > 0:
            grid[pos] = grid[pos - n]
            grid[pos - n] = 0
        elif action == self.DOWN and y < n - 1:
            grid[pos] = grid[pos + n]
            grid[pos + n] = 0
        return NPuzzleState(n, grid)

    def stepcost(self, prev, action, new):
        """ Step cost is equal to depth """
        return 1

    def __init__(self, n):
        """ Initializes the problem with a random state """
        self.init_state = self.randstate(n)


class NPuzzleAgent(Agent):
    """
    Specialized agent for dealing with the N Puzzle problem.

    """
    def tiles_misplaced(self, node):
        """ Counts the tiles out of place """
        value = 0
        for index, number in enumerate(node.state.grid):
            if index != number:
                value += 1
        return value

    def tiles_distance(self, node):
        value = 0
        for index, number in enumerate(node.state.grid):
            x1 = index % node.state.n
            y1 = index // node.state.n
            x2 = number % node.state.n
            y2 = number // node.state.n
            value += abs(x2 - x1) + abs(y2 - y1)
        return value

    def astar_eval(self, node):
        """ Estimates the value of a given state. """
        return node.cost + self.tiles_distance(node)

    def greedy_eval(self, node):
        """ Estimates the value of a given state. """
        return  self.tiles_distance(node)


class NPuzzleState(object):
    """
    Encapsulates the members of a vacuum world state.

    """
    def copy(self):
        return NPuzzleState(self.n, self.grid)

    def __init__(self, n, grid):
        super(NPuzzleState, self).__setattr__('n', n)
        super(NPuzzleState, self).__setattr__('grid', grid)

    def __eq__(self, other):
        return all((self.n == other.n,
                    self.grid == other.grid))

    def __hash__(self):
        return hash(self.n)

    def __setattr__(self, *args):
        raise TypeError('Can\'t modify immutable instance')

    def __str__(self):
        result = ''
        for y in range(self.n):
            for x in range(self.n):
                result += '%4d' % (self.grid[y * self.n + x])
            result += '\n'
        return result


def usage(details=False):
    if details:
        print 'Usage:', sys.argv[0], '[-ibdga] [-m n] [-n] [-s n] width height'
        print
        print '   -i --iter     Use iterative search'
        print '   -b --breadth  Use breadth first search'
        print '   -d --depth    Use depth first search'
        print '   -g --greedy   Use greedy search'
        print '   -a --astar    Use A* search'
        print
        print '   -m --max n    Set maximum depth for depth first search'
        print '   -n --nograph  Use a tree instead of graph for state space'
        print '   -s --seed n   Seed the random number generator'
        print '   -h --help     Print this message'
        pass
    else:
        print 'Usage:', sys.argv[0], '[-ibdga] [-m n] [-n] [-s n] width height'


def handleargs():
    type = 'astar'
    max = None
    graph = True
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'ibdgam:ns:h', ['iter',
            'breadth', 'depth', 'greedy', 'astar', 'max=', 'nograph',
            'help', 'seed='])
    except getopt.GetoptError, err:
        print str(err)
        usage()
        sys.exit(1)
    for o, a in opts:
        if o in ('-i', '--iter'):
            type = 'iter'
        elif o in ('-b', '--breadth'):
            type = 'breadth'
        elif o in ('-d', '--depth'):
            type = 'depth'
        elif o in ('-g', '--greedy'):
            type = 'greedy'
        elif o in ('-a', '--astar'):
            type = 'astar'
        elif o in ('-m', '--max'):
            max = a
        elif o in ('-n', '--nograph'):
            graph = False
        elif o in ('-s', '--seed'):
            random.seed(a)
        elif o in ('-h', '--help'):
            usage(True)
            sys.exit(0)
    try:
        n = int(args[0])
    except:
        usage()
        sys.exit(1)
    return (type, max, graph, n)


def main():
    (type, max, graph, n) = handleargs()
    problem = NPuzzle(n)
    agent = NPuzzleAgent(problem, graph)
    print 'Initial State:'
    print problem.init_state
    Node.nodes_created = 0
    if type == 'breadth':
        print 'Doing breadth first search'
        solution = agent.breadth_search()
    elif type == 'depth':
        print 'Doing depth first search'
        solution = agent.depth_search(max)
    elif type == 'iter':
        print 'Doing iteretive search'
        solution = agent.iterative_search()
    elif type == 'greedy':
        print 'Doing greedy search'
        solution = agent.greedy_search()
    elif type == 'astar':
        print 'Doing A* search'
        solution = agent.astar_search()
    print
    if solution:
        print 'Nodes created:', Node.nodes_created
        print 'Branching factor:', 5
        print 'Effective branching factor:',
        if solution.depth != 0:
            print Node.nodes_created ** (1.0 / solution.depth)
        else:
            print '?'
        print solution
    else:
        print 'No soltuion found.'


if __name__ == '__main__':
    main()
