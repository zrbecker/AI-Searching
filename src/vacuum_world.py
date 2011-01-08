#!/usr/bin/python -tt

import sys
import getopt
import random

from searching import *


__all__ = ['VacuumWorld', 'VacuumWorldAgent', 'VacuumWorldState']


class VacuumWorld(object):
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

    def successors(self, state):
        """
        Finds all the possible actions and resulting states
        for a paticular state.
        
        """
        result = []
        for action in (self.LEFT, self.RIGHT, self.UP, self.DOWN, self.SUCK):
            newstate = self.doaction(action,state)
            result.append((action, newstate))
        return result

    def isgoal(self, state):
        """ Tests if a state is a goal state (All spaces clean) """
        return state.grid == 0
    
    def randstate(self, width, height):
        """ Creates a random state """
        x = random.randint(0, width - 1)
        y = random.randint(0, height - 1)
        grid = random.randint(0, 1 << (width * height) - 1)
        return VacuumWorldState(x, y, width, height, grid)
    
    def doaction(self, action, state):
        """
        Finds the resulting state from performing an action
        on another state.
        
        """
        x, y = state.x, state.y
        w, h = state.w, state.h
        grid = state.grid
        if action == self.LEFT and x > 0:
            x -= 1
        elif action == self.RIGHT and x < w - 1:
            x += 1
        elif action == self.UP and y > 0:
            y -= 1
        elif action == self.DOWN and y < h - 1:
            y += 1
        elif action == self.SUCK:
            grid &= ((1 << (w * h)) - 1 ^ 1 << (y * w + x))
        return VacuumWorldState(x, y, w, h, grid)

    def stepcost(self, prev, action, new):
        """ Step cost is equal to depth """
        return 1

    def __init__(self, width, height):
        """ Initializes the problem with a random state """
        self.init_state = self.randstate(width, height)


class VacuumWorldAgent(Agent):
    """
    Specialized agent for dealing with the vacuum world problem.

    """
    def heuristic(self, node):
        """
        Evaluates the number of tiles on the board and the
        number of actions it would take to clean them up
        if they were all in a row.

        """
        state = node.state
        value = 0
        if state.grid & 1 << (state.y * state.w + state.x):
            value -= 1
        g = state.grid
        while g > 0:
            if g % 2:
                value += 2
            g >>= 1
        return value

    def astar_eval(self, node):
        """ Estimates the value of a given state. """
        return node.cost + self.heuristic(node)

    def greedy_eval(self, node):
        """ Estimates the value of a given state. """
        return self.heuristic(node)


class VacuumWorldState(object):
    """
    Encapsulates the members of a vacuum world state.

    """
    def copy(self):
        return VacuumWorldState(self.x, self.y, self.w, self.h, self.grid)

    def __init__(self, x, y, w, h, grid):
        super(VacuumWorldState, self).__setattr__('x', x)
        super(VacuumWorldState, self).__setattr__('y', y)
        super(VacuumWorldState, self).__setattr__('w', w)
        super(VacuumWorldState, self).__setattr__('h', h)
        super(VacuumWorldState, self).__setattr__('grid', grid)

    def __eq__(self, other):
        return all((self.x == other.x,
                    self.y == other.y,
                    self.w == other.w,
                    self.h == other.h,
                    self.grid == other.grid))

    def __hash__(self):
        return hash((self.x, self.y, self.w, self.h, self.grid))

    def __setattr__(self, *args):
        raise TypeError('Can\'t modify immutable instance')

    def __str__(self):
        result = ''
        for y in range(self.h):
            for x in range(self.w):
                if self.x == x and self.y == y:
                    result += '('
                else:
                    result += ' '
                if self.grid & (1 << (y * self.w + x)):
                    result += '*'
                else:
                    result += ' '
                if self.x == x and self.y == y:
                    result += ')'
                else:
                    result += ' '
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
        width = int(args[0])
        height = int(args[1])
    except:
        usage()
        sys.exit(1)
    return (type, max, graph, width, height)
        

def main():
    (type, max, graph, width, height) = handleargs()
    problem = VacuumWorld(width, height)
    agent = VacuumWorldAgent(problem, graph)
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
    print 'Nodes created:', Node.nodes_created
    print 'Branching factor:', 5
    print 'Effective branching factor:',
    print Node.nodes_created ** (1.0 / solution.depth)
    print solution


if __name__ == '__main__':
    main()
