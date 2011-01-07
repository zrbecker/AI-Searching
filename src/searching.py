
from collections import deque
from heapq import heappop, heappush
from Queue import *

def breadthFirstSearch(problem, graph=True):
    closed = set()
    fringe = deque()
    fringe.append( Node(None, None, problem.init_state, 0) )
    while len(fringe):
        node = fringe.popleft()
        if problem.goalTest(node.state):
            return node.solution()
        if node.state not in closed or not graph:
            if graph:
                closed.add(node.state)
            for (action, state) in problem.successorFunction(node.state):
                step_cost = problem.stepCost(node.state, action, state)
                fringe.append( Node(node, action, state, step_cost) )

def depthFirstSearch(problem, max_depth = None, graph=True):
    closed = set()
    fringe = deque()
    fringe.append( Node(None, None, problem.init_state, 0) )
    while len(fringe):
        node = fringe.pop()
        if problem.goalTest(node.state):
            return node.solution()
        if node.state not in closed or not graph:
            if graph:
                closed.add(node.state)
            if max_depth is None or node.depth <= max_depth:
                for (action, state) in problem.successorFunction(node.state):
                    step_cost = problem.stepCost(node.state, action, state)
                    fringe.append( Node(node, action, state, step_cost) )

def iteretiveDepthFirstSearch(problem, graph=True):
    n = 0
    while 1:
        solution = depthFirstSearch(problem, n, graph)
        if solution is None:
            n += 1
        else:
            return solution

def greedyBestFirstSearch(problem, graph = True):
    closed = set()
    fringe = []
    root = Node(None, None, problem.init_state, 0)
    heappush(fringe, (problem.greedyEval(root), root))
    while len(fringe):
        (value, node) = heappop(fringe)
        if problem.goalTest(node.state):
            return node.solution()
        if node.state not in closed or not graph:
            if graph:
                closed.add(node.state)
            for (action, state) in problem.successorFunction(node.state):
                step_cost = problem.stepCost(node.state, action, state)
                newNode = Node(node, action, state, step_cost)
                heappush(fringe, (problem.greedyEval(newNode), newNode))

def aStarSearch(problem, graph=True):
    closed = {}
    fringe = []
    root = Node(None, None, problem.init_state, 0)
    heappush(fringe, (problem.aStarEval(root), root))
    while len(fringe):
        (value, node) = heappop(fringe)
        if problem.goalTest(node.state):
            return node.solution()
        skip = False
        if graph and node.state in closed:
            skip = True
            if node.cost < closed[node.state]:
                skip = False
        if not graph or not skip:
            if graph:
                closed[node.state] = node.cost
            for (action, state) in problem.successorFunction(node.state):
                step_cost = problem.stepCost(node.state, action, state)
                newNode = Node(node, action, state, step_cost)
                heappush(fringe, (problem.aStarEval(newNode), newNode))

class Node:
    nodes_created = 0
    depth_found = 0

    def __init__(self, parent, action, state, step_cost):
        Node.nodes_created += 1
        
        self.parent = parent
        self.state = state
        self.action = action
        if parent:
            self.cost = parent.cost + step_cost
            self.depth = parent.depth + 1
        else:
            self.cost = step_cost
            self.depth = 0

    def solution(self):
        """
        Returns the cost of the solution, and a list
        of actions to execute the solution.
        """
        Node.depth_found = self.depth
        current_node = self
        actions = []
        while current_node.action != None:
            actions.append(current_node.action)
            current_node = current_node.parent
        actions.reverse()
        return Solution(current_node, actions, self.cost)

class Problem:
    """
    An empty problem
    """
    def __init__(self):
        self.init_state = None

    def successorFunction(self, state):
        return []

    def goalTest(self, state):
        return True

    def stepCost(self, prevState, action, newState):
        return 1

class Solution:
    """
    Contains init_state, a list of actions, and a cost
    """
    def __init__(self, init_state, actions, cost):
        self.init_state = init_state
        self.actions = actions
        self.cost = cost

def effectiveBranchingFactor(bf, d, n):
    """
    Calculates the effective branching factor of
    a heuristic.

    bf - Is the branching factor
    d - Is the depth of a solution
    n - Is the total number of nodes generated
    """
    return effectiveBranchingFactorHelper(1, bf, d, n)

def effectiveBranchingFactorHelper(beg, end, d, n):
    beg = float(beg)
    end = float(end)
    if (end - beg) / 2 < 0.01:
        return (beg + end) / 2

    nodes = summation(0, d, (beg + end) / 2)

    if nodes > n:
        return effectiveBranchingFactorHelper(beg, (beg + end) / 2, d, n)
    elif nodes < n:
        return effectiveBranchingFactorHelper((beg + end) / 2, end, d, n)
    else:
        return (beg + end) / 2

def summation(i, n, b):
    sum = 0
    while i <= n:
        sum += b ** i
        i += 1
    return sum