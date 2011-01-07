from collections import deque
from heapq import heappop, heappush


__all__ = ['Agent', 'Node', 'Solution', 'find_eb_factor']


class Agent(object):
    def breadth_search(self, problem, graph=True):
        closed = set()
        fringe = deque()
        fringe.append( Node(None, None, problem.init_state, 0) )
        while len(fringe):
            node = fringe.popleft()
            if problem.isgoal(node.state):
                return node.solution()
            if node.state not in closed or not graph:
                if graph:
                    closed.add(node.state)
                for (action, state) in problem.successors(node.state):
                    step_cost = problem.stepcost(node.state, action, state)
                    fringe.append( Node(node, action, state, step_cost) )

    def depth_search(self, problem, max_depth=None, graph=True):
        closed = set()
        fringe = deque()
        fringe.append( Node(None, None, problem.init_state, 0) )
        while len(fringe):
            node = fringe.pop()
            if problem.isgoal(node.state):
                return node.solution()
            if node.state not in closed or not graph:
                if graph:
                    closed.add(node.state)
                if max_depth is None or node.depth <= max_depth:
                    for (action, state) in problem.successors(node.state):
                        step_cost = problem.stepcost(node.state, action, state)
                        fringe.append( Node(node, action, state, step_cost) )

    def iterative_search(self, problem, graph=True):
        n = 0
        while 1:
            solution = self.depth_search(problem, n, graph)
            if solution is None:
                n += 1
            else:
                return solution

    def greedy_search(self, problem, graph=True):
        closed = set()
        fringe = []
        root = Node(None, None, problem.init_state, 0)
        heappush(fringe, (self.greedy_eval(root), root))
        while len(fringe):
            (value, node) = heappop(fringe)
            if problem.isgoal(node.state):
                return node.solution()
            if node.state not in closed or not graph:
                if graph:
                    closed.add(node.state)
                for (action, state) in problem.successors(node.state):
                    step_cost = problem.stepcost(node.state, action, state)
                    newNode = Node(node, action, state, step_cost)
                    heappush(fringe, (self.greedy_eval(newNode), newNode))

    def astar_search(self, problem, graph=True):
        closed = {}
        fringe = []
        root = Node(None, None, problem.init_state, 0)
        heappush(fringe, (self.astar_eval(root), root))
        while len(fringe):
            (value, node) = heappop(fringe)
            if problem.isgoal(node.state):
                return node.solution()
            skip = False
            if graph and node.state in closed:
                skip = True
                if node.cost < closed[node.state]:
                    skip = False
            if not graph or not skip:
                if graph:
                    closed[node.state] = node.cost
                for (action, state) in problem.successors(node.state):
                    step_cost = problem.stepcost(node.state, action, state)
                    newNode = Node(node, action, state, step_cost)
                    heappush(fringe, (self.astar_eval(newNode), newNode))

    def greedy_eval(self, node):
        return 0

    def astar_eval(self, node):
        return node.cost


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


class Solution:
    """
    Contains init_state, a list of actions, and a cost
    """
    def __init__(self, init_state, actions, cost):
        self.init_state = init_state
        self.actions = actions
        self.cost = cost


def find_eb_factor(bf, depth, nodes):
    """
    Calculates the effective branching factor of
    a search.

    Node - If effective branching factor is not between
           1 and the branching factor then this will return
           a value near 1 or the branching factor.
    """
    return eb_factor_helper(1, bf, depth, nodes)


def eb_factor_helper(beg, end, d, n):
    """
    Does a binary search to estimate the solution to
    the equation.
    1 + x + x^2 + ... + x^d = n
    Assumes the answer is between beg and end
    """
    if float(end - beg) / 2 < 0.01:
        return float(beg + end) / 2
    nodes = summation(0, d, float(beg + end) / 2)
    if nodes > n:
        return eb_factor_helper(beg, float(beg + end) / 2, d, n)
    elif nodes < n:
        return eb_factor_helper(float(beg + end) / 2, end, d, n)
    else:
        return float(beg + end) / 2


def summation(i, n, b):
    """
    Evaluates the equation b^i + b^{i + 1} + ... + b^n
    """
    sum = 0
    while i <= n:
        sum += b ** i
        i += 1
    return sum
