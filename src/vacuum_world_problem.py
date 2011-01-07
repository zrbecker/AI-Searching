# To change this template, choose Tools | Templates
# and open the template in the editor.

from searching import *
import random

class VacuumWorld(Problem):
    """
    This world has two locations an agent can be in. Each of
    the location can have dirt or not have dirt.

    The goal of this problem is to find a optimal solution to clean
    both locations of the world.

    States:
    0 - (D)D
    1 - D(D)
    2 - (C)D
    3 - C(D)
    4 - (D)C
    5 - D(C)
    6 - (C)C
    7 - C(C)

    D - Dirty | C - Clean | () - Location of Agent
    """
    ACTION_LEFT  = 0
    ACTION_RIGHT = 1
    ACTION_SUCK  = 2

    def __init__(self):
        """
        Picks a random initial state.
        """
        self.init_state = random.randint(0, 7)

    def successorFunction(self, state):
        """
        Returns a list of all the actions and resulting states that
        can be done in a given state.
        """
        return [(self.ACTION_LEFT, self.doAction(state, self.ACTION_LEFT)),
                (self.ACTION_RIGHT, self.doAction(state, self.ACTION_RIGHT)),
                (self.ACTION_SUCK, self.doAction(state, self.ACTION_SUCK))]

    def doAction(self, state, action):
        """
        Returns a result state after performing the supplied action
        in the supplied state.
        """
        if action == self.ACTION_LEFT:
            if   state in [0, 1]: return 0
            elif state in [2, 3]: return 2
            elif state in [4, 5]: return 4
            elif state in [6, 7]: return 6
        elif action == self.ACTION_RIGHT:
            if   state in [0, 1]: return 1
            elif state in [2, 3]: return 3
            elif state in [4, 5]: return 5
            elif state in [6, 7]: return 7
        elif action == self.ACTION_SUCK:
            if   state in [0, 2]: return 2
            elif state in [1, 5]: return 5
            elif state in [4, 6]: return 6
            elif state in [3, 7]: return 7

    def goalTest(self, state):
        """
        Returns true if in goal state
        """
        return state in [6, 7]

def main():
    vacuum_problem = VacuumWorld()
    solution = breadthFirstSearch(vacuum_problem)
    print 'Initial State:', vacuum_problem.init_state

    if solution is None:
        print 'No solutions found'
    elif len(solution) == 0:
        print 'Started out in goal state'
    else:
        for action in solution:
            if action == VacuumWorld.ACTION_LEFT:
                print 'Left'
            elif action == VacuumWorld.ACTION_RIGHT:
                print 'Right'
            else:
                print 'Suck'
    
if __name__ == '__main__':
    main()
