# valueIterationAgents.py
# -----------------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


# valueIterationAgents.py
# -----------------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


import mdp, util

from learningAgents import ValueEstimationAgent
import collections

class ValueIterationAgent(ValueEstimationAgent):
    """
        * Please read learningAgents.py before reading this.*

        A ValueIterationAgent takes a Markov decision process
        (see mdp.py) on initialization and runs value iteration
        for a given number of iterations using the supplied
        discount factor.
    """
    def __init__(self, mdp, discount = 0.9, iterations = 100):
        """
          Your value iteration agent should take an mdp on
          construction, run the indicated number of iterations
          and then act according to the resulting policy.

          Some useful mdp methods you will use:
              mdp.getStates()
              mdp.getPossibleActions(state)
              mdp.getTransitionStatesAndProbs(state, action)
              mdp.getReward(state, action, nextState)
              mdp.isTerminal(state)
        """
        self.mdp = mdp
        self.discount = discount
        self.iterations = iterations
        self.values = util.Counter() # A Counter is a dict with default 0
        self.runValueIteration()

    def runValueIteration(self):
        # Write value iteration code here
        "*** YOUR CODE HERE ***"
        for i in range (self.iterations):
            next_value = util.Counter()
            for state in self.mdp.getStates():
                if not self.mdp.isTerminal(state):
                    action = self.getAction(state)
                    next_value[state] = self.computeQValueFromValues(state,action)
            self.values = next_value
            #     max_value = float('inf')
            #     for action in self.mdp.getPossibleActions(state):
            #         q = self.computeQValueFromValues(state, action)
            #         if q > max_value:
            #             max_value = q
            #             next_value[state] = q
            #             self.actions[state] = action
            # self.values = next_value

    def getValue(self, state):
        """
          Return the value of the state (computed in __init__).
        """
        return self.values[state]


    def computeQValueFromValues(self, state, action):
        """
          Compute the Q-value of action in state from the
          value function stored in self.values.
        """
        "*** YOUR CODE HERE ***"''
        q = 0
        for next_state, p in self.mdp.getTransitionStatesAndProbs(state, action):
            q += p * (self.mdp.getReward(state, action, next_state) + self.discount * self.values[next_state])
        return q

        util.raiseNotDefined()

    def computeActionFromValues(self, state):
        """
          The policy is the best action in the given state
          according to the values currently stored in self.values.

          You may break ties any way you see fit.  Note that if
          there are no legal actions, which is the case at the
          terminal state, you should return None.
        """
        "*** YOUR CODE HERE ***"
        if self.mdp.isTerminal(state):
            return None
        best_action = None
        best_value = float('-inf')
        possible_actions = self.mdp.getPossibleActions(state)
        for action in possible_actions:
            v = 0
            q = self.computeQValueFromValues(state, action)
            if best_value < q:
                best_value = q
                best_action = action
        return best_action


        util.raiseNotDefined()

    def getPolicy(self, state):
        return self.computeActionFromValues(state)

    def getAction(self, state):
        "Returns the policy at the state (no exploration)."
        return self.computeActionFromValues(state)

    def getQValue(self, state, action):
        return self.computeQValueFromValues(state, action)

class AsynchronousValueIterationAgent(ValueIterationAgent):
    """
        * Please read learningAgents.py before reading this.*

        An AsynchronousValueIterationAgent takes a Markov decision process
        (see mdp.py) on initialization and runs cyclic value iteration
        for a given number of iterations using the supplied
        discount factor.
    """
    def __init__(self, mdp, discount = 0.9, iterations = 1000):
        """
          Your cyclic value iteration agent should take an mdp on
          construction, run the indicated number of iterations,
          and then act according to the resulting policy. Each iteration
          updates the value of only one state, which cycles through
          the states list. If the chosen state is terminal, nothing
          happens in that iteration.

          Some useful mdp methods you will use:
              mdp.getStates()
              mdp.getPossibleActions(state)
              mdp.getTransitionStatesAndProbs(state, action)
              mdp.getReward(state)
              mdp.isTerminal(state)
        """
        ValueIterationAgent.__init__(self, mdp, discount, iterations)

    def runValueIteration(self):
        "*** YOUR CODE HERE ***"
        states = self.mdp.getStates()
        for k in range(self.iterations):
            state = states[k % len(states)]
            possible_actions = self.mdp.getPossibleActions(state)
            if not possible_actions:
                continue
            else:
                q_values = []
                for action in self.mdp.getPossibleActions(state):
                    q_values.append(self.computeQValueFromValues(state, action))
                maxq= max(q_values)
                self.values[state] = maxq

class PrioritizedSweepingValueIterationAgent(AsynchronousValueIterationAgent):
    """
        * Please read learningAgents.py before reading this.*

        A PrioritizedSweepingValueIterationAgent takes a Markov decision process
        (see mdp.py) on initialization and runs prioritized sweeping value iteration
        for a given number of iterations using the supplied parameters.
    """
    def __init__(self, mdp, discount = 0.9, iterations = 100, theta = 1e-5):
        """
          Your prioritized sweeping value iteration agent should take an mdp on
          construction, run the indicated number of iterations,
          and then act according to the resulting policy.
        """
        self.theta = theta
        ValueIterationAgent.__init__(self, mdp, discount, iterations)

    def max_q(self, state):
        q_values = []
        possible_actions = self.mdp.getPossibleActions(state)
        if len(possible_actions) != 0:
            for action in possible_actions:
                q_values.append(self.computeQValueFromValues(state, action))
            return max(q_values)
        else:
            return self.values[state]



    def runValueIteration(self):
        "*** YOUR CODE HERE ***"
        p_queue = util.PriorityQueue()
        states = self.mdp.getStates()
        parents = {}
        for state in states:
            parents[state] = set()
        for state in states:
            possible_actions = self.mdp.getPossibleActions(state)
            for action in possible_actions:
                transitions = self.mdp.getTransitionStatesAndProbs(state, action)
                for trans_state, p in transitions:
                    parents[trans_state].add(state)

        for state in states:
            if state == "TERMINAL_STATE":
                continue
            maxq = self.max_q(state)
            difference = abs(self.values[state] - maxq)
            p_queue.push(state, -difference)

        counter = 0
        while counter < self.iterations:
            if p_queue.isEmpty():
                return
            state = p_queue.pop()
            self.values[state] = self.max_q(state)
            for p in parents[state]:
                maxq = self.max_q(p)
                difference = abs(self.values[p] - maxq)
                if difference > self.theta:
                    p_queue.update(p, -difference)
            counter += 1





