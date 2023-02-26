import numpy as np
import heapq


class AStarNode:
    def __init__(self, state, parent, g, f):
        self.state = state
        self.parent = parent

        self.g = g
        self.f = f

    def __lt__(self, other):
        return (self.f < other.f) or (self.f == other.f and ((self.f - self.g) < (other.f - other.g)))


class AStarPlanner(object):
    def __init__(self, planning_env, epsilon=1):
        self.planning_env = planning_env
        self.epsilon = epsilon
        self.actions = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
        # used for visualizing the expanded nodes
        # make sure that this structure will contain a list of positions (states, numpy arrays) without duplicates
        self.expanded_nodes = [] 

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''

        # initialize an empty plan.
        plan = []

        # TODO: Task 4.3
        env = self.planning_env
        start_state = tuple(env.start)
        start_h = env.compute_heuristic(start_state)
        start_node = AStarNode(state=start_state, parent=None, g=0, f=start_h)

        # The OPEN priority queue
        frontier = []
        heapq.heappush(frontier, start_node)

        # helper hash, for faster search of a state in a set
        frontier_states = dict()
        frontier_states[start_node.state] = start_node

        # The CLOSE set
        reached_states = dict()
        goal_state = tuple(env.goal)

        while frontier:
            node = heapq.heappop(frontier)
            del frontier_states[node.state]

            self.expanded_nodes.append(node.state)

            if node.state == goal_state:
                break

            reached_states[node.state] = node

            curr_state_list = list(node.state)
            for action in self.actions:
                new_state = (node.state[0] + action[0], node.state[1] + action[1])
                new_state_list = list(new_state)
                if not env.state_validity_checker(new_state_list):
                    continue
                # Make sure walkable terrain
                if not env.edge_validity_checker(curr_state_list, new_state_list):
                    continue
                new_g = node.g + env.compute_distance(curr_state_list, new_state_list)
                new_h = env.compute_heuristic(new_state_list)
                new_f = new_g + self.epsilon * new_h
                # if we didn't see this state before (not in OPEN and not in CLOSE)
                if not ((new_state in reached_states) or (new_state in frontier_states)):
                    new_node = AStarNode(state=new_state, parent=node, g=new_g, f=new_f)
                    heapq.heappush(frontier, new_node)
                    frontier_states[new_node.state] = new_node
                # else if the next state is in OPEN
                elif new_state in frontier_states:
                    n_curr = frontier_states[new_state]
                    if new_g < n_curr.g:
                        # we update the node cost to go as we found a better path to it
                        n_curr.g = new_g
                        n_curr.f = new_f
                        heapq.heapify(frontier)
                # else the next state was already in CLOSE
                else:
                    assert (new_state in reached_states)
                    n_curr = reached_states[new_state]
                    if new_g < n_curr.g:
                        # we update the node cost to go as we found a better path to it
                        n_curr.g = new_g
                        n_curr.f = new_f
                        heapq.heappush(frontier, n_curr)
                        frontier_states[new_state] = n_curr
                        del reached_states[new_state]

        total_cost = 0
        current_node = node
        while current_node is not None:
            current_list = list(current_node.state)
            plan.append(np.array(current_list))
            if current_node.parent is not None:
                parent_state_list = list(current_node.parent.state)
                total_cost += env.compute_distance(np.array(parent_state_list), np.array(current_list))
            current_node = current_node.parent
        plan.reverse()
        print(f"total cost: {total_cost}")
        return np.array(plan)

    def get_expanded_nodes(self):
        '''
        Return list of expanded nodes without duplicates.
        '''

        # used for visualizing the expanded nodes
        return self.expanded_nodes
