import numpy as np
from RRTTree import RRTTree
import time


class RRTPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob):

        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''
        start_time = time.time()

        # initialize an empty plan.
        plan = []

        # TODO: Task 4.4

        env = self.planning_env
        start_state = env.start
        goal_state = env.goal
        self.tree = RRTTree(env)
        self.tree.add_vertex(state=start_state)

        while not self.tree.is_goal_exists(state=goal_state):

            # 1. sample
            if np.random.rand() < self.goal_prob:
                rand_state = goal_state
            else:
                [x_min, x_max] = env.xlimit
                [y_min, y_max] = env.ylimit
                x_rand = np.random.uniform(x_min, x_max)
                y_rand = np.random.uniform(y_min, y_max)
                rand_state = np.array([x_rand, y_rand])

            # 2. get nearest neighbor on the existing tree
            [near_id, near_state] = self.tree.get_nearest_state(state=rand_state)

            # 3. extend
            new_state = self.extend(near_state, rand_state)

            # 4. local planner
            if env.state_validity_checker(state=new_state) and \
                    env.edge_validity_checker(state1=near_state, state2=new_state):
                self.tree.add_vertex(state=new_state)
                new_id = self.tree.get_idx_for_state(state=new_state)
                dist = env.compute_distance(start_state=near_state, end_state=new_state)
                self.tree.add_edge(sid=near_id, eid=new_id, edge_cost=dist)

        curr_state_id = self.tree.get_idx_for_state(state=goal_state)
        while curr_state_id != self.tree.get_root_id():
            curr_state = self.tree.vertices[curr_state_id].state
            plan.append(curr_state)
            curr_state_id = self.tree.edges[curr_state_id]
        curr_state = self.tree.vertices[curr_state_id].state
        plan.append(curr_state)
        plan.reverse()

        # print total path cost and time
        print('Total cost of path: {:.2f}'.format(self.compute_cost(plan)))
        print('Total time: {:.2f}'.format(time.time()-start_time))

        return np.array(plan)

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps.
        @param plan A given plan for the robot.
        '''
        # TODO: Task 4.4
        cost = 0
        env = self.planning_env
        for idx in range(len(plan)-1):
            cost += env.compute_distance(start_state=plan[idx], end_state=plan[idx+1])
        return cost

    def extend(self, near_state, rand_state):
        '''
        Compute and return a new position for the sampled one.
        @param near_state The nearest position to the sampled position.
        @param rand_state The sampled position.
        '''
        # TODO: Task 4.4

        eta = 1 #0.7
        if self.ext_mode == 'E1':
            return rand_state
        elif self.ext_mode == 'E2':
            return near_state + eta * (rand_state - near_state)
        else:
            assert 0
