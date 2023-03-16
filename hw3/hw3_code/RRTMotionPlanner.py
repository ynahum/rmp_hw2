import numpy as np
from RRTTree import RRTTree
import time
import Robot

class RRTMotionPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob):

        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob
        self.eta = 0.5

        self.num_of_runs_for_average = 10
        self.Robot = Robot.Robot()

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states in the configuration space.
        '''
        start_time = time.time()

        # initialize an empty plan.
        plan = []

        # TODO: Task 2.3

        env = self.planning_env
        start_state = env.start
        goal_state = env.goal

        sum_cost = 0
        sum_time = 0
        best_cost = 10000
        best_plan = []

        for run_idx in range(self.num_of_runs_for_average):

            start_time = time.time()
            plan = []

            self.tree = RRTTree(env)
            self.tree.add_vertex(config=start_state)

            while not self.tree.is_goal_exists(config=goal_state):

                # 1. sample
                if np.random.rand() < self.goal_prob:
                    rand_state = goal_state
                else:
                    min_angle = -np.pi
                    max_angle = np.pi
                    rand_state = \
                        np.array([np.random.uniform(min_angle, max_angle),
                                  np.random.uniform(min_angle, max_angle),
                                  np.random.uniform(min_angle, max_angle),
                                  np.random.uniform(min_angle, max_angle)])

                # 2. get nearest neighbor on the existing tree
                [near_id, near_state] = self.tree.get_nearest_config(config=rand_state)

                # 3. extend
                new_state = self.extend(near_state, rand_state)

                # 4. local planner
                if env.config_validity_checker(config=new_state) and \
                        env.edge_validity_checker(config1=near_state, config2=new_state):
                    new_id = self.tree.add_vertex(config=new_state)
                    dist = self.Robot.compute_distance(prev_config=near_state, next_config=new_state)
                    self.tree.add_edge(sid=near_id, eid=new_id, edge_cost=dist)

            curr_state_id = self.tree.get_idx_for_config(config=goal_state)
            while curr_state_id != self.tree.get_root_id():
                curr_state = self.tree.vertices[curr_state_id].config
                plan.append(curr_state)
                curr_state_id = self.tree.edges[curr_state_id]
            curr_state = self.tree.vertices[curr_state_id].config
            plan.append(curr_state)
            plan.reverse()

            # print total path cost and time
            cost = self.compute_cost(plan)
            run_time = time.time() - start_time
            print('Total cost of path (run {}): {:.2f}'.format(run_idx, cost))
            print('Total time (run {}): {:.2f}'.format(run_idx, run_time))
            sum_cost += cost
            sum_time += run_time
            if cost < best_cost:
                best_cost = cost
                best_plan = plan[:]
        avg_cost = sum_cost/self.num_of_runs_for_average
        avg_time = sum_time/self.num_of_runs_for_average
        print('Calc plan for:')
        print(f'Goal prob: {self.goal_prob}')
        print(f'Extend mode: {self.ext_mode}')
        if self.ext_mode == 'E2':
            print(f'eta: {self.eta}')

        print('Best plan cost: {:.2f}'.format(best_cost))
        print('Avg cost of path: {:.2f}'.format(avg_cost))
        print('Avg time: {:.2f}'.format(avg_time))

        return np.array(best_plan)


    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps in the configuration space.
        @param plan A given plan for the robot.
        '''
        # TODO: Task 2.3
        cost = 0
        for idx in range(len(plan)-1):
            cost += self.Robot.compute_distance(plan[idx], plan[idx+1])
        return cost


    def extend(self, near_config, rand_config):
        '''
        Compute and return a new configuration for the sampled one.
        @param near_config The nearest configuration to the sampled configuration.
        @param rand_config The sampled configuration.
        '''
        # TODO: Task 2.3
        if self.ext_mode == 'E1':
            return rand_config
        elif self.ext_mode == 'E2':
            return near_config + self.eta * (rand_config - near_config)
        else:
            assert 0
