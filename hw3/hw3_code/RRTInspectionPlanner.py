import numpy as np
from RRTTree import RRTTree
import time
import Robot


class RRTInspectionPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob, coverage):

        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env, task="ip")

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob
        self.coverage = coverage

        #self.goal_prob = 0.05
        self.eta = 0.35
        # after some tuning
        if self.goal_prob == 0.05:
            self.eta = 0.3
        if self.goal_prob == 0.1:
            self.eta = 0.3
        if self.goal_prob == 0.2:
            self.eta = 0.2

        self.Robot = Robot.Robot()

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states in the configuration space.
        '''
        start_time = time.time()

        # initialize an empty plan.
        plan = []

        # TODO: Task 2.4

        env = self.planning_env
        start_config = env.start

        num_of_runs_for_average = 10

        sum_cost = 0
        sum_time = 0
        min_cost = np.inf

        for run_idx in range(num_of_runs_for_average):

            start_time = time.time()
            cur_plan = []
            self.tree = RRTTree(self.planning_env, task="ip")
            start_config_poi = env.get_inspected_points(config=start_config)
            self.tree.add_vertex(start_config, start_config_poi)

            is_new_milestone_detected = False
            num_of_near_milestone_config_checks = 10
            near_milestone_config_checks_count = 0
            milestones = []

            while self.tree.max_coverage < self.coverage:
            
                # 1. sample
                if is_new_milestone_detected:
                    near_milestone_config_checks_count += 1
                    rand_state = self.get_near_milestone_config(milestones[-1])
                    if near_milestone_config_checks_count == num_of_near_milestone_config_checks:
                        is_new_milestone_detected = False
                elif np.random.rand() < self.goal_prob:
                    rand_state = self.get_poi_biased_config(milestones)
                else:
                    rand_state = self.get_random_config()

                # 2. get nearest neighbor on the existing tree
                [near_id, near_state] = self.tree.get_nearest_config(config=rand_state)

                # 3. extend
                new_state = self.extend(near_state, rand_state)

                # 4. local planner
                if env.config_validity_checker(config=new_state) and \
                        env.edge_validity_checker(config1=near_state, config2=new_state):
                    tree_near_poi = self.tree.vertices[near_id].inspected_points
                    new_poi = env.get_inspected_points(config=new_state)
                    unified_poi = env.compute_union_of_points(tree_near_poi, new_poi)
                    new_id = self.tree.add_vertex(config=new_state, inspected_points=unified_poi)
                    dist = self.Robot.compute_distance(prev_config=near_state, next_config=new_state)
                    self.tree.add_edge(sid=near_id, eid=new_id, edge_cost=dist)
                    if len(unified_poi) > len(tree_near_poi):
                        is_new_milestone_detected = True
                        near_milestone_config_checks_count = 0
                        milestones.append(new_state)

            curr_id = self.tree.max_coverage_id
            cur_plan.append(self.tree.vertices[curr_id].config)
            while curr_id != self.tree.get_root_id():
                curr_id = self.tree.edges[curr_id]
                cur_plan.append(self.tree.vertices[curr_id].config)
            cur_plan.reverse()

            # print total path cost and time
            cur_cost = self.compute_cost(cur_plan)
            run_time = time.time() - start_time
            print('Total cost of path (run {}): {:.2f}'.format(run_idx, cur_cost))
            print('Total time (run {}): {:.2f}'.format(run_idx, run_time))
            sum_cost += cur_cost
            sum_time += run_time
            if cur_cost < min_cost:
                min_cost = cur_cost
                plan = cur_plan[:]
        avg_cost = sum_cost/num_of_runs_for_average
        avg_time = sum_time/num_of_runs_for_average
        print('Calc plan for:')
        print(f'Coverage: {self.coverage}')
        print(f'Goal prob: {self.goal_prob}')
        print(f'Extend mode: {self.ext_mode}')
        if self.ext_mode == 'E2':
            print(f'eta: {self.eta}')

        print('Best plan cost: {:.2f}'.format(min_cost))
        print('Avg cost of path: {:.2f}'.format(avg_cost))
        print('Avg time: {:.2f}'.format(avg_time))

        return np.array(plan)

    def get_random_config(self):
        min_angle = -np.pi
        max_angle = np.pi
        rand_config = \
            np.array([np.random.uniform(0, np.pi/2),
                      np.random.uniform(min_angle, max_angle),
                      np.random.uniform(min_angle, max_angle),
                      np.random.uniform(min_angle, max_angle)])
        return rand_config

    def get_near_milestone_config(self, milestone_config):
        close_config = np.array([np.random.uniform(milestone_config[0] - np.pi/16, milestone_config[0] + np.pi/16),
                                 np.random.uniform(milestone_config[1] - np.pi/12, milestone_config[1] + np.pi/12),
                                 np.random.uniform(milestone_config[2] - np.pi/8, milestone_config[2] + np.pi/8),
                                 np.random.uniform(milestone_config[3] - np.pi/4, milestone_config[3] + np.pi/4)])
        return close_config

    def get_poi_biased_config(self, milestones):
        if len(milestones) > 0:
            random_int = np.random.randint(0, len(milestones))
            milestone = milestones[random_int]
            close_config = self.get_near_milestone_config(milestone)
            return close_config
        else:
            return self.get_random_config()

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps in the configuration space.
        @param plan A given plan for the robot.
        '''
        # TODO: Task 2.4

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
        # TODO: Task 2.4
        if self.ext_mode == 'E1':
            return rand_config
        elif self.ext_mode == 'E2':
            diff_vector = (rand_config - near_config)
            diff_vector_norm = self.Robot.compute_distance(rand_config, near_config)
            if np.isclose(diff_vector_norm, 0):
                # Handle the case where diff_vector_norm is zero or very small
                # You can set the direction vector to a default value or use a fallback strategy
                unit_direction_vector = np.zeros_like(diff_vector)  # Default: Zero vector
            else:
                # Calculate the direction vector
                unit_direction_vector = diff_vector / diff_vector_norm

            new_point = rand_config
            if self.eta < diff_vector_norm:
                new_point = near_config + self.eta * unit_direction_vector
            return new_point
        else:
            assert 0