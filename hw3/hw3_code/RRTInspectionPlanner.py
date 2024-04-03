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

        num_of_runs_for_average = 1

        average_runs_enable = False
        if average_runs_enable:
            num_of_runs_for_average = 10

        self.eta = 0.35

        tuning_enable = True
        if tuning_enable:
            #self.goal_prob = 0.05
            # after some tuning
            if self.goal_prob == 0.05:
                self.eta = 0.3
            if self.goal_prob == 0.1:
                self.eta = 0.3
            if self.goal_prob == 0.2:
                self.eta = 0.2

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
            num_of_near_milestone_config_checks = 20
            near_milestone_config_checks_count = 0
            milestones = []
            is_new_improving_config_detected = False
            num_of_near_improving_config_checks = 5
            near_improving_config_checks_count = 0
            improving = []

            max_poi_size = 0
            max_poi = None

            enable_extreme_and_mean_config_focus = True

            enable_poi_com_explore = True
            find_com_config = True
            close_to_com_counter = 0
            close_to_com_counter_thresh = 10

            while self.tree.max_coverage < self.coverage:

                if self.tree.max_coverage >= 0.75:
                    if self.goal_prob != 0.7:
                        self.goal_prob = 0.7
                        #print(f"increase goal prob to {self.goal_prob}")

                # 1. sample
                if is_new_milestone_detected:
                    near_milestone_config_checks_count += 1
                    rand_state = self.get_near_milestone_config(milestones[-1])
                    if near_milestone_config_checks_count == num_of_near_milestone_config_checks:
                        is_new_milestone_detected = False
                elif is_new_improving_config_detected and self.tree.max_coverage < 0.75:
                    near_improving_config_checks_count += 1
                    rand_state = self.get_near_milestone_config(improving[-1])
                    if near_improving_config_checks_count == num_of_near_improving_config_checks:
                        is_new_improving_config_detected = False
                elif np.random.rand() < self.goal_prob:
                    #if np.random.rand() < 0.1:
                    #    print(f"max_coverage = {self.tree.max_coverage}")
                    #print(f"goal prob {self.goal_prob}")
                    if enable_poi_com_explore and self.tree.max_coverage >= 0.8:
                        if find_com_config:
                            box_size = 30
                            poi_com_x = np.mean(max_poi[:, 0])
                            poi_com_y = np.mean(max_poi[:, 1])
                            #print(f"poi_com_x = {poi_com_x}")
                            #print(f"poi_com_y = {poi_com_y}")
                            com_valid_config = self.get_poi_biased_config(milestones)
                            found_com_valid_config = False
                            for i in range(200):
                                test_config = self.get_poi_biased_config(milestones)
                                joints_loc = self.Robot.compute_forward_kinematics(test_config)
                                ee_loc = joints_loc[-1]
                                #ee_orientation = np.sum(test_config)
                                in_box_x = (ee_loc[0] < poi_com_x + box_size) and (ee_loc[0] > poi_com_x - box_size)
                                in_box_y = (ee_loc[1] < poi_com_y + box_size) and (ee_loc[1] > poi_com_y - box_size)
                                if in_box_x and in_box_y and env.config_validity_checker(config=test_config):
                                    com_valid_config = test_config
                                    found_com_valid_config = True
                                    break
                            if found_com_valid_config:
                                find_com_config = False
                                close_to_com_counter = 0
                                #print(f"found com valid config {com_valid_config}")
                            rand_state = com_valid_config
                        else:
                            close_to_com_counter += 1
                            if close_to_com_counter == close_to_com_counter_thresh:
                                find_com_config = True
                            rand_state = self.get_near_poi_com_config(com_valid_config,
                                                                      close_to_com_counter/ close_to_com_counter_thresh)

                    elif enable_extreme_and_mean_config_focus and self.tree.max_coverage >= 0.75:
                        sums = np.array([np.sum(arr) for arr in milestones])
                        max_sum_config = max(milestones, key=lambda array: np.sum(array))
                        min_sum_config = min(milestones, key=lambda array: np.sum(array))
                        median_sum = np.median(sums)
                        index = np.argmin(np.abs(sums - median_sum))
                        median_sum_config = milestones[index]
                        mean_sum = np.mean(sums)
                        index = np.argmin(np.abs(sums - mean_sum))
                        mean_sum_config = milestones[index]

                        if np.random.rand() < 0.7:
                            if np.random.rand() < 0.25:
                                #print(f" max_sum_config = {max_sum_config}")
                                rand_state = self.get_near_milestone_config(max_sum_config, range_factor=3)
                            elif np.random.rand() < 0.5:
                                #print(f" median_sum_config = {median_sum_config}")
                                rand_state = self.get_near_milestone_config(median_sum_config, range_factor=3)
                            elif np.random.rand() < 0.75:
                                #print(f" mean_sum_config = {mean_sum_config}")
                                rand_state = self.get_near_milestone_config(mean_sum_config, range_factor=3)
                            else:
                                #print(f" min_sum_config = {min_sum_config}")
                                rand_state = self.get_near_milestone_config(min_sum_config, range_factor=3)
                        else:
                            rand_state = self.get_poi_biased_config(milestones)
                    else:
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

                    if len(unified_poi) > len(new_poi):
                        is_new_improving_config_detected = True
                        near_improving_config_checks_count = 0
                        improving.append(new_state)
                    if len(unified_poi) > max_poi_size:
                        max_poi_size = len(unified_poi)
                        max_poi = unified_poi
                        #print(unified_poi)
                        #print(f"len(unified_poi) {len(unified_poi)}")
                        #print(f"len(tree_near_poi) {len(tree_near_poi)}")
                        #print("new_milestone_detected")
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

    def get_near_milestone_config(self, milestone_config, range_factor=None):
        if range_factor is None:
            range_factor = 4
        close_config = np.array([np.random.uniform(milestone_config[0] - np.pi/(4*range_factor), milestone_config[0] + np.pi/(4*range_factor)),
                                 np.random.uniform(milestone_config[1] - np.pi/(3*range_factor), milestone_config[1] + np.pi/(3*range_factor)),
                                 np.random.uniform(milestone_config[2] - np.pi/(2*range_factor), milestone_config[2] + np.pi/(2*range_factor)),
                                 np.random.uniform(milestone_config[3] - np.pi/range_factor, milestone_config[3] + np.pi/range_factor)])
        return close_config

    def get_near_poi_com_config(self, poi_com_config, frac):
        close_config = poi_com_config
        joint3_angle = poi_com_config[3] + (frac*2*np.pi)
        joint3_angle = self.Robot.compute_link_angle(0, joint3_angle)
        close_config[3] = joint3_angle
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