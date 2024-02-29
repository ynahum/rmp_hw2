import matplotlib.pyplot as plt
import numpy as np
from RRTTree import RRTTree
import time

class RRTStarPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob, k, num_of_runs_for_average=1, eta=8, timeout=None):

        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob
        self.k = k
        if k == 0:
            self.const_k = False
        else:
            self.const_k = True
        self.eta = eta
        self.num_of_runs_for_average = num_of_runs_for_average
        self.timeout = timeout

    def get_knn(self, new_state):
        if self.k >= len(self.tree.vertices):
            knn_ids = []
            knn_states = []
            for vid in range(len(self.tree.vertices)):
                knn_ids.append(vid)
                knn_states.append(self.tree.vertices[vid].state)
            return [knn_ids, knn_states]
        else:
            return self.tree.get_k_nearest_neighbors(state=new_state, k=self.k)

    def filter_valid_edges_from_state_to_neighbors(self, state, neighbors_states):
        env = self.planning_env
        filtered_neighbors_ids = []
        for n_state in neighbors_states:
            if env.edge_validity_checker(state1=state, state2=n_state):
                filtered_neighbors_ids.append(self.tree.get_idx_for_state(n_state))
        return filtered_neighbors_ids

    def rewire(self, potential_parent_vertex, child_vertex):
        # if collision free
        #   edge_cost <- dist(potential_parent, child)
        #   if cost(potential_parent) + edge_cost < cost(child):
        #      child.parent = potential_parent

        # we have checked already collision (filtered only knn which can connect)
        # and now we just need to check if there is some cost improvement
        edge_cost = self.planning_env.compute_distance(potential_parent_vertex.state, child_vertex.state)
        if child_vertex.cost > (potential_parent_vertex.cost + edge_cost):
            parent_id = self.tree.get_idx_for_state(potential_parent_vertex.state)
            child_id = self.tree.get_idx_for_state(child_vertex.state)
            self.tree.add_edge(parent_id, child_id, edge_cost)

    def plots(self, run_times, costs):
        num_of_runs = len(run_times)
        max_t = int(np.max(run_times))+1
        t = np.arange(start=0, stop=max_t)
        run_times_np_arr = np.array(run_times)
        run_time_sorted_indices = np.argsort(run_times_np_arr)
        run_times_sorted = run_times_np_arr[run_time_sorted_indices]
        rate_of_success = []
        run_times_idx = 0
        success_counter = 0
        for t_idx, t_val in enumerate(t):
            while run_times_idx < num_of_runs and t_val >= run_times_sorted[run_times_idx]:
                run_times_idx += 1
                success_counter += 1
            rate_of_success.append(success_counter/num_of_runs)
        if self.const_k is True:
            title_append = f'const K = {self.k}'
        else:
            title_append = f'K = log(i)'
        title = 'success rate vs time '
        title += title_append
        plt.plot(t, rate_of_success)
        plt.title(title)
        plt.xlabel('time')
        plt.ylabel('success rate')
        plt.show()

        #cost_sorted_indices = np.argsort(costs_np_arr)
        #run_times_cost_sorted = run_times_np_arr[cost_sorted_indices]
        costs_np_arr = np.array(costs)
        costs_sorted = costs_np_arr[run_time_sorted_indices]
        title = 'quality vs time '
        title += title_append

        plt.plot(run_times_sorted, costs_sorted)
        plt.title(title)
        plt.xlabel('time')
        plt.ylabel('cost')
        plt.show()


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

        best_cost = 10000
        best_plan = []
        run_times = []
        successes = 0
        costs = []
        sum_cost = 0
        sum_time = 0
        for run_idx in range(self.num_of_runs_for_average):

            start_time = time.time()
            plan = []

            self.tree = RRTTree(env)
            self.tree.add_vertex(state=start_state)

            while True:

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
                    new_id = self.tree.add_vertex(state=new_state)
                    dist = env.compute_distance(start_state=near_state, end_state=new_state)
                    self.tree.add_edge(sid=near_id, eid=new_id, edge_cost=dist)

                    if self.const_k is False:
                        self.k = int(np.ceil(np.log2(len(self.tree.vertices))))
                    [_, knn_states] = self.get_knn(new_state)
                    knn_id_valids = self.filter_valid_edges_from_state_to_neighbors(new_state, knn_states)
                    for nn_id in knn_id_valids:
                        self.rewire(self.tree.vertices[nn_id], self.tree.vertices[new_id])
                    for nn_id in knn_id_valids:
                        self.rewire(self.tree.vertices[new_id], self.tree.vertices[nn_id])

                if self.timeout is not None:
                    if time.time()-start_time > self.timeout:
                        break
                elif self.tree.is_goal_exists(state=goal_state):
                    break
            if self.tree.is_goal_exists(state=goal_state):
                # get plan
                curr_state_id = self.tree.get_idx_for_state(state=goal_state)
                while curr_state_id != self.tree.get_root_id():
                    curr_state = self.tree.vertices[curr_state_id].state
                    plan.append(curr_state)
                    curr_state_id = self.tree.edges[curr_state_id]
                curr_state = self.tree.vertices[curr_state_id].state
                plan.append(curr_state)
                plan.reverse()

                successes += 1

                # print total path cost and time
                cost = self.compute_cost(plan)
                run_time = time.time() - start_time
                run_times.append(run_time)
                costs.append(cost)
                print('Total cost of path (run {}): {:.2f}'.format(run_idx, cost))
                print('Total time (run {}): {:.2f}'.format(run_idx, run_time))
                sum_cost += cost
                sum_time += run_time
                if cost < best_cost:
                    best_cost = cost
                    best_plan = plan[:]

        #self.plots(run_times, costs)
        if successes > 0:
            avg_cost = sum_cost / successes
            avg_time = sum_time / successes
            success_rate = successes / self.num_of_runs_for_average
            print('Best plan cost: {:.2f}'.format(best_cost))
            print('Avg cost of path: {:.2f}'.format(avg_cost))
            print('Avg time: {:.2f}'.format(avg_time))
        if self.timeout is not None:
            print(f"timeout {self.timeout}")
        print(f"success rate {success_rate}")
        print(f'Goal prob: {self.goal_prob}')
        if self.const_k is True:
            print(f'const k: {self.k}')
        else:
            print(f'k is log(i)')
        print(f'Extend mode: {self.ext_mode}')
        if self.ext_mode == 'E2':
            print(f'eta: {self.eta}')


        return np.array(best_plan)

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps.
        @param plan A given plan for the robot.
        '''
        # TODO: Task 4.4
        cost = 0
        for idx in range(len(plan)-1):
            cost += self.planning_env.compute_distance(start_state=plan[idx], end_state=plan[idx+1])
        return cost

    def extend(self, near_state, rand_state):
        '''
        Compute and return a new position for the sampled one.
        @param near_state The nearest position to the sampled position.
        @param rand_state The sampled position.
        '''
        # TODO: Task 4.4

        if self.ext_mode == 'E1':
            return rand_state
        elif self.ext_mode == 'E2':
            diff_vector = (rand_state - near_state)
            diff_vector_norm = self.planning_env.compute_distance(start_state=rand_state, end_state=near_state)
            if np.isclose(diff_vector_norm, 0):
                # Handle the case where diff_vector_norm is zero or very small
                # You can set the direction vector to a default value or use a fallback strategy
                unit_direction_vector = np.zeros_like(diff_vector)  # Default: Zero vector
            else:
                # Calculate the direction vector
                unit_direction_vector = diff_vector / diff_vector_norm

            new_point = rand_state
            if self.eta < diff_vector_norm:
                new_point = near_state + self.eta * unit_direction_vector
            return new_point
        else:
            assert 0
    