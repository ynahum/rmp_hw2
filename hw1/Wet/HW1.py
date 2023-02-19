import argparse
import os
from typing import List, Tuple

import shapely.geometry
import shapely.affinity

from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString

import heapq as heap


# TODO
def get_minkowsky_sum(original_shape: Polygon, r: float) -> Polygon:
    """
    Get the polygon representing the Minkowsky sum
    :param original_shape: The original obstacle
    :param r: The radius of the rhombus
    :return: The polygon composed from the Minkowsky sums
    """
    convex_hull_method = False
    num_of_ob_points = len(original_shape.exterior.coords) - 1
    if convex_hull_method:
        total_points = num_of_ob_points * 4 * [0]
        for idx in range(num_of_ob_points):
            idx_offset = idx*4
            curr_obs_point = original_shape.exterior.coords[idx]
            total_points[idx_offset] = tuple(map(lambda i, j: i + j, (r, 0), curr_obs_point))
            total_points[idx_offset + 1] = tuple(map(lambda i, j: i + j, (0, r), curr_obs_point))
            total_points[idx_offset + 2] = tuple(map(lambda i, j: i + j, (-r, 0), curr_obs_point))
            total_points[idx_offset + 3] = tuple(map(lambda i, j: i + j, (0, -r), curr_obs_point))
        ret_poly = Polygon(total_points).convex_hull
    else:
        r_poly_points = [(0, -r), (r, 0), (0, r), (-r, 0)]
        ob_poly = shapely.geometry.polygon.orient(original_shape, 1)
        assert ob_poly.exterior.is_ccw

        # find the most bottom and then left coordinate and rotate the coords by it
        ob_points = ob_poly.exterior.coords[:-1]
        pos = 0
        for idx, item in enumerate(ob_points):
            if item[1] < ob_points[pos][1] or (item[1] == ob_points[pos][1] and item[0] < ob_points[pos][0]):
                pos = idx
        #print(ob_points[:])
        ob_points = ob_points[pos:] + ob_points[:pos]
        #print(ob_points[:])

        # we append first vertices for being cyclic
        r_poly_points.append(r_poly_points[0])
        r_poly_points.append(r_poly_points[1])
        ob_points.append(ob_points[0])
        ob_points.append(ob_points[1])

        out_points = []
        i = 0
        j = 0
        while i < len(r_poly_points)-2 or j < len(ob_points)-2:
            out_points.append(tuple(sum(x) for x in zip(r_poly_points[i], ob_points[j])))
            p = tuple(map(lambda a, b: a - b, r_poly_points[i+1], r_poly_points[i]))
            q = tuple(map(lambda a, b: a - b, ob_points[j+1], ob_points[j]))
            cross_prod = p[0] * q[1] - p[1] * q[0]
            if cross_prod >= 0:
                i += 1
            if cross_prod <= 0:
                j += 1
        ret_poly = Polygon(out_points)

    return ret_poly


# TODO
def is_visible_edge(obstacles: List[Polygon], candidate_edge):
    for obstacle in obstacles:
        if obstacle.intersects(candidate_edge):
            if not obstacle.touches(candidate_edge):
                return False
    return True


class VisibilityGraphVertex:
    def __init__(self, coords=None):
        self.coords = coords
        self.adj = []

    def add_neighbor(self,node):
        self.adj.append(node)

    def get_coords(self):
        return self.coords


class VisibilityGraph:

    def __init__(self, obstacles: List[Polygon], source=None, dest=None):
        self.vertices = []
        self.edges = []
        self.src = None
        self.dest = None
        self.num_of_vertices = 0
        if source is not None:
            self.src = VisibilityGraphVertex(source)
            self.num_of_vertices += 1
        if dest is not None:
            self.dest = VisibilityGraphVertex(dest)
            self.num_of_vertices += 1
        for ob in obstacles:
            for coord in ob.exterior.coords:
                self.vertices.append(VisibilityGraphVertex(coord))
                self.num_of_vertices += 1
        n = len(self.vertices)
        adj_connect_mat = [[False for _ in range(n)] for _ in range(n)]
        for u_idx, u in enumerate(self.vertices):
            for v_idx, v in enumerate(self.vertices):
                if adj_connect_mat[v_idx][u_idx]:
                    adj_connect_mat[u_idx][v_idx] = True
                    continue
                if adj_connect_mat[u_idx][v_idx]:
                    adj_connect_mat[v_idx][u_idx] = True
                    continue
                candidate_edge = LineString([u.get_coords(), v.get_coords()])
                if is_visible_edge(obstacles, candidate_edge):
                    adj_connect_mat[u_idx][v_idx] = True
                    v.add_neighbor(u)
                    u.add_neighbor(v)
                    self.edges.append(candidate_edge)
        for _, v in enumerate(self.vertices):
            if source is not None:
                candidate_edge = LineString([self.src.get_coords(), v.get_coords()])
                if is_visible_edge(obstacles, candidate_edge):
                    self.src.add_neighbor(v)
                    v.add_neighbor(self.src)
                    self.edges.append(candidate_edge)
            if dest is not None:
                candidate_edge = LineString([v.get_coords(), self.dest.get_coords()])
                if is_visible_edge(obstacles, candidate_edge):
                    self.dest.add_neighbor(v)
                    v.add_neighbor(self.dest)
                    self.edges.append(candidate_edge)

    def get_edges(self):
        return self.edges


def get_visibility_graph(obstacles: List[Polygon], source=None, dest=None) -> List[LineString]:
    """
    Get The visibility graph of a given map
    :param obstacles: A list of the obstacles in the map
    :param source: The starting position of the robot. None for part 1.
    :param dest: The destination of the query. None for part 1.
    :return: A list of LineStrings holding the edges of the visibility graph
    """
    vg = VisibilityGraph(obstacles, source, dest)
    return vg.get_edges()


def vg_dijkstra(vg: VisibilityGraph) -> tuple[List, float]:
    """
    gets shortest path using dijkstra and this shortest path cost
    """

    # The priority queue
    frontier = []
    # The source has cost 0
    heapq.heappush(frontier, (0,0))

    # the visited nodes empty set
    visited = set()


    # TODO:
    shortest_path = []
    cost = 0
    return shortest_path, cost


def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("The file %s does not exist!" % arg)


def get_points_and_dist(line):
    source, dist = line.split(' ')
    dist = float(dist)
    source = tuple(map(float, source.split(',')))
    return source, dist


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("Robot", help="A file that holds the starting position of the robot, and the distance from the center of the robot to any of its vertices")
    parser.add_argument("Obstacles", help="A file that contains the obstacles in the map")
    parser.add_argument("Query", help="A file that contains the ending position for the robot.")
    args = parser.parse_args()
    obstacles = args.Obstacles
    robot = args.Robot
    query = args.Query
    is_valid_file(parser, obstacles)
    is_valid_file(parser, robot)
    is_valid_file(parser, query)
    workspace_obstacles = []
    with open(obstacles, 'r') as f:
        for line in f.readlines():
            ob_vertices = line.split(' ')
            if ',' not in ob_vertices:
                ob_vertices = ob_vertices[:-1]
            points = [tuple(map(float, t.split(','))) for t in ob_vertices]
            workspace_obstacles.append(Polygon(points))
    with open(robot, 'r') as f:
        source, dist = get_points_and_dist(f.readline())

    # step 1:
    c_space_obstacles = [get_minkowsky_sum(p, dist) for p in workspace_obstacles]
    plotter1 = Plotter()

    plotter1.add_obstacles(workspace_obstacles)
    plotter1.add_c_space_obstacles(c_space_obstacles)
    plotter1.add_robot(source, dist)

    plotter1.show_graph()

    # step 2:

    lines = get_visibility_graph(c_space_obstacles)
    plotter2 = Plotter()

    plotter2.add_obstacles(workspace_obstacles)
    plotter2.add_c_space_obstacles(c_space_obstacles)
    plotter2.add_visibility_graph(lines)
    plotter2.add_robot(source, dist)

    plotter2.show_graph()
    exit(0)

    # step 3:
    with open(query, 'r') as f:
        dest = tuple(map(float, f.readline().split(',')))

    lines = get_visibility_graph(c_space_obstacles, source, dest)
    #TODO: fill in the next line
    shortest_path, cost = None, None

    plotter3 = Plotter()
    plotter3.add_robot(source, dist)
    plotter3.add_obstacles(workspace_obstacles)
    plotter3.add_robot(dest, dist)
    plotter3.add_visibility_graph(lines)
    plotter3.add_shorterst_path(list(shortest_path))


    plotter3.show_graph()