import argparse
import os
from typing import List, Tuple

import shapely.geometry
import shapely.affinity

from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString


# TODO
def get_minkowsky_sum(original_shape: Polygon, r: float) -> Polygon:
    """
    Get the polygon representing the Minkowsky sum
    :param original_shape: The original obstacle
    :param r: The radius of the rhombus
    :return: The polygon composed from the Minkowsky sums
    """
    convex_hull_method = True
    if convex_hull_method:
        num_of_points = len(original_shape.exterior.coords) - 1
        total_points = num_of_points * 4 * [0]
        for idx in range(num_of_points):
            idx_offset = idx*4
            curr_obs_point = original_shape.exterior.coords[idx]
            total_points[idx_offset] = tuple(sum(x) for x in zip((r, 0), curr_obs_point))
            total_points[idx_offset + 1] = tuple(sum(x) for x in zip((0, r), curr_obs_point))
            total_points[idx_offset + 2] = tuple(sum(x) for x in zip((-r, 0), curr_obs_point))
            total_points[idx_offset + 3] = tuple(sum(x) for x in zip((0, -r), curr_obs_point))
        #print(total_points[:])
        ret_poly = Polygon(total_points).convex_hull
    else:
        #r_poly = [(0, 0), (r, r), (0, 2*r), (-r, r)]
        r_points = [(r, r), (0, 0), (0, 2 * r), (-r, r)]
        r_poly = Polygon(r_points)
        r_bb_bottom_left_point = shapely.geometry.Point(r_poly.bounds[:2])
        #print(r_bb_bottom_left_point)
        ref_point = min(r_poly.exterior.coords, key=lambda x: shapely.geometry.Point(x).distance(r_bb_bottom_left_point))
        #print(ref_point)
        r_trans = shapely.affinity.translate(r_poly, xoff=-ref_point[0], yoff=-ref_point[1])
        #print(r_trans)
        #print(r_poly.exterior.coords[:])
        r_poly = shapely.geometry.polygon.orient(r_poly, sign=-1.0)
        #print(r_poly.exterior.coords[:])
        #print(original_shape.exterior.coords[:])
        ccw_poly = shapely.geometry.polygon.orient(original_shape)
        ret_poly = original_shape

    return ret_poly


# TODO
def get_visibility_graph(obstacles: List[Polygon], source=None, dest=None) -> List[LineString]:
    """
    Get The visibility graph of a given map
    :param obstacles: A list of the obstacles in the map
    :param source: The starting position of the robot. None for part 1.
    :param dest: The destination of the query. None for part 1.
    :return: A list of LineStrings holding the edges of the visibility graph
    """
    pass


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
    exit(0)
    # step 2:

    lines = get_visibility_graph(c_space_obstacles)
    plotter2 = Plotter()

    plotter2.add_obstacles(workspace_obstacles)
    plotter2.add_c_space_obstacles(c_space_obstacles)
    plotter2.add_visibility_graph(lines)
    plotter2.add_robot(source, dist)

    plotter2.show_graph()

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