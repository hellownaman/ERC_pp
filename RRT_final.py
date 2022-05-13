from matplotlib import pyplot as plt
import random
import math
from shapely.geometry import Point, Polygon, LineString

class Shape:
    def __init__(self, vertex):
        self.vertex = vertex
        self.Polygon = Polygon(vertex)

class Nodes:
    def __init__(self, x, y, parent) -> None:
        self.x = x
        self.y = y
        self.parent = parent

def sample_envir(plane_limits, goal_box, i, i_check):
    min_x = 0
    min_y = 0
    max_x = plane_limits[0]
    max_y = plane_limits[0]

    if (i % i_check == 0):
        min_x, min_y, max_x, max_y = goal_box.Polygon.bounds

    point = [random.uniform(min_x, max_x), random.uniform(min_y, max_y)]

    return point

def isValidPoint(point, node, obstacles):
    is_valid = True
    
    for obj in obstacles:
        if(obj.Polygon.contains(Point(point[0], point[1]))):
            is_valid = False
            break

        for j in range(len(obj.vertex) - 1):
            line_a = LineString([point, (node.x, node.y)])
            line_b = LineString([(obj.vertex[j][0], obj.vertex[j][1]), (obj.vertex[j+1][0], obj.vertex[j+1][1])])
            if(line_a.intersects(line_b)):
                is_valid = False
                break

    return is_valid

def treeExtend(point, min_index, tree, goal_box):
    found_goal = False

    tree.append(Nodes(point[0], point[1], min_index))

    if(goal_box.Polygon.contains(Point(point[0], point[1]))):
        found_goal = True

    return found_goal

def RRT(start, goal, obstacle_list):
    plane_limits = (100, 100)
    fixed_distance = 2
    i_check = 5
    i = 0
    goal_reached = False

    obstacles = []
    for obst in obstacle_list:
        obst.append(obst[0])
        obstacles.append(Shape(obst))

    goal_box = Shape([(goal[0], goal[1]), (goal[0] - 1, goal[1]), (goal[0] - 1, goal[1] - 1), (goal[0], goal[1] - 1)])

    tree = []
    tree.append(Nodes(start[0], start[1], 0))

    while (not goal_reached):
        point = sample_envir(plane_limits, goal_box, i, i_check)
        i += 1

        min_dist = math.sqrt((point[0] - start[0]) ** 2 + (point[1] - start[1]) ** 2)
        min_index = 0

        for i in range(len(tree)):
            curr_dist = math.sqrt((point[0] - tree[i].x) ** 2 + (point[1] - tree[i].y) ** 2)
            if (curr_dist < min_dist):
                min_dist = curr_dist
                min_index = i

        if(min_dist > fixed_distance):
            point[0] = (((point[0] - tree[min_index].x) * fixed_distance)/min_dist) + tree[min_index].x
            point[1] =  (((point[1] - tree[min_index].y) * fixed_distance)/min_dist) + tree[min_index].y

        if (not isValidPoint(point, tree[min_index], obstacles)):
            continue

        found_goal = treeExtend(point, min_index, tree, goal_box)

        if (found_goal):
            treeExtend(goal, len(tree) - 1, tree, goal_box)

def visualize(tree, obstacle_list):
    # # Window Settings
    # plt.figure('RRT Path Planning', (1000 / float(96), 600 / float(96)))
    # plt.xlim(-10, 110)
    # plt.ylim(-10, 110)

    # # Creating Obstacles
    # # plt.scatter(start[0], start[1])
    # # plt.scatter(goal[0], goal[1])
    # for i in obstacle_list:
    #     plt.plot(*zip(*(i+i[:1])))    
    # pass

    #Drawing obstacles
    for obst in obstacle_list:
        obst.append(obst[0])
        xs, ys = zip(*obst)
        plt.plot(xs, ys)

    #Plotting and connecting nodes with respective parent nodes in tree
    for node in tree:
        plt.plot([node.x_coord, tree[node.parent_index].x_coord], [node.y_coord, tree[node.parent_index].y_coord], "r.-", markersize = 3, linewidth = 0.3)

    #Connecting Goal to the start point
    curr_index = len(tree) - 1
    while(curr_index != 0):
        parent_index = tree[curr_index].parent_index
        plt.plot([tree[curr_index].x_coord, tree[parent_index].x_coord], [tree[curr_index].y_coord, tree[parent_index].y_coord], 'b.-', markersize = 5, linewidth = 0.5)
        curr_index = parent_index

    plt.show()

def main():

    #initial data
    start = (1, 1)
    goal = (100, 1)
    obstacle_list = [
        [(40, 0), (40, 40), (50, 50), (60, 40), (50, 40)],
        [(10, 10), (20, 20), (10, 30), (0, 20)],
        [(50, 60), (70, 80), (60, 100), (40, 80), (45, 100)],
        [(70, 20), (90, 20), (80, 40)]
    ]

    # Calculate Path using RRT
    path = RRT(start, goal, obstacle_list)

    # Visualize the calculated path in Matplotlib
    visualize(path, obstacle_list)
        
    plt.show()

if __name__ == '__main__':
    main()