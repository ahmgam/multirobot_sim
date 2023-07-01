
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from rospy import loginfo
from rospy import ServiceProxy
# parameter
N_SAMPLE = 500  # number of sample_points
N_KNN = 10  # number of edge from one sampled point
MAX_EDGE_LEN = 30.0  # [m] Maximum edge length

show_animation = True



class Node:
    """
    Node class for dijkstra search
    """

    def __init__(self, pos, parent_index):
        self.position = pos
        self.f = 0.0
        self.g = 0.0
        self.h = 0.0
        self.parent = parent_index

    def __str__(self):
        return str(self.position[0]) + "," + str(self.position[1]) + "," +\
               str(self.f) + "," + str(self.parent)

    def __eq__(self, other):
        return self.position[0] == other.position[0] and self.position[1] == other.position[1] 


def prm_planning(start_x, start_y, goal_x, goal_y,
                 obstacle_x_list, obstacle_y_list, robot_radius, *, rng=None):
    """
    Run probabilistic road map planning
    :param start_x: start x position
    :param start_y: start y position
    :param goal_x: goal x position
    :param goal_y: goal y position
    :param obstacle_x_list: obstacle x positions
    :param obstacle_y_list: obstacle y positions
    :param robot_radius: robot radius
    :param rng: (Optional) Random generator
    :return:
    """
    obstacle_kd_tree = KDTree(np.vstack((obstacle_x_list, obstacle_y_list)).T)

    sample_x, sample_y = sample_points(start_x, start_y, goal_x, goal_y,
                                       robot_radius,
                                       obstacle_x_list, obstacle_y_list,
                                       obstacle_kd_tree, rng)
    if show_animation:
        plt.plot(sample_x, sample_y, ".b")

    road_map = generate_road_map(sample_x, sample_y,
                                 robot_radius, obstacle_kd_tree)

    rx, ry = dijkstra_planning(
        start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y)

    return rx, ry


def is_collision(sx, sy, gx, gy, rr, obstacle_kd_tree):
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.hypot(dx, dy)

    if d >= MAX_EDGE_LEN:
        return True

    D = rr
    n_step = round(d / D)

    for i in range(n_step):
        dist, _ = obstacle_kd_tree.query([x, y])
        if dist <= rr:
            return True  # collision
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    # goal point check
    dist, _ = obstacle_kd_tree.query([gx, gy])
    if dist <= rr:
        return True  # collision

    return False  # OK


def generate_road_map(sample_x, sample_y, rr, obstacle_kd_tree):
    """
    Road map generation
    sample_x: [m] x positions of sampled points
    sample_y: [m] y positions of sampled points
    robot_radius: Robot Radius[m]
    obstacle_kd_tree: KDTree object of obstacles
    """

    road_map = []
    n_sample = len(sample_x)
    sample_kd_tree = KDTree(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(n_sample), sample_x, sample_y):

        dists, indexes = sample_kd_tree.query([ix, iy], k=n_sample)
        edge_id = []

        for ii in range(1, len(indexes)):
            nx = sample_x[indexes[ii]]
            ny = sample_y[indexes[ii]]

            if not is_collision(ix, iy, nx, ny, rr, obstacle_kd_tree):
                edge_id.append(indexes[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    #  plot_road_map(road_map, sample_x, sample_y)

    return road_map


def dijkstra_planning(sx, sy, gx, gy, road_map, sample_x, sample_y):
    """
    s_x: start x position [m]
    s_y: start y position [m]
    goal_x: goal x position [m]
    goal_y: goal y position [m]
    obstacle_x_list: x position list of Obstacles [m]
    obstacle_y_list: y position list of Obstacles [m]
    robot_radius: robot radius [m]
    road_map: ??? [m]
    sample_x: ??? [m]
    sample_y: ??? [m]
    @return: Two lists of path coordinates ([x1, x2, ...], [y1, y2, ...]), empty list when no path was found
    """

    start_node = Node(sx, sy, 0.0, -1)
    goal_node = Node(gx, gy, 0.0, -1)

    open_set, closed_set = dict(), dict()
    open_set[len(road_map) - 2] = start_node

    path_found = True

    while True:
        if not open_set:
            print("Cannot find path")
            path_found = False
            break

        c_id = min(open_set, key=lambda o: open_set[o].cost)
        current = open_set[c_id]

        # show graph
        if show_animation and len(closed_set.keys()) % 2 == 0:
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(current.x, current.y, "xg")
            plt.pause(0.001)

        if c_id == (len(road_map) - 1):
            print("goal is found!")
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            break

        # Remove the item from the open set
        del open_set[c_id]
        # Add it to the closed set
        closed_set[c_id] = current

        # expand search grid based on motion model
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            dx = sample_x[n_id] - current.x
            dy = sample_y[n_id] - current.y
            d = math.hypot(dx, dy)
            node = Node(sample_x[n_id], sample_y[n_id],
                        current.cost + d, c_id)

            if n_id in closed_set:
                continue
            # Otherwise if it is already in the open set
            if n_id in open_set:
                if open_set[n_id].cost > node.cost:
                    open_set[n_id].cost = node.cost
                    open_set[n_id].parent_index = c_id
            else:
                open_set[n_id] = node

    if path_found is False:
        return [], []

    # generate final course
    rx, ry = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index

    return rx, ry


def plot_road_map(road_map, sample_x, sample_y):  # pragma: no cover

    for i, _ in enumerate(road_map):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]

            plt.plot([sample_x[i], sample_x[ind]],
                     [sample_y[i], sample_y[ind]], "-k")


def sample_points(sx, sy, gx, gy, rr, ox, oy, obstacle_kd_tree, rng):
    max_x = max(ox)
    max_y = max(oy)
    min_x = min(ox)
    min_y = min(oy)

    sample_x, sample_y = [], []

    if rng is None:
        rng = np.random.default_rng()

    while len(sample_x) <= N_SAMPLE:
        tx = (rng.random() * (max_x - min_x)) + min_x
        ty = (rng.random() * (max_y - min_y)) + min_y

        dist, index = obstacle_kd_tree.query([tx, ty])

        if dist >= rr:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    return sample_x, sample_y



def main(rng=None):
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    robot_size = 5.0  # [m]

    ox = []
    oy = []

    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "^r")
        plt.plot(gx, gy, "^c")
        plt.grid(True)
        plt.axis("equal")

    rx, ry = prm_planning(sx, sy, gx, gy, ox, oy, robot_size, rng=rng)

    assert rx, 'Cannot found path'

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()



class PRM:
    def __init__(self, start, occupGrid,gridInfo, robotSize=0.5, rng=None):
        self.start = start
        self.info = gridInfo
        self.grid = occupGrid
        self.robotSize = robotSize
        self.rng = rng

        self.sample_x = []
        self.sample_y = []
        self.road_map = []
        self.obstacleLists = self.getObstacleList()
        self.obstacle_kd_tree = KDTree(np.vstack(self.obstacleLists).T)
    
  

    def setGoal(self, goal):
        self.goal = goal

    def getObstacleList(self):
        #get obstacle list
        obstacleListX = []
        obstacleListY = []
        for i in range(len(self.grid)):
            for j in range(len(self.grid[0])):
                if self.grid[i][j] == 100:
                    obstacleListX.append(i)
                    obstacleListY.append(j)
        return obstacleListX, obstacleListY

    def samplePoints(self):
        max_x = max(self.obstacleLists[0])
        max_y = max(self.obstacleLists[1])
        min_x = min(self.obstacleLists[0])
        min_y = min(self.obstacleLists[1])

        sample_x, sample_y = [], []
        rng = np.random.default_rng()

        while len(sample_x) <= N_SAMPLE:
            tx = (rng.random() * (max_x - min_x)) + min_x
            ty = (rng.random() * (max_y - min_y)) + min_y

            dist, index = self.obstacle_kd_tree.query([tx, ty])

            if dist >= self.robotSize:
                sample_x.append(tx)
                sample_y.append(ty)

        sample_x.append(self.start[0])
        sample_y.append(self.start[1])
        sample_x.append(self.goal[0])
        sample_y.append(self.goal[1])

        return sample_x, sample_y
    def getRoadMap(self, sample_x, sample_y):
        road_map = []
        for (i, ix, iy) in zip(range(len(sample_x)), sample_x, sample_y):
            for (j, jx, jy) in zip(range(len(sample_x)), sample_x, sample_y):
                if i == j:
                    continue
                if self.isCollision(ix, iy, jx, jy):
                    continue
                road_map.append([i, j])
        return road_map

    def isCollision(self, x, y):
        for (ox, oy) in zip(self.obstacleLists[0], self.obstacleLists[1]):
            dx = ox - x
            dy = oy - y
            d = math.hypot(dx, dy)
            if d <= self.robotSize:
                return True
        return False

    def plan(self):

        sample_x, sample_y = self.samplePoints()
        

        road_map = self.getRoadMap(sample_x, sample_y)

        planner = AStar(road_map, self.start, self.robotSize)
        planner.setGoal(self.goal)
        planner.plan()
        self.path = planner.path

        return 


######################################
# A* algorithm
######################################
class AStar:
    def __init__(self, occupGrid,gridInfo, window=1,simplify=True):
        self.window = window
        self.info = gridInfo
        self.grid = occupGrid
        self.start = None
        self.goal = None
        self.open = []
        self.closed = []
        self.path = []
        self.current = None
        self.simple = simplify

    def setStart(self, start):
        loginfo("Setting start to: " + str(start))
        self.start =self.node(start)

    def setGoal(self, goal):
        loginfo("Setting goal to: " + str(goal))
        self.goal = self.node((goal[0],goal[1]))

    def node(self, position, parent=None):
        return Node(position, parent)

    def flash(self):
        self.open = []
        self.closed = []
        self.path = []
        self.current = None

    def plan(self):
        #flash the lists
        self.flash()
        # Add the start node
        self.open.append(self.start)
        # Loop until you find the end
        while len(self.open) > 0:
            
            # Get the current node
            self.current = self.open[0]
            current_index = 0
            for index, item in enumerate(self.open):
                if item.f < self.current.f:
                    self.current = item
                    current_index = index

            # Pop current off open list, add to closed list
            self.open.pop(current_index)
            self.closed.append(self.current)
            ref = self.closed.index(self.current)
            # Found the goal
            if self.current.position == self.goal.position :
                current = self.current
                while current.parent is not None:
                    self.path.append(current.position)
                    current = self.closed[current.parent]
                self.path = self.path[::-1]
                # simply path
                if self.simple:
                    self.path = rdp_simplify(self.path, 0.5)
                return 

            # Generate children
            children = []
            for new_position in [(0, -self.window), (0, self.window), (-self.window, 0), (self.window, 0), (-self.window, -self.window), (-self.window, self.window), (self.window, -self.window), (self.window, self.window)]:  # Adjacent squares

                # Get node position
                node_position = (
                    int(self.current.position[0]) + new_position[0], int(self.current.position[1]) + new_position[1])
                # Make sure within range
                if node_position[0] > (self.info["width"] - 1) or node_position[0] < 0 or node_position[1] > (self.info["height"]) or node_position[1] < 0:
                    continue
                # Make sure walkable terrain
                if self.grid[node_position[0]][node_position[1]] != 0:
                    continue
                # Create new node
                new_node = self.node(node_position, ref)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:
                # Child is on the closed list
                if child in self.closed:
                    continue
                # Create the f, g, and h values
                child.g = self.current.g + self.window
                child.h = ((child.position[0] - self.goal.position[0])
                            ** 2) + ((child.position[1] - self.goal.position[1]) ** 2)
                child.f = child.g + child.h

                # Child is already in the open list
                if child in self.open:
                    continue
                # Add the child to the open list
                self.open.append(child)
        
                

class RTT: 
    def __init__(self,occuGrid,gridInfo,incDist=1):
        self.start = None
        self.grid = occuGrid
        self.tree = []
        self.xMax = gridInfo['width']
        self.yMax = gridInfo['height']
        self.maxDist = np.sqrt(self.xMax**2 + self.yMax**2)
        self.verts = []
        self.path = []
        self.incDist = incDist
        self.goal=None

    def setGoal(self,goal):
        self.goal = goal
    
    def setStart(self,start):
        self.start = start
        self.tree=[start]


    def flush(self):
        self.tree = [self.start]
        self.verts = []
        self.path = []
    
    def plan(self):
        self.flush()
        cntr = 0
        keep_iterating = True
        goal_reached = False
        while keep_iterating:
            collision = True
            # check to see if it's a clear path to the goal
            if self.no_collision(*self.tree[-1], self.goal[0],self.goal[1], self.grid):
                collision = False
                goal_reached = True
                q_new = self.goal
                q_near = self.tree[-1]
            else:
                cntr+=1
            while collision:
                # get random point in plot
                q_rand = list((int(np.random.randint(0,self.xMax,1)), int(np.random.randint(0,self.yMax,1))))
                # find nearest vertex in the self.tree to the random point
                q_near = self.nearest_vertex(q_rand, self.tree, self.maxDist)
                # find point along the line from q_near to q_rand that is at most inc_dist away
                q_new = self.new_config(q_near, q_rand, self.incDist)
                # # check to see if the new point collides with a colored pixel
                if self.no_collision(*q_near, *q_new, self.grid):
                    collision = False

            self.tree.append(q_new)
            self.verts.append(q_near)
            self.verts.append(q_new)
            if goal_reached:
                keep_iterating = False

        print("Goal achieved in " + str(cntr) + " iterations.")
        q_curr = self.goal
        while q_curr != self.start:
            self.path.append(q_curr)
            self.path.append(self.verts[self.verts.index(q_curr) - 1])
            # make a plot and show RRT
            q_curr = self.path[-1]

        # simplify path
        self.path = rdp_simplify(self.path)
        print("Done")

    def no_collision(self,x0, y0, x1, y1, world):
        # check if q_new is in a colored pixel or if the point already exists in the tree
        if world[int(y1)][int(x1)] == 0 or (x0, y0) == (x1, y1):
            return False
        # check if the line to be drawn is vertical
        elif (x1 - x0) == 0:
            return self.plotLineVert(x0, y0, x1, y1, world)
        # check if the line to be drawn is horizontal
        elif (y1 - y0) == 0:
            return self.plotLineHorz(x0, y0, x1, y1, world)
        else:
            # check if slope is less than +/-45 degrees steep
            # (i.e. within Octants 0, 3, 4, and 7)
            if abs(y1 - y0) < abs(x1 - x0):
                # check if slope is within Octant 3 and 4.
                # if yes, reverse start and end points so
                # the slope is within Octant 0 and 7
                if x0 > x1:
                    return self.plotLineLow(x1, y1, x0, y0, world)
                else:
                    return self.plotLineLow(x0, y0, x1, y1, world)
            # slope must be steeper than +/- 45 degrees
            # (i.e. within Octants 1, 2, 5, and 6)
            else:
                # check if slope is within Octants 5 and 6.
                # if yes, reverse start and end points so
                # the slope is within Octants 1 and 2
                if y0 > y1:
                    return self.plotLineHigh(x1, y1, x0, y0, world)
                else:
                    return self.plotLineHigh(x0, y0, x1, y1, world)


    # draws lines using pixels for slopes in Octant 0 and 7
    def plotLineLow(self,x0, y0, x1, y1, world):
        dx = x1 - x0
        dy = y1 - y0
        # the next two lines allow the function to handle Octant 7
        yi = np.sign(dy)
        dy = dy * yi
        D = 2*dy - dx
        y = y0

        for x in range(x0, x1):
            if world[y, x] == 255:
                return False
            if D > 0:
                y = y + yi
                D = D - 2*dx
            D = D + 2*dy
        return True

    # draws lines using pixels for slopes in Octant 1 and 2
    def plotLineHigh(self,x0, y0, x1, y1, world):
        dx = x1 - x0
        dy = y1 - y0
        # the next two lines allow the function to handle Octant 2
        xi = np.sign(dx)
        dx = dx * xi
        D = 2*dx - dy
        x = x0

        for y in range(y0, y1):
            if world[y, x] == 255:
                return False
            if D > 0:
                x = x + xi
                D = D - 2*dy
            D = D + 2*dx
        return True

    # draws vertical lines using pixels
    def plotLineVert(self,x0, y0, x1, y1, world):
        dy = y1 - y0
        yi = np.sign(dy)
        y = y0
        while y != y1:
            if world[y, x0] == 255:
                return False
            y = y + yi
        return True

    # draws horizontal lines using pixels
    def plotLineHorz(self,x0, y0, x1, y1, world):
        dx = x1 - x0
        xi = np.sign(dx)
        x = x0
        while x != x1:
            if world[y0, x] == 255:
                return False
            x = x + xi
        return True

    def vacant_space(self,q_pnt, world):
        if world[q_pnt[1], q_pnt[0]] != 0:
            print("That point lies within an obstacle. Please choose a different one.")
            return False
        return True

    def distance(self,pnt1, pnt2):
        return np.sqrt((pnt2[0] - pnt1[0])**2 + (pnt2[1] - pnt1[1])**2)

    def nearest_vertex(self,q_rand, tree, max_dist):
        min_dist = max_dist
        q_near = None
        for v in tree:
            dist = self.distance(v, q_rand)
            if dist < min_dist:
                min_dist = dist
                q_near = v
        return q_near

    def new_config(self,q_near, q_rand, inc_dist):
        dist = np.sqrt((q_rand[0] - q_near[0])**2 + (q_rand[1] - q_near[1])**2)
        if dist <= inc_dist:
            return q_rand
        else:
            v = [q_rand[0] - q_near[0], q_rand[1] - q_near[1]]
            v_mag = np.sqrt(v[0]**2 + v[1]**2)
            v_unit = [v[0]/v_mag, v[1]/v_mag]
            q_new_x = q_near[0] + v_unit[0]*inc_dist
            q_new_y = q_near[1] + v_unit[1]*inc_dist
            return [int(round(q_new_x)), int(round(q_new_y))]

###########################################
# RDP Algorithm for Path Simplification
###########################################
def rdp_simplify(points, epsilon):
    # Check if the list is long enough to simplify
    if len(points) < 3:
        return points
    
    # Find the point with the maximum distance
    dmax = 0
    index = 0
    end = len(points) - 1
    
    for i in range(1, end):
        d = perpendicular_distance(points[i], points[0], points[end])
        if d > dmax:
            index = i
            dmax = d
    
    # Check if the maximum distance is greater than epsilon
    simplified = []
    
    if dmax > epsilon:
        # Recursive call to simplify each half
        rec_results1 = rdp_simplify(points[:index + 1], epsilon)
        rec_results2 = rdp_simplify(points[index:], epsilon)
        
        # Build the simplified list
        simplified.extend(rec_results1[:-1])
        simplified.extend(rec_results2)
    else:
        # The maximum distance is below epsilon
        simplified = [points[0], points[end]]
    
    return simplified

def perpendicular_distance(point, start, end):
    # Calculate the perpendicular distance between a point and a line segment
    x1, y1 = start
    x2, y2 = end
    x, y = point
    # Calculate the slope of the line segment
    if x2 == x1:
        slope = math.inf
    else:
        slope = (y2 - y1) / (x2 - x1)
    
    # Calculate the y-intercept of the line segment
    y_intercept = y1 - slope * x1
    
    # Calculate the perpendicular distance
    distance = abs(slope * x - y + y_intercept) / math.sqrt(slope ** 2 + 1)
    
    return distance


class TaskAllocationManager:
    def __init__(self,node_id):
        self.node_id = node_id
        self.robots = {}
        self.targets = {}
        self.tasks = {}
        self.records= {}
        self.paths = {}
        self.idle= {}
        self.last_id = 0
        #self.get_blockchain_records = ServiceProxy('get_blockchain_records')

    def sync_records(self):
        #get new records from blockchain service 
        records = self.get_blockchain_records()
        for record in records:
            self.process_record(record)
            
    def process_record(self,record):
        if record['meta']['table'] == 'states':
            self.robots[record['data']['node_id']] = record['data']
        if record['meta']['table'] == 'targets':
            self.targets[record['data']['node_id']] = record['data']
        if record['meta']['table'] == 'task_records':
            data = record['data']
            if record['record_type'] == 'commit':
                self.idle[data['node_id']] = False
            else:
                self.idle[data['node_id']] = True
            if data['task_id'] in self.tasks.keys():
                self.tasks[data['task_id']].append(data)
            else:
                self.tasks[data['task_id']] = [data]
            self.records[data['id']] = data
        if record['meta']['table'] == 'path':
            data = record['data']
            task_id = self.records[data['commit_id']]['task_id']
            if task_id in self.paths.keys():
                self.paths[task_id][data['commit_id']] = data
            else:
                self.paths[task_id] = {}
                self.records[task_id][data['commit_id']] = [data]
        self.last_id = record['meta']['id']

    def is_task_committed(self,task_id):
        #check all records in task
        task = self.tasks[task_id]
        req_uav = int(self.targets[task[0]['target_id']]['needed_uav'])
        req_ugv = int(self.targets[task[0]['target_id']]['needed_ugv'])
        committed_uav = 0
        committed_ugv = 0
        for record in task:
            if record['record_type'] == 'commit':
                if record['node_type'] == 'uav':
                    committed_uav += 1
                if record['node_type'] == 'ugv':
                    committed_ugv += 1
        if committed_ugv == req_ugv and committed_uav == req_uav:
            return True
        else:
            return False

    def is_task_planned(self,task_id):
        #get all paths records for a task
        paths = self.paths[task_id]
        req_uav = int(self.targets[task_id]['needed_uav'])
        req_ugv = int(self.targets[task_id]['needed_ugv'])
        planned_uav = {}
        planned_ugv = {}
        for commit_id,path in paths.items():
            if path['node_type'] == 'uav':
                planned_uav[commit_id] = path
            if path['node_type'] == 'ugv':
                planned_ugv[commit_id] = path

        if len(planned_uav.keys()) != req_uav or len(planned_ugv.keys()) != req_ugv:
            return False
        #check paths conflicts 
        return self.check_conflict([p["path_points"] for p in paths.values()]) and self.check_conflict([planned_uav[p]["path_points"] for p in planned_uav.keys()])
            
    def is_task_complete(self,task_id):
        #check all records in task
        task = self.tasks[task_id]
        req_robots = int(self.targets[task[0]['target_id']]['needed_uav']) + int(self.targets[task[0]['target_id']]['needed_ugv'])
        completed = 0
        for record in task:
            if record['record_type'] == 'complete':
                completed += 1
        if completed >= req_robots:
            return True
        else:
            return False

    def clear_task(self,task_id):
        #get task
        task = self.tasks[task_id]
        #get all commit ids 
        record_ids = task.keys()
        for record_id in record_ids:
            target_id = task[record_id]['target_id']
            if target_id in self.targets.keys():
                del self.targets[target_id]
            del self.records[record_id]
        del self.tasks[task_id]
        del self.paths[task_id]

    def get_target_best_candidates(self,task_id):
        pass

    def is_robot_idle(self,robot_id):
        return self.idle[robot_id]
    
    def add_target(self,target):
        pass

    def update_state(self,state):
        pass

    def commmit_to_target(self,robot_id,target):
        pass

    def get_robot_state(self,robot_id):
        pass

    def allocate_robots_to_target(self,task_id):
        pass

    def send_to_blockchain(self,message):
        pass

    def set_initial_path_to_target(self,target):
        pass

    def reset_path_to_target(self,robot_id,paths):
        pass

    def check_conflict(self,paths):
        pass



if __name__ == '__main__':

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 30.0  # [m]
    gy = 50.0  # [m]
    robot_size = 5.0  # [m]
    start = [sx, sy]
    goal = [gx, gy]

    ox = []
    oy = []

    grid = np.zeros((61, 61))
    for i in range(60):
        ox.append(i)
        oy.append(0.0)
        grid[i,0] = 100
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
        grid[60,i] = 100
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
        grid[i,60] = 100
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
        grid[0,i] = 100
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
        grid[20,i] = 100
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)
        grid[40,60-i] = 100

    gridInfo = {
        "width": 61,
        "height": 61,
    }
    astar = AStar(grid, gridInfo,simplify=True)
    astar.setStart(start)
    astar.setGoal(goal)
    astar.plan()
    #print start and goal
    print(f"start: {start}, goal: {goal}")
    print(astar.path)
    #convert path to numpy array
    path = np.array(astar.path)
    #print grid in matplotlib plot
    plt.imshow(grid, cmap='Greys', origin='lower')
    plt.plot(start[0], start[1], 'ro')
    plt.plot(goal[0], goal[1], 'ro')
    plt.plot(path[:,0], path[:,1], 'r-')
    plt.show()




   

