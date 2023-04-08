# rrt straight line path planner for mavsim_python
#
# mavsim_python
#     - Beard & McLain, PUP, 2012
#     - Last updated:
#         4/3/2019 - Brady Moon
#         4/11/2019 - RWB
#         3/31/2020 - RWB
import numpy as np
from message_types.msg_waypoints import MsgWaypoints
from viewers.planner_viewer import PlannerViewer

class RRTStraightLine:
    def __init__(self, app, show_planner=True):
        
        self.segment_length = 350 # standard length of path segments
        self.show_planner = show_planner
        if show_planner:
            self.planner_viewer = PlannerViewer(app)

    def update(self, start_pose, end_pose, Va, world_map, radius):

        
        
        sp = MsgWaypoints()
        sp.add(start_pose, Va)
        ep = MsgWaypoints()
        ep.add(end_pose, Va)
        
        self.segment_length = 4 * radius
        tree = MsgWaypoints()
        tree.type = 'fillet'

        ##### TODO #####
        # add the start pose to the tree
        tree.add(sp.ned, 
                 Va)
        # check to see if start_pose connects directly to end_pose

        # find path with minimum cost to end_node
        
        while int(tree.connect_to_goal[-1]) != 1:
            if self.extend_tree(tree, end_pose, Va, world_map):
                # print("successful addition")
                pass
            else:
                # print("extend failure")
                pass
            # print(distance(column(tree.ned, -1), end_pose[0:2]))
            
        waypoints_not_smooth = findMinimumPath(tree, end_pose)
        waypoints = smoothPath(waypoints_not_smooth, world_map)
        # waypoints_not_smooth = MsgWaypoints()
        # waypoints = MsgWaypoints()
        if self.show_planner:
            self.planner_viewer.draw_tree_and_map(world_map, tree, waypoints_not_smooth, waypoints, radius)
        return waypoints

    def extend_tree(self, tree, end_pose, Va, world_map):
        rp = randomPose(world_map, column(tree.ned, 0)[2])
        score = []
        index = []
        
        flag = None
        # print(tree.ned.shape[1])
        for j in range(tree.ned.shape[1]):
            base = column(tree.ned, j)
            dir = (rp - base)
            distanceVal = np.linalg.norm(rp - base)
            dir = dir / np.linalg.norm(dir) * self.segment_length
            new_point = base + dir 
            
            if not collision(base, new_point, world_map):
                score.append(distance(base, new_point)) # + tree.cost[j]
                index.append(j)

        
        if len(score) > 0:
            print(score)
            score_val      = np.min(score)
            parent_temp    = np.argmin(score)
            parent_index   = index[parent_temp]
            base = column(tree.ned, parent_index)
            dir = (rp - base)
            dir = dir / np.linalg.norm(dir) * self.segment_length
            new_point = base + dir 
            # print(score, score_val, score[parent_temp])
            # print(index, index[parent_temp])
            
            connects_to_end = (int( not (collision(new_point, end_pose, world_map))))
            # print("RP = ", rp[0:2])
            # print("EP = ", end_pose[0:2])
            # print("Collision = ", collision(rp[0:2], end_pose[0:2], world_map, radius))
        
        
            if collision(column(tree.ned, parent_index), new_point,world_map):
                flag = False
                
            if flag != False:

                tree.add(new_point, Va, 0. ,score_val + tree.cost[parent_index], parent_index, connects_to_end)
                flag = True
        else:
            flag = False
        
        return flag
        
    def process_app(self):
        self.planner_viewer.process_app()

def smoothPath(waypoints, world_map):

    ##### TODO #####
    # smooth the waypoint path
    smooth = [0]  # add the first waypoint
    i = 0
    j = 1
    print("NUM_WAYPOINTS = ", waypoints.num_waypoints)
    print("WAYPOINT = \n", waypoints.ned)
    while j < waypoints.num_waypoints - 1:
        print(f'i={i}, j={j+1}')
        ws = column(waypoints.ned, i)
        wp = column(waypoints.ned, j+1)
        if collision(ws, column(waypoints.ned, -1), world_map) == False:
            smooth.append(i)
            break
        if collision(ws, wp, world_map, print_flag=False) == True:
            smooth.append(j)
            i = j
            
        j = j + 1
    smooth.append(-1)
    print("SMOOTH = ", smooth)
    # construct smooth waypoint path
    smooth_waypoints = MsgWaypoints()
    smooth_waypoints.type = 'fillet'
    
    # for i in range(len(smooth)-2):
    #     print("SMOOTH COL = ",collision(column(waypoints.ned, smooth[i]), column(waypoints.ned, smooth[i+2]), world_map))
    

    for w in range(len(smooth)):
        
        angle = 0
        if w == 0:
            parent = 0
        else:
            parent = smooth[w-1]
            
        smooth_waypoints.add(column(waypoints.ned, smooth[w]), 
                                waypoints.airspeed[smooth[w]], 
                                0,
                                np.inf,
                                0,
                                0)

    return smooth_waypoints


def findMinimumPath(tree, end_pose):
    # find the lowest cost path to the end node

    ##### TODO #####
    # find nodes that connect to end_node
    connecting_nodes = []
    
    # find minimum cost last node
    idx = 0
    
    # construct lowest cost path order
    path = []

    i = tree.ned.shape[1] - 1
    cost = [0.]
    
    while i > 0:
        path = [i] + path
        cost = [tree.cost[i]] + cost
        i = int(tree.parent[i])    
    path = [0] + path
    print("Basic Path = ", path)

    # construct waypoint path
    waypoints = MsgWaypoints()
    waypoints.type = 'fillet'
    # waypoints.add(column(tree.ned, 0), tree.airspeed[0], 0., cost[0], 0, tree.connect_to_goal[0])
    for i in range(0, len(path)):
        waypoints.add(column(tree.ned, path[i]), tree.airspeed[path[i]], 0., 0., 0., 0.)
    
    waypoints.add(end_pose, 25., 0., 0., 0, 0)
    
    print("COMPLETED")
    return waypoints


def randomPose(world_map, pd):
    # generate a random pose

    ##### TODO #####
    pn   = np.abs(np.random.randn()) * world_map.city_width
    pe   = np.abs(np.random.randn()) * world_map.city_width
    pose = np.array([[pn], [pe], pd])
    return pose


def distance(start_pose, end_pose):
    # compute distance between start and end pose

    ##### TODO #####
    d = np.linalg.norm(end_pose - start_pose)
    return d


def collision(start_pose, end_pose, world_map, print_flag=False):
    # check to see of path from start_pose to end_pose colliding with map
    sp = np.copy(start_pose)
    ep = np.copy(end_pose)
    
    ###### TODO ######
    collision_flag = False
    line = (ep - sp)
    # line = line / np.linalg.norm(line)
    
    for i in range(world_map.num_city_blocks):
        for j in range(world_map.num_city_blocks):
            point = np.array([[world_map.building_north[0, j],
                                world_map.building_east[0, i],
                                0.]]).T
            
            point_proj = sp + line * (point - sp).T @ line / (line.T @ line) 
            # print((point - sp).T @ line / (line.T @ line) )
            dist_point = point_proj - point
            
            if  np.abs(dist_point[0,0]) <= world_map.building_width  and \
                np.abs(dist_point[1,0]) <= world_map.building_width  and \
                np.abs(dist_point[2,0]) <= world_map.building_height[j,i] and \
                    (point - sp).T @ line / (line.T @ line) > 0 and \
                    (point - sp).T @ line / (line.T @ line)  < 1.0:
                collision_flag = True
                
                if print_flag: 
                    print(start_pose)
                    print(point)
                    print(dist_point)
                    print(f"COLLISION at n={j}, e={i} h={world_map.building_height[j,i]}")
                
                # print( f"point_proj = {dist_point}")
    return collision_flag


def height_above_ground(world_map, point):
    # find the altitude of point above ground level
    
    ##### TODO #####
    h_agl = 0
    return h_agl

def points_along_path(start_pose, end_pose, N):
    # returns points along path separated by Del
    points = None
    return points


def column(A, i):
    # extracts the ith column of A and return column vector
    tmp = A[:, i]
    col = tmp.reshape(A.shape[0], 1)
    return col