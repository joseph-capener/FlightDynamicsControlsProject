# rrt dubins path planner for mavsim_python
#
# mavsim_python
#     - Beard & McLain, PUP, 2012
#     - Last updated:
#         4/16/2019 - RWB
import numpy as np
from message_types.msg_waypoints import MsgWaypoints
from viewers.draw_waypoints import DrawWaypoints
from viewers.draw_map import DrawMap
from planning.dubins_parameters import DubinsParameters
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from viewers.planner_viewer import PlannerViewer


class RRTDubins:
    def __init__(self, app, show_planner=True):
        self.segment_length = 450  # standard length of path segments
        self.dubins_path = DubinsParameters()
        # initialize Qt gui application and window
        self.show_planner = show_planner
        if show_planner:
            self.planner_viewer = PlannerViewer(app)

    def update(self, start_pose, end_pose, Va, world_map, radius):
        
        sp = MsgWaypoints()
        sp.add(start_pose[0:3], Va, start_pose[3])
        ep = MsgWaypoints()
        ep.add(end_pose[0:3], Va, end_pose[3])
        
        self.segment_length = 4 * radius
        tree = MsgWaypoints()
        tree.type = 'dubins'

        ##### TODO #####
        # add the start pose to the tree
        tree.add(sp.ned, 
                 Va, 
                 sp.course)
        # check to see if start_pose connects directly to end_pose
        if not self.collision(sp.ned, ep.ned, world_map, radius):
            
            waypoints = MsgWaypoints()
            waypoints.type = 'dubins'
            waypoints.add(sp.ned, Va, sp.course, 0,0,0)
            waypoints.add(ep.ned, Va, ep.course, 0,0,0)
            if self.show_planner:
                self.planner_viewer.draw_tree_and_map(world_map, tree, waypoints, waypoints, radius, self.dubins_path)
            return waypoints
       
        # find path with minimum cost to end_node
        
        while int(tree.connect_to_goal[-1]) != 1:
            if self.extendTree(tree, end_pose, Va, world_map, self.segment_length):
                # print("successful addition")
                pass
            else:
                # print("extend failure")
                pass
            # print(distance(column(tree.ned, -1), end_pose[0:3]))
            
        
            
        waypoints_not_smooth = findMinimumPath(tree, end_pose)
        waypoints = self.smoothPath(waypoints_not_smooth, world_map, self.segment_length)
        # waypoints_not_smooth = MsgWaypoints()
        # waypoints = MsgWaypoints()
        if self.show_planner:
            self.planner_viewer.draw_tree_and_map(world_map, tree, waypoints_not_smooth, waypoints, radius, self.dubins_path)
        return waypoints

    def extendTree(self, tree: MsgWaypoints, end_pose, Va, world_map, radius):
        # extend tree by randomly selecting pose and extending tree toward that pose
        
        rp = randomPose(world_map, column(tree.ned, 0)[2])
        
        score = []
        index = []
        
        flag = None
        print(tree.ned.shape[1])
        for j in range(tree.ned.shape[1]):
            base = column(tree.ned,j)
            new_point = base + ((rp[0:3] - base) / np.linalg.norm(rp[0:3] - base)) * 10 #np.max([radius, np.linalg.norm(rp[0:3] - base)])
            
            if not self.collision(column(tree.ned,j), new_point, world_map, radius):
                score.append(distance(column(tree.ned,j), new_point ) + tree.cost[j])
                index.append(j)
        
        if len(score) > 0:
            score_val      = np.min(score)
            parent_temp    = np.argmin(score)
            parent_index   = index[parent_temp]
            # print(score, score_val, score[parent_temp])
            # print(index, index[parent_temp])
            
            connects_to_end = (int( not (self.collision(rp[0:3], end_pose[0:3], world_map, radius))))
            # print("RP = ", rp[0:3])
            # print("EP = ", end_pose[0:3])
            # print("Collision = ", self.collision(rp[0:3], end_pose[0:3], world_map, radius))
        
        
            if self.collision(column(tree.ned, parent_index), rp[0:3],world_map, radius):
                flag = False
            if distance(column(tree.ned, parent_index), rp[0:3]) <= radius / 2 :
                flag = False
            if flag != False:
                tree.add(rp[0:3], Va, rp[3],score_val, parent_index, connects_to_end)
                flag = True
        else:
            flag = False
        
        return flag

    def collision(self, start_pose, end_pose, world_map, radius, print_option=False):
        # check to see of path from start_pose to end_pose colliding with world_map
        
        line = (end_pose - start_pose) / np.linalg.norm(end_pose - start_pose)
        collision_flag = False
        for i in range(world_map.num_city_blocks):
            for j in range(world_map.num_city_blocks):
                point = np.array([[world_map.building_north[0, j],
                                   world_map.building_east[0, i],
                                   0.]]).T
                point_proj = start_pose + ((point - start_pose).T @ line) / (line.T @ line) * line
                dist_point = point_proj - point
                
                if np.abs(dist_point[0,0]) <= world_map.building_width*2 and \
                   np.abs(dist_point[1,0]) <= world_map.building_width*2 and \
                   np.abs(dist_point[2,0]) <= world_map.building_height[i,j] :
                    collision_flag = True
                    if print_option: print(dist_point)

                
                
        ##### TODO #####
        return collision_flag

    def process_app(self):
        self.planner_viewer.process_app()

    def smoothPath(self, waypoints, world_map, radius):
        
        ##### TODO #####
        # smooth the waypoint path
        smooth = [0]  # add the first waypoint
        i = 0
        j = 1
        print("NUM_WAYPOINTS = ", waypoints.num_waypoints)
        while j < waypoints.num_waypoints - 1:
            ws = column(waypoints.ned, i)
            wp = column(waypoints.ned, j+1)
            if self.collision(ws, wp, world_map, radius) == True:
                smooth.append(j)
                print("abriding to ", j)
                i = j
                
            j = j + 1
        smooth.append(-1)
        print("SMOOTH = ", smooth)
        # construct smooth waypoint path
        smooth_waypoints = MsgWaypoints()
        smooth_waypoints.type = 'dubins'
        
        for i in range(len(smooth)-1):
            self.collision(column(waypoints.ned, smooth[i]), column(waypoints.ned, smooth[i+1]), world_map, radius, print_option=True)
        

        for w in range(len(smooth)):
            
            angle = 0
            if w == 0:
                parent = 0
            else:
                parent = smooth[w-1]
            if w != len(smooth)-1: 
                angle = np.arccos(np.array([[1, 0, 0]]) @ (column(waypoints.ned, smooth[w+1]) - column(waypoints.ned, smooth[w])) / np.linalg.norm((column(waypoints.ned, smooth[w+1]) - column(waypoints.ned, smooth[w]))))
            
            smooth_waypoints.add(column(waypoints.ned, smooth[w]), 
                                 waypoints.airspeed[smooth[w]], 
                                 angle,
                                 np.inf,
                                 parent,
                                 0)
        
        
        return smooth_waypoints


def findMinimumPath(tree: MsgWaypoints, end_pose: MsgWaypoints):
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


    
    # construct waypoint path
    waypoints = MsgWaypoints()
    waypoints.type = 'dubins'
    waypoints.add(column(tree.ned, 0), tree.airspeed[0], tree.course[0], cost[0], 0, tree.connect_to_goal[0])
    for i in range(0, len(path)):
        waypoints.add(column(tree.ned, path[i]), tree.airspeed[path[i]], tree.course[path[i]], cost[i], path[i-1], tree.connect_to_goal[i])
    
    waypoints.add(end_pose[0:3], 25., end_pose[3], 0., 0, 0)
    print("COMPLETED")
    return waypoints


def distance(start_pose, end_pose):
    # compute distance between start and end pose
    ##### TODO #####
    d = np.linalg.norm(end_pose[0:3] - start_pose[0:3])
    return d


def heightAboveGround(world_map, point):
    # find the altitude of point above ground level
    ###### TODO ######
    h_agl = 0
    return h_agl


def randomPose(world_map, pd):
    # generate a random pose
    ###### TODO ######
    
    pn   = np.random.randint(0, world_map.city_width)
    pe   = np.random.randint(0, world_map.city_width)
    chi  = mod(np.random.randn() * 2*np.pi)
    pose = np.array([[pn], [pe], pd, [chi]])
    
    return pose


def mod(x):
    # force x to be between 0 and 2*pi
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi:
        x -= 2*np.pi
    return x


def column(A, i):
    # extracts the ith column of A and return column vector
    tmp = A[:, i]
    col = tmp.reshape(A.shape[0], 1)
    return col
