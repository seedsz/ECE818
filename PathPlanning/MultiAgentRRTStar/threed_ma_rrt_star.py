"""
Path planning Sample Code with RRT*
author: Atsushi Sakai(@Atsushi_twi)
"""

import math
import sys
import matplotlib.pyplot as plt
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from RRT.threed_rrt import RRT

show_animation = True



class MultiAgentSim:
    '''
    Class for simulating the multi-agent algorithm
    '''

    class MovingObstacle:
        def __init__(self, radius = 1, d_path = None, ):
            self.radius = radius
            self.d_path = d_path
            self.tstep = d_path[1][3]-d_path[0][3] #DONE: changed index to 3 (new t)

    def __init__(self, #multi-agent simulation
                 RRT_list=None, #list of all cooperating RRTs/robots
                 safety_buffer = .1):
        self.RRT_list = RRT_list        #TODO referenced rrt
        self.conflict_index_list = [] #list of indexes of all paths/robots in conflict
        self.safety_buffer = safety_buffer

    def print_test(self):
        for bot in self.RRT_list:
            print(bot.speed)
            
    def set_ax(self):           #Added this for graphing so that there is only one instance of graph
        fig = plt.figure()
        self.ax = fig.add_subplot(projection='3d')
        self.ax.set_aspect('equal')  
        self.ax.set_xlim3d(self.min_rand, self.max_rand)
        self.ax.set_ylim3d(self.min_rand, self.max_rand) 
        self.ax.set_zlim3d(self.min_rand, self.max_rand)
        self.ax.grid(True)
        
    def check_paths(self):
        '''
        Check the planned paths of each robot to identify collisions.
        Because the control is decentralized, each robot must individually check the other paths.
        Safety buffer is the distance that each robot must maintain from another.
        '''
        primary_bot_index = 0
        conflict_list = []
        while primary_bot_index<len(self.RRT_list):
            secondary_bot_index = 0
            if primary_bot_index not in conflict_list:
                while secondary_bot_index<len(self.RRT_list):   #TODO rrt referenced
                    if secondary_bot_index != primary_bot_index:
                        path_index = 0
                        dpath1 = self.RRT_list[primary_bot_index].d_path
                        dpath2 = self.RRT_list[secondary_bot_index].d_path
                        while path_index<len(self.RRT_list[primary_bot_index].d_path) and path_index<len(self.RRT_list[secondary_bot_index].d_path):
                            d = math.hypot(dpath1[path_index][0]-dpath2[path_index][0],dpath1[path_index][1]-dpath2[path_index][1],dpath1[path_index][2]-dpath2[path_index][2]) #DONE: Changed to hypot(x,y,z)
                            if d <= self.RRT_list[primary_bot_index].robot_radius + self.RRT_list[secondary_bot_index].robot_radius + self.safety_buffer:
                                if primary_bot_index not in conflict_list:
                                    conflict_list.append(primary_bot_index)
                                if secondary_bot_index not in conflict_list:
                                    conflict_list.append(secondary_bot_index)
                                print("conflict detected between robots ", primary_bot_index, " and ", secondary_bot_index," at (x,y,z,t): ", self.RRT_list[secondary_bot_index].d_path[path_index], self.RRT_list[primary_bot_index].d_path[path_index]) #fix later
                                path_index = len(self.RRT_list[primary_bot_index].d_path) #break out of this part of loop once conflict is found
                            path_index = path_index+1
                    secondary_bot_index = secondary_bot_index+1
            primary_bot_index=primary_bot_index+1
        if conflict_list:
            print("Conflict(s) found. Paths must be replanned.")
        else: 
            print("No conflicts found. All paths have been finalized")
        self.conflict_index_list = conflict_list
        return conflict_list

    def conflict_resolution(self):
        moving_ob_list = []
        New_RRT_List = [] #store index of RRT and the new RRT

        for bot in self.RRT_list: 
            moving_ob = self.MovingObstacle(bot.robot_radius+self.safety_buffer,bot.d_path) #add a safety buffer to ensure no collision
            moving_ob_list.append(moving_ob)
        for i in self.conflict_index_list: #create new RRT with moving obstacles for each conflicted path
            spec_moving_obstacle_list = moving_ob_list[:i] + moving_ob_list[i+1:] #do not compare a path with itself
            old_rrt = self.RRT_list[i]  #TODO rrt referenced
            rrt_star = RRTStar(
            start = old_rrt.start_coords,
            goal = old_rrt.goal_coords,
            rand_area=old_rrt.rand_area,
            obstacle_list=old_rrt.obstacle_list,
            expand_dis=old_rrt.expand_dis,
            robot_radius=old_rrt.robot_radius,
            search_until_max_iter=True,
            moving_obstacle_list=spec_moving_obstacle_list,
            name = old_rrt.name,
            rewire_on=False) #do not allow rewiring, as this can cause unforeseen collision
            (path, d_path) = rrt_star.path_creation()
            delta_cost = path[-1][3]-self.RRT_list[i].path[-1][3] #DONE: changed t index to 3 to accomodate z
            New_RRT_List.append((i, rrt_star, self.RRT_list[i], delta_cost)) #tuple is (index, new, old, cost)
        lowest_cost_tuple = New_RRT_List[0]
        for tup in New_RRT_List:
            if tup[-1] < lowest_cost_tuple[-1]:
                lowest_cost_tuple = tup
        self.RRT_list[lowest_cost_tuple[0]] = lowest_cost_tuple[1] #replaces RRT star with lowest difference in costs
        print("Robot ", lowest_cost_tuple[0], "'s path has been modified to avoid conflict. ", "The new path is ", round(lowest_cost_tuple[3],2), " seconds slower.")
class RRTStar(RRT):     #TODO instance of rrt
    """
    Class for RRT Star planning
    """

    class Node(RRT.Node):   #TODO instance of rrt
        def __init__(self, x, y, z, t = None): #DONE: added z
            super().__init__(x, y, z) 
            self.cost = 0.0 #cost = time in this sim

    def __init__(self,
                 start, 
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=.1,
                 path_resolution=.1,
                 goal_sample_rate=95,
                 max_iter=1000,
                 connect_circle_dist=50.0,
                 search_until_max_iter=True,
                 robot_radius=0.0,
                 speed = 1,
                 moving_obstacle_list = [],
                 rewire_on = True,
                 name = "Unnamed Robot"): 
        """
        Setting Parameter
        start:Start Position [x,y,z]
        goal:Goal Position [x,y,z]
        obstacleList:obstacle Positions [[x,y,z,size],...]
        randArea:Random Sampling Area [min,max]
        """
        super().__init__(start, goal, obstacle_list, rand_area, expand_dis,
                         path_resolution, goal_sample_rate, max_iter,
                         robot_radius=robot_radius)
        self.connect_circle_dist = connect_circle_dist
        self.goal_coords = goal
        self.rand_area = rand_area
        self.start_coords = start
        self.goal_node = self.Node(goal[0], goal[1], goal[2]) # DONE: added z
        self.search_until_max_iter = search_until_max_iter
        self.node_list = [] 
        self.speed = speed
        self.path = None #no path to start
        self.d_path = []
        self.moving_obstacle_list = moving_obstacle_list
        self.name = name
        self.rewire_on = rewire_on #used to decide whether rewiring is allowed

    def planning(self, animation=True):
        """
        rrt star path planning
        animation: flag for animation on or off .
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            #print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node() # use get_random from 3DRRT
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd, #DONE: get steer from 3DRRT
                                  self.expand_dis)
            near_node = self.node_list[nearest_ind]
            new_node.cost = near_node.cost + \
                math.hypot(new_node.x-near_node.x,
                           new_node.y-near_node.y,
                           new_node.z-near_node.z,)/self.speed #DONE: added z

            if self.check_collision( 
                    new_node, self.obstacle_list, self.robot_radius):
                near_inds = self.find_near_nodes(new_node)
                node_with_updated_parent = self.choose_parent(
                    new_node, near_inds)
                if node_with_updated_parent:
                    if self.rewire_on: #only rewire if rewire is allowed
                        self.rewire(node_with_updated_parent, near_inds)
                    self.node_list.append(node_with_updated_parent)
                else:
                    self.node_list.append(new_node)

            if animation:
                self.draw_graph(rnd) #TODO: get draw_graph from 3DRRT

            if ((not self.search_until_max_iter)
                    and new_node):  # if reaches goal
                last_index = self.search_best_goal_node()
                if last_index is not None:
                    self.path = self.generate_final_course(last_index)
                    return self.path


        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index is not None:
            self.path = self.generate_final_course(last_index)
            return self.path

        return None
    
    def path_creation(self, timestep=.1):
        path = self.planning(animation=False)   #TODO 
        if path is None:
            print("Cannot find path")
            sys.exit() #if a path cannot be found, notify the user and end the program
        else:
            print("found path for",self.name)
            print("[x, y, z, t]")
            for spot in path:
                print(spot)
            d_path =self.discretize_path(path, timestep)
            self.d_path = d_path
            return (path, d_path)

    def generate_final_course(self, goal_ind): #calculates final path once goal is reached
        path = [[self.end.x, self.end.y, self.end.z]] #DONE: added z here (zac)
        node = self.node_list[goal_ind]
    
        while node.parent is not None: #go backwards from goal and find point that spawned it
            path.append([node.x, node.y, node.z]) #DONE: added z here (zac)
            node = node.parent
        path.append([node.x, node.y, node.z]) #DONE: add z here (zac) origin has no parent, it is the last point.
        i = len(path)-1
        node_path =[] #create placeholder for list of nodes in final path
        while i>=0:
            if i == len(path)-1: #time of origin is 0
                t = 0
            else:
                t = path[i+1][3]+math.hypot(path[i+1][0]-path[i][0],path[i+1][1]-path[i][1],path[i+1][2]-path[i][2])/self.speed #DONE: added z here, used math.hypot
            x = path[i][0] 
            y = path[i][1] 
            z = path[i][2] #DONE: added z
            n = self.Node(x,y,z,t) #DONE: added z here
            path[i].append(t)
            node_path.append(n) 
            i=i-1
        self.node_path = node_path #save the node path in case it will be useful (it won't be)
        path.reverse()

        return path

    def choose_parent(self, new_node, near_inds):
        """
        Computes the cheapest point to new_node contained in the list
        near_inds and set such a node as the parent of new_node.
            Arguments:
            --------
                new_node, Node
                    randomly generated node with a path from its neared point
                    There are not coalitions between this node and th tree.
                near_inds: list
                    Indices of indices of the nodes what are near to new_node
            Returns.
            ------
                Node, a copy of new_node
        """
        if not near_inds:
            return None

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            t_node.cost=  self.calc_new_cost(near_node, new_node) #Added here to define cost earlier.
            if t_node and self.check_collision(
                    t_node, self.obstacle_list, self.robot_radius): #Called before cost of new node is defined
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self):
        dist_to_goal_list = [
            self.calc_dist_to_goal(n.x, n.y, n.z) for n in self.node_list #DONE: added z here
        ]
        goal_inds = [
            dist_to_goal_list.index(i) for i in dist_to_goal_list
            if i <= self.expand_dis
        ]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node) #: get steer from 3DRRT
            t_node.cost = self.calc_new_cost(t_node.parent, t_node) 
            if self.check_collision(
                    t_node, self.obstacle_list, self.robot_radius):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the three that are inside this ball
            Arguments:
            ---------
                new_node: Node
                    new randomly generated node, without collisions between
                    its nearest node
            Returns:
            -------
                list
                    List with the indices of the nodes inside the ball of
                    radius r
        """
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt(math.log(nnode) / nnode)
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2 + (node.z - new_node.z)**2
                     for node in self.node_list] #DONE: added z
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds

    def rewire(self, new_node, near_inds):
        """
            For each node in near_inds, this will check if it is cheaper to
            arrive to them from new_node.
            In such a case, this will re-assign the parent of the nodes in
            near_inds to new_node.
            Parameters:
            ----------
                new_node, Node
                    Node randomly added which can be joined to the tree
                near_inds, list of uints
                    A list of indices of the self.new_node which contains
                    nodes within a circle of a given radius.
            Remark: parent is designated in choose_parent.
        """
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(
                edge_node, self.obstacle_list, self.robot_radius)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost: #DONE: added z
                near_node.x = edge_node.x
                near_node.y = edge_node.y
                near_node.z = edge_node.z
                near_node.cost = edge_node.cost
                near_node.path_x = edge_node.path_x
                near_node.path_y = edge_node.path_y
                near_node.path_z = edge_node.path_z
                near_node.parent = edge_node.parent
                self.propagate_cost_to_leaves(new_node)

    def calc_new_cost(self, from_node, to_node):
        d, _, _ = self.calc_distance_and_angle(from_node, to_node)
        t = d/self.speed #cost is equal to time
        return from_node.cost + t

    def propagate_cost_to_leaves(self, parent_node):

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)
    def discretize_path(self, path, timestep): #reorganize the path in terms of timesteps to aid in collision detection
        t = 0
        p = path
        d_path = []
        while t <= p[len(p)-1][3]: #DONE: changed to index time while accomodating z
            i = 0
            while i<len(p): 
                if t>= p[i][3] and t<= p[i+1][3]: #DONE: changed to index time while accomodating z
                    x = p[i][0]+((t-p[i][3])/(p[i+1][3]-p[i][3]))*(p[i+1][0]-p[i][0]) #compute x coord
                    y = p[i][1]+((t-p[i][3])/(p[i+1][3]-p[i][3]))*(p[i+1][1]-p[i][1]) #compute y coord
                    z = p[i][2]+((t-p[i][3])/(p[i+1][3]-p[i][3]))*(p[i+1][2]-p[i][2]) #compute z coord
                    d_path.append([round(x,4),round(y,4),round(z,4),round(t,1)]) #DONE: added z here and above
                i=i+1
            t=t+timestep
        self.d_path = d_path
        return d_path

    def check_collision(self, node, obstacleList, robot_radius): #whole method must change when z is added

        if node is None:
            return False

        for (ox, oy, oz, size) in obstacleList:             #need z
            dx_list = [ox - x for x in node.path_x]         #need z
            dy_list = [oy - y for y in node.path_y]
            dz_list = [oz - z for z in node.path_z]         #they multiply the d terms by themselves next line to get distance
            d_list = [dx * dx + dy * dy + dz * dz for (dx, dy, dz) in zip(dx_list, dy_list, dz_list)]  #need z

            if min(d_list) <= (size+robot_radius)**2: #if minimum distance is within robot, collision occured
                return False  # collision
        movingObstacleList = self.moving_obstacle_list
        if  movingObstacleList:
            if node.parent is None:
                for ob in movingObstacleList:
                    d = math.hypot(node.x - ob.d_path[0][0], node.y - ob.d_path[0][1], node.z - ob.d_path[0][2]) #DONE: added z
                    if d <= ob.radius + robot_radius+.1:
                        return False
            else:
                old_node = node.parent
                time = old_node.cost #start time of node path
                timediff = node.cost - old_node.cost
                if old_node.x == node.x and old_node.y == node.y and old_node.z == node.z: #DONE: added z
                    return True #if the nodes are equal, collision has already been checked.
                speed = math.hypot(node.x-old_node.x,node.y-old_node.y,node.z-old_node.z)/(timediff) #DONE: add z
                node_path_index = 1 #setting index to iterate through discretized node path. Start at 1
                while node_path_index<len(node.path_x):
                    (x,y,z) = (node.path_x[node_path_index],node.path_y[node_path_index],node.path_z[node_path_index]) #DONE: added z
                    (x0,y0,z0) = (node.path_x[node_path_index-1],node.path_y[node_path_index-1],node.path_z[node_path_index-1]) #DONE: added z
                    dist_btwn_path_points =math.hypot(x-x0,y-y0,z-z0) #DONE: added z
                    time = time+(dist_btwn_path_points/speed)
                    for ob in movingObstacleList:
                        if len(ob.d_path)>1: #check if the path exists before calculating speed
                            ob_path_tstep = ob.d_path[1][3] - ob.d_path[0][3] #DONE: changed time index to accomodate z
                            ob_path_index = int(time//ob_path_tstep)
                        else:
                            ob_path_index = 0 
                        if ob_path_index<len(ob.d_path):
                            d = math.hypot(x-ob.d_path[ob_path_index][0],y-ob.d_path[ob_path_index][1],z-ob.d_path[ob_path_index][2]) #DONE: added z to compte dist
                            if d <= ob.radius + robot_radius +ob_path_tstep: #ob tstep is added because we need to cover the whole uncertainty region
                                return False
                    node_path_index = node_path_index+1
        return True  # safe

def main():
    print("Start " + __file__)
    print()

    
    # ====Search Path with RRT*====
    ''' obstacle_list = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1),
        (6, 12, 1)]'''
    obstacle_list = [(-2,-2,-2,1), (5,5,5,1), (11,7,9,1), (1,12,12, 2)] #DONE: add z
      # [x,y,z,size(radius)]

    # Set Initial parameters
    #Robot 0
    rrt_star0 = RRTStar( #DONE: add z to all coordinates (not rand_area)    #TODO
        start=[-1, 1, 1],
        goal=[6.5, 6.5, 6.5],
        rand_area=[-2, 15],
        obstacle_list=obstacle_list,
        expand_dis=5,
        robot_radius=.3,
        name = "Robot 0") #include speed
    (path1, d_path1) = rrt_star0.path_creation()
    #Robot1
    rrt_star1 = RRTStar( #DONE: add z to all coordinates (not rand_area) #TODO These are the three instances of the figure
        start=[12, 8, 10],
        goal=[2, -1, 0],
        rand_area=[-2, 15],
        obstacle_list=obstacle_list,
        expand_dis=5,
        robot_radius=.3,
        name = "Robot 1") #include speed
    (path2, d_path2) = rrt_star1.path_creation()
    #Robot2
    rrt_star2 = RRTStar( #TODO: add z to all coordinates (not rand_area)
        start=[2, -1, 0],
        goal=[12, 8, 10],
        rand_area=[-2, 15],
        obstacle_list=obstacle_list,
        expand_dis=5,
        robot_radius=.3,
        name = "Robot 2") #include speed
    (path3, d_path3) = rrt_star2.path_creation()

        # Draw final path


    RRTlist = [rrt_star0, rrt_star1, rrt_star2]
    MARRT = MultiAgentSim(RRTlist)
    MARRT.check_paths()
    print("Robots in conflict: ",MARRT.conflict_index_list)
    res_attempts = 0 #initialize number of resolution loops
    while res_attempts < len(RRTlist) and MARRT.conflict_index_list: #run if conflict and feasible solution
        MARRT.conflict_resolution()
        MARRT.check_paths()
        res_attempts = res_attempts+1
        
    ax = rrt_star0.draw_graph()
    ax.set_title('Single Robot 3D RRT Star With Nodes')
    ax.plot3D([x for (x, y, z, t) in rrt_star0.path[1:-1]], [y for (x, y, z, t) in rrt_star0.path[1:-1]], [z for (x, y, z, t) in rrt_star0.path[1:-1]], 'r*')  #This is the old path
    ax.plot3D([x for (x, y, z, t) in rrt_star0.d_path], [y for (x, y, z, t) in rrt_star0.d_path], [z for (x, y, z, t) in rrt_star0.d_path], '-r', label='Robot 0')

    ax.plot3D([x for (x, y, z, t) in rrt_star1.path[1:-1]], [y for (x, y, z, t) in rrt_star1.path[1:-1]], [z for (x, y, z, t) in rrt_star1.path[1:-1]], 'r*')  #need z I think this worked with out the 3D on plot
    ax.plot3D([x for (x, y, z, t) in rrt_star1.d_path], [y for (x, y, z, t) in rrt_star1.d_path], [z for (x, y, z, t) in rrt_star1.d_path], '-b', label='Robot 1')
    ax.plot3D(rrt_star1.start.x, rrt_star1.start.y, rrt_star1.start.z, "xb")  #Start node of rrt2
    ax.plot3D(rrt_star1.end.x, rrt_star1.end.y, rrt_star1.end.z, "xg")      #End node of rrt3

    ax.plot3D([x for (x, y, z, t) in rrt_star2.path[1:-1]], [y for (x, y, z, t) in rrt_star2.path[1:-1]], [z for (x, y, z, t) in rrt_star2.path[1:-1]], 'r*')  #need z I think this worked with out the 3D on plot
    ax.plot3D([x for (x, y, z, t) in rrt_star2.d_path], [y for (x, y, z, t) in rrt_star2.d_path], [z for (x, y, z, t) in rrt_star2.d_path], '-y', label='Robot 2')
    ax.plot3D(rrt_star2.start.x, rrt_star2.start.y, rrt_star2.start.z, "xb")  #Start node of rrt2
    ax.plot3D(rrt_star2.end.x, rrt_star2.end.y, rrt_star2.end.z, "xg")      #End node of rrt3
    
    plt.legend(loc='upper right') 
    plt.pause(0.01)  # Need for Mac
    plt.show()
    
    if MARRT.RRT_list[0].path != rrt_star0.path or MARRT.RRT_list[1].path != rrt_star1.path or MARRT.RRT_list[2].path != rrt_star2.path:

        ax = rrt_star0.draw_graph()
        ax.set_title('3D RRT Star With Collision Correction')


        ax.plot3D([x for [x, y, z, t] in MARRT.RRT_list[0].path[1:-1]], [y for [x, y, z, t] in MARRT.RRT_list[0].path[1:-1]], [z for [x, y, z, t] in MARRT.RRT_list[0].path[1:-1]],'r*' ) #TODO: change when z is added
        ax.plot3D([x for [x, y, z, t] in MARRT.RRT_list[0].d_path], [y for [x, y, z, t] in MARRT.RRT_list[0].d_path], [z for [x, y, z, t] in MARRT.RRT_list[0].d_path], 'r', label='Robot 0')
        ax.plot3D(MARRT.RRT_list[0].start.x, MARRT.RRT_list[0].start.y, MARRT.RRT_list[0].start.z, "xb")  #Start node of rrt2
        ax.plot3D(MARRT.RRT_list[0].end.x, MARRT.RRT_list[0].end.y, MARRT.RRT_list[0].end.z, "xg")      #End node of rrt3


        ax.plot3D([x for [x, y, z, t] in MARRT.RRT_list[1].path[1:-1]], [y for [x, y, z, t] in MARRT.RRT_list[1].path[1:-1]], [z for [x, y, z, t] in MARRT.RRT_list[1].path[1:-1]], 'r*') #TODO: change when z is added
        ax.plot3D([x for [x, y, z, t] in MARRT.RRT_list[1].d_path], [y for [x, y, z, t] in MARRT.RRT_list[1].d_path], [z for [x, y, z, t] in MARRT.RRT_list[1].d_path], 'b', label='Robot 1')
        ax.plot3D(MARRT.RRT_list[1].start.x, MARRT.RRT_list[1].start.y, MARRT.RRT_list[1].start.z, "xb")  #Start node of rrt2
        ax.plot3D(MARRT.RRT_list[1].end.x, MARRT.RRT_list[1].end.y, MARRT.RRT_list[1].end.z, "xg")      #End node of rrt3

        ax.plot3D([x for [x, y, z, t] in MARRT.RRT_list[2].path[1:-1]], [y for [x, y, z, t] in MARRT.RRT_list[2].path[1:-1]], [z for [x, y, z, t] in MARRT.RRT_list[2].path[1:-1]], 'r*') #TODO: change when z is added
        ax.plot3D([x for [x, y, z, t] in MARRT.RRT_list[2].d_path], [y for [x, y, z, t] in MARRT.RRT_list[2].d_path], [z for [x, y, z, t] in MARRT.RRT_list[2].d_path], 'y', label='Robot 2')
        ax.plot3D(MARRT.RRT_list[2].start.x, MARRT.RRT_list[2].start.y, MARRT.RRT_list[2].start.z, "xb")  #Start node of rrt2
        ax.plot3D(MARRT.RRT_list[2].end.x, MARRT.RRT_list[2].end.y, MARRT.RRT_list[2].end.z, "xg")      #End node of rrt3
        
        plt.legend(loc='upper right')
        plt.grid(True)
        plt.show()



if __name__ == '__main__':
    main()