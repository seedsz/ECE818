"""

Path planning Sample Code with RRT*

author: Atsushi Sakai(@Atsushi_twi)

"""

import math
import sys
import matplotlib.pyplot as plt
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from RRT.rrt import RRT

show_animation = True

class MultiAgentSim:
    '''
    Class for simulating the multi-agent algorithm
    '''

    class MovingObstacle:
        def __init__(self, radius = 1, d_path = None, ):
            self.radius = radius
            self.d_path = d_path
            self.tstep = d_path[1][2]-d_path[0][2] #TODO: add z to index.

    def __init__(self, #multi-agent simulation
                 RRT_list=None, #list of all cooperating RRTs/robots
                 safety_buffer = .1):
        self.RRT_list = RRT_list
        self.conflict_index_list = [] #list of indexes of all paths/robots in conflict
        self.safety_buffer = safety_buffer

    def print_test(self):
        for bot in self.RRT_list:
            print(bot.speed)

    def check_paths(self):
        '''
        Check the planned paths of each robot to identify collisions.
        Because the control is decentralized, each robot must individually check the other paths.
        Safety buffer is the distance that each robot must maintain from another.
        '''
        primary_bot_index = 0
        conflict_list = []
        while primary_bot_index<len(self.RRT_list):
            secondary_bot_index =0
            if primary_bot_index not in conflict_list:
                while secondary_bot_index<len(self.RRT_list):
                    if secondary_bot_index != primary_bot_index:
                        path_index = 0
                        dpath1 = self.RRT_list[primary_bot_index].d_path
                        dpath2 = self.RRT_list[secondary_bot_index].d_path
                        while path_index<len(self.RRT_list[primary_bot_index].d_path) and path_index<len(self.RRT_list[secondary_bot_index].d_path):
                            d = calc_dist(dpath1[path_index],dpath2[path_index]) #TODO: 
                            if d <= self.RRT_list[primary_bot_index].robot_radius + self.RRT_list[secondary_bot_index].robot_radius + self.safety_buffer:
                                if primary_bot_index not in conflict_list:
                                    conflict_list.append(primary_bot_index)
                                if secondary_bot_index not in conflict_list:
                                    conflict_list.append(secondary_bot_index)
                                print("conflict detected between robots ", primary_bot_index, " and ", secondary_bot_index," at (x,y,t): ", self.RRT_list[secondary_bot_index].d_path[path_index], self.RRT_list[primary_bot_index].d_path[path_index]) #fix later
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
            moving_ob = self.MovingObstacle(bot.robot_radius+.1,bot.d_path) #add a .1m safety region
            moving_ob_list.append(moving_ob)
        for i in self.conflict_index_list: #create new RRT with moving obstacles for each conflicted path
            spec_moving_obstacle_list = moving_ob_list[:i] + moving_ob_list[i+1:]
            old_rrt = self.RRT_list[i]
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
            rewire_on=False,) #do not allow rewiring, as this can cause unforeseen collision
            (path, d_path) = rrt_star.path_creation()
            delta_cost = path[-1][2]-self.RRT_list[i].path[-1][2] #TODO: change when z is added, I want to index time
            New_RRT_List.append((i, rrt_star, self.RRT_list[i], delta_cost)) #tuple is (index, new, old, cost)
        lowest_cost_tuple = New_RRT_List[0]
        for tup in New_RRT_List:
            if tup[-1] < lowest_cost_tuple[-1]:
                lowest_cost_tuple = tup
        self.RRT_list[lowest_cost_tuple[0]] = lowest_cost_tuple[1] #replaces RRT star with lowest difference in costs
        print("Robot ", lowest_cost_tuple[0], "'s path has been modified to avoid conflict. ", "The new path is ", round(lowest_cost_tuple[3],2), " seconds slower.")

class RRTStar(RRT):
    """
    Class for RRT Star planning
    """

    class Node(RRT.Node):
        def __init__(self, x, y, t = None): 
            super().__init__(x, y) 
            self.cost = 0.0 #cost = time in this sim

    def __init__(self,
                 start, 
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=.1,
                 path_resolution=.1,
                 goal_sample_rate=20,
                 max_iter=3000,
                 connect_circle_dist=50.0,
                 search_until_max_iter=True,
                 robot_radius=0.0,
                 speed = 1,
                 moving_obstacle_list = [],
                 rewire_on = True,
                 name = "Unnamed Robot"): 
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        super().__init__(start, goal, obstacle_list, rand_area, expand_dis,
                         path_resolution, goal_sample_rate, max_iter,
                         robot_radius=robot_radius)
        self.connect_circle_dist = connect_circle_dist
        self.goal_coords = goal
        self.rand_area = rand_area
        self.start_coords = start
        self.goal_node = self.Node(goal[0], goal[1])
        self.search_until_max_iter = search_until_max_iter
        self.node_list = [] 
        self.speed = speed
        self.path = None #no path to start
        self.d_path = []
        self.moving_obstacle_list = moving_obstacle_list
        self.rewire_on = rewire_on #used to decide whether rewiring is allowed
        self.name = name

    def planning(self, animation=True):
        """
        rrt star path planning

        animation: flag for animation on or off .
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            #print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd,
                                  self.expand_dis)
            near_node = self.node_list[nearest_ind]
            new_node.cost = near_node.cost + \
                math.hypot(new_node.x-near_node.x,
                           new_node.y-near_node.y)

            if self.check_collision( #we will have to edit check collision to include time
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
                self.draw_graph(rnd)

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
        path = self.planning(animation=False)
        if path is None:
            print("Cannot find path")
            sys.exit() #if a path cannot be found, notify the user and end the program
        else:
            print("found path for", self.name)
            print("[x, y, t]")
            for spot in path:
                clean_spot = [round(coord, 2) for coord in spot ]
                print(clean_spot)
            d_path =self.discretize_path(path, timestep)
            self.d_path = d_path
            return (path, d_path)

    def generate_final_course(self, goal_ind): #calculates final path once goal is reached
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
    
        while node.parent is not None: #go backwards from goal and find point that spawned it
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y]) #origin has no parent, it is the last point.
        i = len(path)-1
        node_path =[] #create placeholder for list of nodes in final path
        while i>=0:
            if i == len(path)-1: #time of origin is 0
                t = 0
            else:
                t = path[i+1][2]+math.sqrt((path[i+1][0]-path[i][0])**2+(path[i+1][1]-path[i][1])**2)/self.speed
            x = path[i][0]
            y = path[i][1]
            n = self.Node(x,y,t)
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
            self.calc_dist_to_goal(n.x, n.y) for n in self.node_list
        ]
        goal_inds = [
            dist_to_goal_list.index(i) for i in dist_to_goal_list
            if i <= self.expand_dis
        ]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node)
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
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2
                     for node in self.node_list]
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

            if no_collision and improved_cost: #add and rewire_on = True
                near_node.x = edge_node.x
                near_node.y = edge_node.y
                near_node.cost = edge_node.cost
                near_node.path_x = edge_node.path_x
                near_node.path_y = edge_node.path_y
                near_node.parent = edge_node.parent
                self.propagate_cost_to_leaves(new_node)

    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        t = d/self.speed #cos is equal to time
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
        while t <= p[len(p)-1][2]:
            i = 0
            while i<len(p):
                if t>= p[i][2] and t<= p[i+1][2]:
                    x = p[i][0]+((t-p[i][2])/(p[i+1][2]-p[i][2]))*(p[i+1][0]-p[i][0]) #compute x coord
                    y = p[i][1]+((t-p[i][2])/(p[i+1][2]-p[i][2]))*(p[i+1][1]-p[i][1]) #compute y coord
                    d_path.append([round(x,4),round(y,4),round(t,1)])
                i=i+1
            t=t+timestep
        self.d_path = d_path
        return d_path
    
    def check_collision(self, node, obstacleList, robot_radius): #whole method must change when z is added

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= (size+robot_radius)**2: #if minimum distance is within robot, collision occured
                return False  # collision
        movingObstacleList = self.moving_obstacle_list
        if  movingObstacleList:
            if node.parent is None:
                for ob in movingObstacleList:
                    d = math.hypot(node.x - ob.d_path[0][0], node.y - ob.d_path[0][1])
                    if d <= ob.radius + robot_radius:
                        return False
            else:
                old_node = node.parent
                time = old_node.cost #start time of node path
                timediff = node.cost - old_node.cost
                if old_node.x == node.x and old_node.y == node.y:
                    return True #if the nodes are equal, collision has already been checked.
                speed = math.hypot(node.x-old_node.x,node.y-old_node.y)/(timediff) #TODO: add z
                node_path_index = 1 #setting index to iterate through discretized node path. Start at 1
                while node_path_index<len(node.path_x):
                    (x,y) = (node.path_x[node_path_index],node.path_y[node_path_index])
                    (x0,y0) = (node.path_x[node_path_index-1],node.path_y[node_path_index-1])
                    dist_btwn_path_points =math.hypot(x-x0,y-y0)
                    time = time+dist_btwn_path_points/speed
                    for ob in movingObstacleList:
                        if len(ob.d_path)>1: #check if the path exists before calculating speed
                            ob_path_tstep = ob.d_path[1][2] - ob.d_path[0][2] #will change when z is added - uses index for time
                            ob_path_index = int(time//ob_path_tstep)
                        else:
                            ob_path_index = 0 #TODO: prevent index out of range below
                        if ob_path_index<len(ob.d_path):
                            d = math.hypot(x-ob.d_path[ob_path_index][0],y-ob.d_path[ob_path_index][1]) #change when z is added - computes distance
                            if d <= ob.radius + robot_radius+ ob_path_tstep: #if distance between point on node path and point on obstacle path is too small, collides
                                return False
                            
                    node_path_index = node_path_index+1
        return True  # safe

def calc_dist(point1,point2):
    dist = math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)
    return dist
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
    obstacle_list = [(5,5,1),(10,5,2)]
      # [x,y,size(radius)]

    # Set Initial parameters
    #Robot 0
    rrt_star0 = RRTStar(
        start=[4, -2],
        goal=[6.5, 6.5],
        rand_area=[-2, 15],
        obstacle_list=obstacle_list,
        expand_dis=2,
        robot_radius=.3,
        name = "Robot 0") #include speed
    (path1, d_path1) = rrt_star0.path_creation()
    #Robot1
    rrt_star1 = RRTStar(
        start=[.2, 0],
        goal=[12, 8],
        rand_area=[-2, 15],
        obstacle_list=obstacle_list,
        expand_dis=2,
        robot_radius=.3,
        name = "Robot 1") #include speed
    (path2, d_path2) = rrt_star1.path_creation()
    #Robot2
    rrt_star2 = RRTStar(
        start=[2, -1],
        goal=[8, 8],
        rand_area=[-2, 15],
        obstacle_list=obstacle_list,
        expand_dis=2,
        robot_radius=.3,
        name = "Robot 2") #include speed
    (path3, d_path3) = rrt_star2.path_creation()


    RRTlist = [rrt_star0,rrt_star1,rrt_star2]
    MARRT = MultiAgentSim(RRTlist)
    MARRT.check_paths()
    print("Robots in conflict: ",MARRT.conflict_index_list)
    res_attempts = 0 #initialize number of resolution loops
    while res_attempts < len(RRTlist) and MARRT.conflict_index_list: #run if conflict and feasible solution
        MARRT.conflict_resolution()
        MARRT.check_paths()
        res_attempts = res_attempts+1
        # Draw final path


    MARRT.RRT_list[0].draw_graph()
    plt.plot([x for [x, y, t] in MARRT.RRT_list[0].path[1:-1]], [y for [x, y, t] in MARRT.RRT_list[0].path[1:-1]], 'r*') 
    plt.plot([x for [x, y, t] in MARRT.RRT_list[0].d_path], [y for [x, y, t] in MARRT.RRT_list[0].d_path], 'b', label="Robot 0")
    plt.plot(MARRT.RRT_list[0].start.x, MARRT.RRT_list[0].start.y, "xb")  #need z
    plt.plot(MARRT.RRT_list[0].end.x, MARRT.RRT_list[0].end.y, "xg")  
    plt.plot([x for [x, y, t] in MARRT.RRT_list[1].path[1:-1]], [y for [x, y, t] in MARRT.RRT_list[1].path[1:-1]], 'r*') #TODO: change when z is added
    plt.plot([x for [x, y, t] in MARRT.RRT_list[1].d_path], [y for [x, y, t] in MARRT.RRT_list[1].d_path], 'g', label="Robot 1")
    plt.plot(MARRT.RRT_list[1].start.x, MARRT.RRT_list[1].start.y, "xb")  #need z
    plt.plot(MARRT.RRT_list[1].end.x, MARRT.RRT_list[1].end.y, "xg")  
    plt.plot([x for [x, y, t] in MARRT.RRT_list[2].path[1:-1]], [y for [x, y, t] in MARRT.RRT_list[2].path[1:-1]], 'r*') #TODO: change when z is added
    plt.plot([x for [x, y, t] in MARRT.RRT_list[2].d_path], [y for [x, y, t] in MARRT.RRT_list[2].d_path], 'y', label="Robot 2")
    plt.plot(MARRT.RRT_list[2].start.x, MARRT.RRT_list[2].start.y, "xb")  #need z
    plt.plot(MARRT.RRT_list[2].end.x, MARRT.RRT_list[2].end.y, "xg")  
    plt.grid(True)
    plt.title("2D MA-RRT* Finalized Paths")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend(loc = "upper left")
    plt.show()
    #plot initial path as well if final is different than initial
    if MARRT.RRT_list[0].path != rrt_star0.path or MARRT.RRT_list[1].path != rrt_star1.path or MARRT.RRT_list[2].path != rrt_star2.path:
        rrt_star0.draw_graph()
        plt.plot([x for [x, y, t] in rrt_star0.path[1:-1]], [y for [x, y, t] in rrt_star0.path[1:-1]], 'r*') #TODO: change when z is added
        plt.plot([x for [x, y, t] in rrt_star0.d_path], [y for [x, y, t] in rrt_star0.d_path], 'b', label="Robot 0")
        plt.plot(rrt_star0.start.x, rrt_star0.start.y, "xb")  #need z
        plt.plot(rrt_star0.end.x, rrt_star0.end.y, "xg")  
        plt.plot([x for [x, y, t] in rrt_star1.path[1:-1]], [y for [x, y, t] in rrt_star1.path[1:-1]], 'r*') #TODO: change when z is added
        plt.plot([x for [x, y, t] in rrt_star1.d_path], [y for [x, y, t] in rrt_star1.d_path], 'g', label="Robot 1")
        plt.plot(rrt_star1.start.x, rrt_star1.start.y, "xb")  #need z
        plt.plot(rrt_star1.end.x, rrt_star1.end.y, "xg")  
        plt.plot([x for [x, y, t] in rrt_star2.path[1:-1]], [y for [x, y, t] in rrt_star2.path[1:-1]], 'r*') #TODO: change when z is added
        plt.plot([x for [x, y, t] in rrt_star2.d_path], [y for [x, y, t] in rrt_star2.d_path], 'y', label="Robot 2")
        plt.plot(rrt_star2.start.x, rrt_star2.start.y, "xb")  #need z
        plt.plot(rrt_star2.end.x, rrt_star2.end.y, "xg")  
        plt.grid(True)
        plt.title("2D MA-RRT* Initial Paths")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.legend(loc = "upper left")
        plt.show()

if __name__ == '__main__':
    main()
