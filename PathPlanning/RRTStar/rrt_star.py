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
            self.tstep = d_path[1][2]-d_path[0][2] #timestep is time between points (THIS MUST CHANGE WHEN Z IS ADDED)

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
                            d = calc_dist(dpath1[path_index],dpath2[path_index])
                            if d <= self.RRT_list[primary_bot_index].robot_radius + self.RRT_list[secondary_bot_index].robot_radius + self.safety_buffer:
                                conflict_list.append(primary_bot_index)
                                conflict_list.append(secondary_bot_index)
                                print("conflict detected between robots ", primary_bot_index, " and ", secondary_bot_index," at time t: ", self.RRT_list[secondary_bot_index].d_path[path_index], self.RRT_list[primary_bot_index].d_path[path_index]) #fix later
                                path_index = len(self.RRT_list[primary_bot_index].d_path) #break out of this part of loop once conflict is found
                            path_index = path_index+1
                    secondary_bot_index = secondary_bot_index+1
            primary_bot_index=primary_bot_index+1
        if conflict_list:
            print("Conflict(s) found. Paths must be replanned.")
            self.conflict_index_list = conflict_list
        else: 
            print("No conflicts found. All paths have been finalized")
        return conflict_list
    #def MovingObstacleGeneration(RRT_list)
                 
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
                 expand_dis=1,
                 path_resolution=.1,
                 goal_sample_rate=20,
                 max_iter=1500,
                 connect_circle_dist=50.0,
                 search_until_max_iter=True,
                 robot_radius=0.0,
                 speed = 1,
                 moving_obstacle_list = []): #speed is used to get time
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
        self.goal_node = self.Node(goal[0], goal[1])
        self.search_until_max_iter = search_until_max_iter
        self.node_list = [] 
        self.speed = speed
        self.path = None #no path to start
        self.d_path = []
        self.moving_obstacle_list = moving_obstacle_list

    def planning(self, animation=True):
        """
        rrt star path planning

        animation: flag for animation on or off .
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.node_list))
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
            if t_node and self.check_collision(
                    t_node, self.obstacle_list, self.robot_radius):
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

            if no_collision and improved_cost:
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
    
    @staticmethod
    def check_collision(node, obstacleList, robot_radius, movingObstacleList =[]): #whole method must change when z is added

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= (size+robot_radius)**2: #if minimum distance is within robot, collision occured
                return False  # collision
        if  movingObstacleList:
            tstep = movingObstacleList[0].tstep
            path_index = node.cost//tstep
            for ob in movingObstacleList:
                path = ob.dpath
                if len(path)>path_index+1: #only check if the time occurs within path
                    dx_list = [ox - x for x in node.path_x]
                    dy_list = [oy - y for y in node.path_y]
                    d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

        return True  # safe

def calc_dist(point1,point2):
    dist = math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)
    return dist
def main():
    print("Start " + __file__)

    # ====Search Path with RRT====
    obstacle_list = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1),
        (6, 12, 1),
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    rrt_star1 = RRTStar(
        start=[-1, 1],
        goal=[0, 0],
        rand_area=[-2, 15],
        obstacle_list=obstacle_list,
        expand_dis=2,
        robot_radius=.1) #include speed
    path1 = rrt_star1.planning(animation=False)

    rrt_star2 = RRTStar(
        start=[1, -1],
        goal=[5, 3],
        rand_area=[-2, 15],
        obstacle_list=obstacle_list,
        expand_dis=2,
        robot_radius=.1) #include speed
    path2 = rrt_star2.planning(animation=False)

    if path1 is None:
        print("Cannot find path")
    else:
        print("found path!!")
        for spot in path1:
            print(spot)
        d_path1 =rrt_star1.discretize_path(path1, .1)

    if path2 is None:
        print("Cannot find path")
    else:
        print("found path!!")
        for spot in path2:
            print(spot)
        d_path2 =rrt_star2.discretize_path(path2, .1)

        # Draw final path
    if show_animation:
        rrt_star1.draw_graph()
        plt.plot([x for (x, y, t) in path1], [y for (x, y, t) in path1], 'r*')
        plt.plot([x for (x, y, t) in d_path1], [y for (x, y, t) in d_path1], 'y*')

    if show_animation:
        plt.plot([x for (x, y, t) in path2], [y for (x, y, t) in path2], 'r*')
        plt.plot([x for (x, y, t) in d_path2], [y for (x, y, t) in d_path2], 'y*')
        plt.grid(True)

    RRTlist = [rrt_star1,rrt_star2]
    MARRT = MultiAgentSim(RRTlist)
    print("test")
    MARRT.print_test()
    MARRT.check_paths()
    print("Robots in conflict: ",MARRT.conflict_index_list)
    plt.show()


if __name__ == '__main__':
    main()
