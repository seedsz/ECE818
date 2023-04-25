"""

Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

author: AtsushiSakai(@Atsushi_twi)

"""

import math
import random

import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits import mplot3d #To graoh 3D


show_animation = True

 


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y, z): #added Z
            self.x = x
            self.y = y
            self.z = z
            self.path_x = []
            self.path_y = []
            self.path_z = []
            self.parent = None

    class AreaBounds:

        def __init__(self, area):  #need a z
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])
            self.zmin = float(area[4])
            self.zmax = float(area[5])

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500,
                 play_area=None,
                 robot_radius=0.0,
                 ):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        play_area:stay inside this area [xmin,xmax,ymin,ymax]
        robot_radius: robot body modeled as circle with given radius

        """
        self.start = self.Node(start[0], start[1], start[2]) #added the [2]
        self.end = self.Node(goal[0], goal[1], goal[2])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.robot_radius = robot_radius

    def planning(self, animation=True):
        """
        rrt path planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_if_outside_play_area(new_node, self.play_area) and \
               self.check_collision(
                   new_node, self.obstacle_list, self.robot_radius):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y,
                                      self.node_list[-1].z) <= self.expand_dis: #need z
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision(
                        final_node, self.obstacle_list, self.robot_radius):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)

        return None  # cannot find path

    def steer( self, from_node, to_node, extend_length=float("inf")):
    
        new_node = self.Node(from_node.x, from_node.y, from_node.z)          #need Z
        d, theta, phi = self.calc_distance_and_angle(new_node, to_node)      # need z? two angles need to be here
    
        new_node.path_x = [new_node.x]  #need z
        new_node.path_y = [new_node.y]  #need z
        new_node.path_z = [new_node.z]
    
        if extend_length > d:
            extend_length = d
    
        n_expand = math.floor(extend_length / self.path_resolution)   #checking how many points to check between start and end point
    
        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta) * math.sin(phi) #need z
            new_node.y += self.path_resolution * math.sin(theta)  #need both angles a line that can roate in any direction
            new_node.z += self.path_resolution * math.sin(theta) * math.cos(phi)  #no way thats right
            new_node.path_x.append(new_node.x)                  #need z
            new_node.path_y.append(new_node.y)
            new_node.path_z.append(new_node.z)
    
        d, _,_ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)   #need z
            new_node.path_y.append(to_node.y)
            new_node.path_z.append(to_node.z)
            new_node.x = to_node.x              #need z
            new_node.y = to_node.y
            new_node.z = to_node.z              #this may need to be different
    
        new_node.parent = from_node
        
        
        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y, self.end.z]]   #need z
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y, node.z])   #need z
            node = node.parent
        path.append([node.x, node.y, node.z]) #Added z

        return path

    def calc_dist_to_goal(self, x, y, z):  #need z
        dx = x - self.end.x
        dy = y - self.end.y
        dz = z - self.end.z
        return math.hypot(dx, dy, dz)   #need z  This function finds the length of the hypotenuse This may actually be fine

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y, self.end.z)     #need z
        return rnd

    def draw_graph(self, rnd=None):     #TODO
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, rnd.z, "^k")        #need z may need new plot method
            if self.robot_radius > 0.0:
                self.plot_sphere(rnd.x, rnd.y, rnd.z, self.robot_radius, '-r')     #need z circle may be wrong
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, node.path_z, "-g")    #need z

        for (ox, oy, oz, size) in self.obstacle_list:
            self.plot_sphere(ox, oy, oz, size)          #need z     Will need to double check this

        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,     #need z
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,     #need z
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     [self.play_area.zmin, self.play_area.zmax,     #may need to swap the order of the mins and maxs
                      self.play_area.zmax, self.play_area.zmin,     #what is going on here
                      self.play_area.zmin],
                     "-k")

        plt.plot(self.start.x, self.start.y, self.start.z, "xr")  #need z
        plt.plot(self.end.x, self.end.y, self.end.z, "xr")      #need z
        plt.axis("equal")
        plt.axis([-2, 15, -2, 15, -2 , 15])      #This is where the min/maxs come into play. I inserted arbitrary z
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_sphere(x, y, z, size, color="-b"):
        plt.rcParams["figure.figsize"] = [7.00, 3.50]
        plt.rcParams["figure.autolayout"] = True
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        u, v = np.mgrid[0:2 * np.pi:30j, 0:np.pi:20j]
        xl = x  + size * np.cos(u) * np.sin(v)
        yl = y + size * np.sin(u) * np.sin(v)
        zl = x + size * np.cos(v)
        ax.plot_surface(xl, yl, zl, color="b")  #plot_surface and plot_wireframea are two different ways to go about it, colors can swap too
        plt.show()

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2 + (node.z -rnd_node.z)**2   #need z how do I go about this
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_if_outside_play_area(node, play_area):

        if play_area is None:
            return True  # no play_area was defined, every pos should be ok

        if node.x < play_area.xmin or node.x > play_area.xmax or \
           node.y < play_area.ymin or node.y > play_area.ymax or \
           node.z < play_area.zmin or node.z > play_area.zmax:  #need z
            return False  # outside - bad
        else:
            return True  # inside - ok

    @staticmethod
    def check_collision(node, obstacleList, robot_radius):

        if node is None:
            return False

        for (ox, oy, oz, size) in obstacleList:             #need z
            dx_list = [ox - x for x in node.path_x]         #need z
            dy_list = [oy - y for y in node.path_y]
            dz_list = [oz - z for z in node.path_z]         #they multiply the d terms by themselves next line to get distance
            d_list = [dx * dx + dy * dy + dz * dz for (dx, dy, dz) in zip(dx_list, dy_list, dz_list)]  #need z

            if min(d_list) <= (size+robot_radius)**2:  
                return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x        #need z
        dy = to_node.y - from_node.y
        dz = to_node.z - from_node.z
        d = math.hypot(dx, dy, dz)         
        theta = math.atan2(dy, dx)  #radian angle ccw from x axis on xy plane
        phi =  math.atan2(dy, dz)   #radian angle ccw from z axis on yz plane
        return d, theta, phi


def main(gx=6.0, gy=10.0, gz= 9.0):
    print("start " + __file__)

    # ====Search Path with RRT====
    obstacleList = [(5, 5, 4, 1), (3, 6, 9, 2), (3, 8, 4, 2), (3, 10, 2, 2), (7, 5, 5, 2),
                    (9, 5, 6, 2), (8, 10, 12, 1)]  # [x, y, radius] need z #TODO I added the third numbers 
    # Set Initial parameters
    rrt = RRT(
        start=[0, 0, 0],                #added z
        goal=[gx, gy, gz],              #need z
        rand_area=[-2, 15],         #need z not sure if thats going to work
        obstacle_list=obstacleList,
        # play_area=[0, 10, 0, 14]
        robot_radius=0.8            #is this 3D
        )
    path = rrt.planning(animation=False)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt.draw_graph()
            ax = plt.axes(projection='3d')  #added this
            ax.plot([x for (x, y, z) in path], [y for (x, y, z) in path], [z for (x, y, z) in path], '-r')  #need z
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()


if __name__ == '__main__':
    main()
