import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from math import sqrt, atan2, sin, cos, floor
import numpy as np
import random


# Class for each small grid
class RRTNode:
    def __init__(self, x, y):
        self.x = x            # coordinate
        self.y = y            # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # length to start


# Class for the whole map
class RRTMaze:
    # Constructor
    def __init__(self, start, goal, obstacles, map_size):
        self.start = RRTNode(start[0], start[1])    # start node
        self.goal = RRTNode(goal[0], goal[1])       # goal node
        self.obstacles = obstacles  # obstacles
        self.row = map_size[1]      # map size
        self.col = map_size[0]      # map size
        self.vertices = []          # node list
        self.goal_bias = 0.05       # possiblity of chossing goal
        self.extend_dis = 10        # extend step length
        self.found = False          # found flag
        self.init_map()

    # Initialize the map before each searching
    def init_map(self):
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)


    # Generate random point
    def get_random_point(self):
        # select goal as random point
        if random.random() < self.goal_bias:
            point = (self.goal.x, self.goal.y)
        # generate random point
        else:
            point = (random.randint(0, self.col), random.randint(0, self.row))
        return point

    # Return the nearest node from the random point
    def get_nearest_node(self, point):
        # Get distances from all node
        distances = [sqrt((point[0]-node.x)**2 + (point[1]-node.y)**2) for node in self.vertices]
        index = distances.index(min(distances))
        # Return the minimum one
        return self.vertices[index]

    # Check if collide with obstacles
    def check_collision(self, x, y):
        collide = False
        # Check boundary
        if x > self.col or x < 0 or y > self.row or y < 0:
            collide = True
        # Check each obstacle
        for (obs_x, obs_y, obs_r) in self.obstacles:
            d = sqrt((x-obs_x)**2 + (y-obs_y)**2)
            if d < obs_r:
                collide = True
        return collide

    # Extend new node
    def extend(self):
        # Generate random points
        random_point = self.get_random_point()
        nearest_node = self.get_nearest_node(random_point)

        # Calculate new node location
        slope = atan2(random_point[1]-nearest_node.y, random_point[0]-nearest_node.x)
        new_x = floor(nearest_node.x + self.extend_dis*cos(slope))
        new_y = floor(nearest_node.y + self.extend_dis*sin(slope))
        # Check collision
        if self.check_collision(new_x, new_y):
            return None

        # Create new node
        new_node = RRTNode(new_x, new_y)
        new_node.parent = nearest_node
        new_node.cost   = nearest_node.cost + self.extend_dis
        self.vertices.append(new_node)

        return new_node

    # Get neighbor nodes
    def get_neighbors(self, new_node, neighbor_size):
        neighbor = []
        for node in self.vertices:
            d = sqrt((new_node.x-node.x)**2 + (new_node.y-node.y)**2)
            # node within neighbor range
            if d < neighbor_size:
                neighbor.append(node)

        return neighbor

    # Rewire node
    def rewire(self, new_node, neighbors):
        if neighbors == []:
            return

        # Get new parent
        distances = [sqrt((new_node.x-node.x)**2 + (new_node.y-node.y)**2)
                     for node in neighbors]
        costs = [distances[i] + neighbors[i].cost
                 for i in range(len(distances))]
        index = costs.index(min(costs))

        new_node.parent = neighbors[index]
        new_node.cost = costs[index]

        # Rewire other neighbors
        for i in range(len(neighbors)):
            node = neighbors[i]
            potential_cost = new_node.cost + distances[i]
            if node.cost > potential_cost:
                node.parent = new_node
                node.cost = potential_cost
        pass


    # RRT
    def RRT_search(self, n_pts=1000):
        self.init_map()
        # Start searching       
        for i_ in range(n_pts):
            # Extend a new node
            new_node = self.extend()
            if new_node is None:
                continue

            # Check if goal arrived
            d = sqrt((new_node.x-self.goal.x)**2 + (new_node.y-self.goal.y)**2)
            if d < self.extend_dis:
                self.goal.cost = new_node.cost + d
                self.goal.parent = new_node
                self.vertices.append(self.goal)

                self.found = True
                break
        
        self.draw_map()


    # RRT Star
    def RRT_star_search(self, n_pts=1000, neighbor_size=30):
        self.init_map()
        # Start searching       
        for i_ in range(n_pts):
            # Extend a new node
            new_node = self.extend()
            if new_node is None:
                continue
            
            # Rewire
            neighbors = self.get_neighbors(new_node, neighbor_size)
            self.rewire(new_node, neighbors)

            # Check if goal arrived
            if not self.found:
                d = sqrt((new_node.x-self.goal.x)**2 + (new_node.y-self.goal.y)**2)
                if d <= self.extend_dis:
                    self.goal.cost = new_node.cost + d
                    self.goal.parent = new_node
                    self.vertices.append(self.goal)
                    neighbors = self.get_neighbors(self.goal, neighbor_size)
                    self.rewire(self.goal, neighbors)

                    self.found = True  

        self.draw_map()


    # Visualization
    def draw_map(self):
        # Create empty map
        fig, ax = plt.subplots()
        img = 255*np.ones([self.row, self.col, 3], dtype=np.uint8)
        ax.imshow(img)

        # Draw start, goal and obstacles
        start_cir = Circle((self.start.x, self.start.y), 10, color='b')
        goal_cir = Circle((self.goal.x, self.goal.y), 10, color='r')
        ax.add_patch(start_cir)
        ax.add_patch(goal_cir)
        for obstacle in self.obstacles:
            obstacle_cir = Circle((obstacle[0], obstacle[1]), obstacle[2], color='k')
            ax.add_patch(obstacle_cir)

        # Draw Trees
        for node in self.vertices[1:]:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color='y')

        # Draw Final Path
        if self.found:
            cur = self.goal
            while cur.x != self.start.x and cur.y != self.start.y:    
                plt.plot([cur.x, cur.parent.x], [cur.y, cur.parent.y], color='c')
                cur = cur.parent
            
            # Display path information
            steps = len(self.vertices)-1
            length = self.vertices[-1].cost
            print("It took %d nodes to found" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        plt.show()


if __name__ == "__main__":
    # Define start, goal and obstacles
    obstacles = [[200, 350, 80], 
                 [250, 150, 50], 
                 [400, 250, 80], 
                 [500, 100, 60], 
                 [600, 350, 80]]
    start = (80, 400)
    goal  = (750, 350)
    map_size = (854, 480)
    
    # Search
    task = RRTMaze(start, goal, obstacles, map_size)
    task.RRT_search()
    task.RRT_star_search()