import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# Class for each small grid
class SearchNode:
    def __init__(self, x, y, is_obs, c2m, h):
        self.x = x            # coordinate
        self.y = y            # coordinate
        self.is_obs = is_obs  # obstacle?
        self.c2m = c2m        # cost to move (weight)
        self.h = h            # heuristic
        self.c2c = None       # total cost to come (previous c2c + current c2m)
        self.cost = None      # total cost, depend on the algorithm
        self.parent = None    # previous node

# Class for the whole map
class SearchMaze:
    # Constructor
    def __init__(self, mat_map, start, goal):
        self.start = start          # start coordinate
        self.goal = goal            # goal coordinate
        self.mat_map = mat_map      # map in matrix form
        self.row = len(mat_map)     # map size
        self.col = len(mat_map[0])  # map size
        # map consisting of a matrix of Nodes
        self.node_map = [[None for i in range(self.col)] for j in range(self.row)]

    # Initialize the map before each searching
    def init_map(self):
        for i in range(self.row):
            for j in range(self.col):
                # cost_to_move: moving in the upper map is a bit more costly
                if i < 4:  c2m = 1.5
                else:      c2m = 1
                # heuristic: manhattan distance from the goal
                h = abs(self.goal[0]-i) + abs(self.goal[1]-j)
                # initialize a node instance and put it in the node_map
                self.node_map[i][j] = Node(i, j, self.mat_map[i][j], c2m, h)
        self.node_map[self.start[0]][self.start[1]].c2c = 0


    # Search methods
    def BFS(self): 
        self.search("BFS")

    def DFS(self):
        self.search("DFS")

    def Dijkstra(self):
        self.search("Dijkstra")
        
    def Astar(self):
        self.search("A*")

    # The search steps are very similar
    # The major difference is how the queue is sorted
    def search(self, search_algo):
        self.init_map() # initialize a node map
        found = False 

        # Start dealing with nodes
        queue = [self.node_map[self.start[0]][self.start[1]]]
        checked = []
        # Keep exploding before there is no more nodes
        while queue != []:
            cur_node = queue.pop(0)  # pop up the next node
            checked.append(cur_node) # put it in the chekced list
            for move_step in [[0, -1], [-1, 0], [0, 1], [1, 0]]:
                # move 1 unit manhattan distance
                new_x = cur_node.x + move_step[0]
                new_y = cur_node.y + move_step[1]
                if not 0<= new_x <self.row or not 0<= new_y <self.col: # if breaks the boundary
                    continue
                new_node = self.node_map[new_x][new_y] # if not obstacle and not checked
                if not new_node.is_obs and new_node not in checked:
                    
                    ### Sort the queue according to algorithm ###
                    # For DFS and BFS, the difference is where to insert new node into the queue
                    if   search_algo == "BFS":
                        if new_node not in queue: # not insert existed one
                            new_node.parent = cur_node
                            queue.append(new_node)
                    elif search_algo == "DFS":    
                        if new_node not in queue: # not insert existed one
                            new_node.parent = cur_node
                            queue.insert(0, new_node)
                    # For A* and Dijksra, the only difference is the cost function
                    else: 
                        new_c2c = cur_node.c2c + new_node.c2m
                        if search_algo == "Dijkstra":
                            new_cost = new_c2c
                        elif search_algo == "A*":
                            new_cost = new_c2c + new_node.h
                        if new_node in queue:     # if already existed in queue
                            if new_cost < new_node.cost:  # remove existed if new one is more costly
                                queue.remove(new_node)
                            else:                         # skip this one if not
                                continue
                        new_node.c2c = new_c2c
                        new_node.cost = new_cost
                        new_node.parent = cur_node
                        queue.append(new_node)
                        queue.sort(key=lambda x:x.cost)   # sort according to total cost
                    
                    # If reach the goal, end
                    if new_node.x == self.goal[0] and new_node.y == self.goal[1]: 
                        found = True
                        queue = []
                        break
        
        # Display result
        if found:
            print("It takes", len(checked), "moves to find the path with", search_algo, ".")
            self.draw_path(search_algo)
        else:
            print("There is no exsiting path :(")


    def draw_path(self, title):
        # Visualization of the found path
        fig, ax = plt.subplots(1)
        # Draw map
        for i in range(self.row):
            for j in range(self.col):
                if self.mat_map[i][j]: # obstacle
                    ax.add_patch(Rectangle((j,-i-1),1,1,edgecolor='k',facecolor='k'))
                else:                  # free space
                    ax.add_patch(Rectangle((j,-i-1),1,1,edgecolor='k',facecolor='w'))
        ax.add_patch(Rectangle((self.start[1],-self.start[0]-1),1,1,edgecolor='k',facecolor='g')) # start
        ax.add_patch(Rectangle((self.goal[1], -self.goal[0] -1),1,1,edgecolor='k',facecolor='r')) # goal
        # Draw path
        cur_x, cur_y = self.goal
        while True: # keep tracing back
            parent_node = self.node_map[cur_x][cur_y].parent
            cur_x = parent_node.x
            cur_y = parent_node.y
            if cur_x == self.start[0] and cur_y == self.start[1]:
                break
            ax.add_patch(Rectangle((cur_y,-cur_x-1),1,1,edgecolor='k',facecolor='b')) # path
        
        plt.title(title)
        plt.axis('scaled')


if __name__ == "__main__":
    # Define a maze, 0 is free and 1 is blocked
    grid = [[0, 1, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 0, 1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 1, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 0, 1, 1, 0, 0, 0, 1, 0],
            [0, 1, 0, 0, 0, 0, 1 ,0 ,0 ,0],
            [1, 1, 1, 0, 0, 0, 1, 0, 0, 0]]
    start = [2, 0]
    goal =  [3, 9]

    # Search with different algorithms
    task = Maze(grid, start, goal)
    task.BFS()
    task.DFS()
    task.Dijkstra()
    task.Astar()
    # Show result
    plt.show()