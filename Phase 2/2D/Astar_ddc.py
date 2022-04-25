import utils
from arena import Node, Graph, Robot
try:
   import queue 
except ImportError:
   import Queue as queue
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import argparse, time, csv, math


HEIGHT, WIDTH = 10.0, 10.0

WHITE = (220, 220, 220)
GREEN = (0, 255, 0)
CYAN = (136, 255, 196)
BLACK = (0, 0, 0)
BLUE = (0, 20, 108)

class Astar_DDC(Graph): 
    def __init__(self, robot, start, goal, clearance):
        # super(Graph, self).__init__()
        Graph.__init__(self, clearance)
        self.savepath = utils.foldercheck('./results')

        self.start, self.goal = start, goal
        self.robot = robot
        # defining set of actions that can be done by the constraint.
        self.actions =  [[robot.RPM1, robot.RPM1], [robot.RPM2, robot.RPM2],\
                        [robot.RPM1, 0],                     [0,robot.RPM1], \
                        [robot.RPM1, robot.RPM2],  [robot.RPM2, robot.RPM1],\
                        [0, robot.RPM2],                   [robot.RPM2, 0]]

        self.visualize = False
        self.ax= None
        self.counter=0
        self.final_moves, self.waypoints, self.final_points = [], [], []

    def reset(self):
        self.visualize=False
        self.ax= None
        self.counter=0
        self.final_moves, self.waypoints, self.final_points = [], [], []

    def search(self):
        """
        Input: Starting and ending node for the robot to browse.
        Output: Returns ordered list of poses if an optimal paths.
        """
        threshold = 0.5
        threshold_angle = 360        

        path= []
        if not self.validityCheck(self.start, self.goal): 
            print("[ERROR] Cannot search for this configuration")
            return False, path

        print("Finding path...")

        # state, parent, move, cost, path_array
        init_node = Node(self.start, None, None, 0, None)
        heap =  queue.PriorityQueue()
        heap.put((init_node.cost, init_node))

        if self.visualize:
            print("GOAL and START is being plotted...")            
            self.ax.add_patch(patches.Circle( (self.start[0],self.start[1]), radius=0.1, linewidth=1, alpha=0.5, edgecolor='r', facecolor='r'))
            self.ax.add_patch(patches.Circle( (self.goal[0],self.goal[1]), radius=0.1, linewidth=1, alpha=0.5, edgecolor='g', facecolor='g'))
            plt.savefig(self.savepath+"/"+str(self.counter)+".png")
            
            self.counter+=1
            time.sleep(0.1)
            print("Plotting curves .....")  

        costMatrix = np.array([[[np.inf for k in range(threshold_angle)] for j in range(int(self.h/threshold))] for _ in range(int(self.w/threshold))])
        while (not heap.empty()):
            currentNode = heap.get()[1]
            # traversed_nodes.append(currentNode)
            
            if self.reachedTarget(currentNode.state, thresh_radius=0.2):
                print("Found a path!")
                self.final_moves, node_path = currentNode.backtrack()                
                self.recordPlan(node_path)
                break
        
            for neighbor in self.findNeighbors(currentNode):            
                
                # check and mark visited
                if self.checkVisited(neighbor, costMatrix,  threshold=0.5):

                    new_state = neighbor.state
                    costToGo = self.HeuristicCost(new_state)
                    
                    nx = int(utils.half_round(new_state[0])/threshold)
                    ny = int(utils.half_round(new_state[1])/threshold)
                    costMatrix[nx, ny, new_state[2]] = neighbor.cost + costToGo
                    
                    heap.put((neighbor.cost + costToGo, neighbor))

        return self.final_moves, self.waypoints, self.final_points

    
    def HeuristicCost(self, node_state, h=1.5):
        """
        Returns the CosttoGo, aka Heuristic Cost for A* algorithm. - uses Euclidean distance here. 
        """
        if node_state is None: return 0.0
        return h*((node_state[0]-self.goal[0])**2 + (node_state[1]-self.goal[1])**2)**(0.5)


    def checkVisited(self, node, costMatrix, threshold=0.5):
        """
        Checks if the node has been already visited based on the costMatrix. 
        """        
        x, y, theta = node.state
        x = int(utils.half_round(x)/threshold)
        y = int(utils.half_round(y)/threshold)
        total_cost = node.cost + self.HeuristicCost(node.state)
        return (total_cost < costMatrix[x, y, theta])

    def nextPose(self, state, action, T=1, path_mode=False):
        """
        Returns the next pose to reach and the step cost, considering the differential drive constraints.
        """
        t = 0
        r, L, dt = self.robot.radius, self.robot.wheelDistance, self.robot.dt
        
        Xn, Yn, theta_new = state
        theta_new = utils.deg2rad(theta_new)
        
        wL, wR = action
        # Xn, Yn, theta_new = Xi, Yi, thetai
        path_array = [[Xn, Yn]]
        cost = 0.0

        while t<T:
            t = t + dt
            Xo, Yo = Xn, Yn
            dx = 0.5 * r * (wL + wR) * math.cos(theta_new) * dt
            dy = 0.5 * r * (wL + wR) * math.sin(theta_new) * dt
            Xn += dx
            Yn += dy
            theta_new += (r / L) * (wL - wR) * dt
            cost += math.sqrt(math.pow(dx,2) + math.pow(dy,2))
            path_array.append([Xn, Yn])

            if self.visualize:
                if path_mode:
                    self.ax.plot([Xo, Xn], [Yo, Yn], color='r', linewidth = 2)
                else:
                    self.ax.plot([Xo, Xn], [Yo, Yn], color='b', alpha=0.2)

        if self.visualize:
            plt.savefig(self.savepath+"/"+str(self.counter)+".png")
            self.counter+=1
            time.sleep(0.05)

        theta_new = int(utils.rad2deg(theta_new))
        return [Xn, Yn, theta_new] , path_array, cost

    def findNeighbors(self, node):
        """
        Returns neighbours for the currentNode, based on differential drive constraints.
        """
        state = node.state
        neighbors = []
        for action in self.actions:
            new_state, path_array, step_cost = self.nextPose(state, action)
            x,y, _ = new_state
            if (self.isInsideArena(x, y)) and (not self.insideObstacle(x, y)):    
                # if self.visited[x][y] : continue
                new_node = Node(new_state, node, action, node.cost + step_cost, path_array)
                neighbors.append(new_node)

        return neighbors

    def recordPlan(self, node_path):
        print("Writing in record in ./results")
        fpts = open('./results/path_points.csv', 'w')
        fnode = open('./results/path_nodes.csv', 'w')
        fvel = open('./results/vel_data.csv', 'w')
        pts_writer = csv.writer(fpts)
        nodes_writer = csv.writer(fnode)
        vel_writer = csv.writer(fvel)

        for move in self.final_moves:
            vel_writer.writerow(move)

        for node in node_path:
            xi, yi, thetai = node.state
            nodes_writer.writerow([xi, yi, thetai])
            self.waypoints.append([xi, yi, thetai])

            points = node.pathArray
            if points is not None:
                for point in points:
                    xn, yn = point
                    pts_writer.writerow([xn, yn])
                    self.final_points.append([xn, yn])
                    xi, yi = xn, yn
    

    def reachedTarget(self, current_state, thresh_radius):
        """
        Checks if the currentnode is in target area to terminal the program
        Input: Current Node co-ordinates
        Output: Boolean
        """
        radius_sq = np.square(current_state[0] - self.goal[0]) + np.square(current_state[1] - self.goal[1])
        return  radius_sq < thresh_radius**2

    def _setup_world(self):
        fig, self.ax = plt.subplots(figsize = (5, 5))
        plt.tight_layout()
        self.ax.set(xlim=(0, 10), ylim = (0,10))
        self.ax = self.plotObstacles(self.ax)
        
        self.ax.set_aspect("equal")
        self.ax.grid()
        # self.ax.xlim(0,100);self.ax.ylim(0,1)
        self.ax.set_title('A star differential constraints',fontsize=10)
    
    def visualizeSearch(self):
        self.reset()
        actions, _, _  = self.search()
        if len(actions) ==0: 
            print("[ERROR] Cannot visualize without a correct path")
            return False
        
        self.reset()
        
        #### Perform search to obtain paths ####
        self._setup_world()
        self.visualize = True
        actions, waypoints, points  = self.search()
        time.sleep(0.1)
        # search is done. 

        # Plot waypoints from start to goal.  
        assert len(waypoints) ==  len(actions)+1
        for idx in range(len(waypoints)-1):
            state = waypoints[idx]
            action = actions[idx] 
            self.nextPose(state, action, path_mode=True)
            time.sleep(0.05)
        return True

if __name__ == "__main__":
    Parser = argparse.ArgumentParser()
    # values are 10x to pass as integer strings and later convertedd within 0-10.0
    Parser.add_argument('--init_state', default="3,3,0", help='initial state of the puzzle')
    Parser.add_argument('--goal_state', default="9,9,0", help='goal state to be reached')
    Parser.add_argument('--rpms', default="15, 10", help='robot wheel rpms')
    Parser.add_argument('--clr', default="0.1", help='clearence')
    args = Parser.parse_args()

 
    # SETUP
    START = utils.string_to_int(args.init_state)
    GOAL = utils.string_to_int(args.goal_state)
    rpm1, rpm2 = utils.string_to_int(args.rpms)
    clearance = float(args.clr)

    robot = Robot(rpm1, rpm2, clearance)
    planner = Astar_DDC(robot, START, GOAL, clearance)

    ret = planner.visualizeSearch()

    if ret:
        utils.createMovie("./results/", video_name= args.init_state, fps=10)
        # utils.deleteFolder("./results")