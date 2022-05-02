import pygame
import sys

from heapq import heappush, heappop
import math
import time
import numpy as np
import math


import os
import cv2
import shutil
from glob import glob
from tqdm import tqdm
import imageio



def createFolder(path):
    if not os.path.exists(path):
        os.makedirs(path)

def deleteFolder(path):
    if os.path.exists(path):
        shutil.rmtree(path)


def remove_file(file):
    try:
        os.remove(file)
    except OSError as e:
        print("Error: %s : %s" % (file, e.strerror))

def createMovie(path):
    image_folder = path
    video_name = 'simulation_video.mp4'

    images = sorted(glob("results/*.png"))
    frame = cv2.imread(os.path.join(images[0]))
    height, width, channels = frame.shape
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(video_name, fourcc, 30, (width,height))
    imgs = []
    for i in tqdm(range(len(images))):
        imgs.append(imageio.imread(images[i]))
        image = cv2.imread(images[i])
        if i==0:
            h, w, _ = image.shape
        video.write(image)

    for i in range(20):
        imgs.append(imageio.imread(images[len(images)-1]))
    imageio.mimsave('simulation_video.gif', imgs, fps=30)
    cv2.destroyAllWindows()
    video.release()




from heapq import heappop
from matplotlib.pyplot import draw
import pygame
import sys
import time 
import numpy as np
import math

GREEN_LIGHT = (76, 255, 130)
RED_LIGHT = (240, 101, 101)
PURPLE = (126, 67, 222)
ORANGE = (130,110,70)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
RED = (255,20,20)
GRAY = (220, 220, 220)
BLUE = (0, 20, 108)
CYAN = (136, 255, 196)
BLACK = (0, 0, 0)
CLEARNCE = 10

START = [0,0,math.pi/7]
GOAL = [280,70,0]
class Arena:
    class Node:
        """
        Node class contains details of a node like location, connection 
        with nearby nodes, parent nodes, distance from start point 
        """
        def __init__(self, x ,y, theta, parent=None):
            self.x = x
            self.y = y
            self.theta= theta
            self.costToCome = float('inf')
            self.parent = parent 
            self.x_thresh= 5
            self.y_thresh = 5

            #TODO: Implement final angle checking condition
            self.theta_threshold = math.pi/6

        def __lt__(self, other):
            return self.costToCome < other.costToCome
        
        def __eq__(self, other):
            if other==None:
                    return False
            return abs(self.x - other.x)<self.x_thresh and \
                abs(self.y - other.y)<self.y_thresh and abs(self.theta-other.theta)<self.theta_threshold
     
    def __init__(self):
        pygame.init()
        self.HEIGHT, self.WIDTH = 250, 400
        pygame.display.set_caption("A Star Algorithm - Path Planning on Car like Robot model")

        #### Create a canvas on which to display everything ####
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))

        #### Create a surface with the same size as the window ####
        # This 'background' surface will have the bottom left as origin.
        # All objects will be drawn in this 'bacground' surface.
        self.background = pygame.Surface((self.WIDTH, self.HEIGHT))
        #### setup variables ####
        self.nodes = {}
        self.SCALE = 2
        self.visited=np.zeros([self.WIDTH*self.SCALE, self.HEIGHT*self.SCALE, 12])
        self.start_location = self.Node(0,0,0) 
        self.start_location.costToCome=0
        self.start_location.parent= self.Node(0,0,0)
        self.open_nodes={}
        self.open_nodes[(self.start_location.x,self.start_location.y)]=self.start_location
        self.obstacle_nodes = {}
        self.obstacle_nodes_clearance = {}
        self.goal_location = self.Node(self.WIDTH-5,self.HEIGHT-5,0)
        # self.goal_location = self.Node(150,190)

        # Manually assign start and goal locations.
        start_x, start_y, start_theta = START
        goal_x, goal_y, goal_theta = GOAL
        # or Get details of start and goal node from user input:
        start_x, start_y, start_theta = input("Enter start node information[x,y,theta(randians)] seperated by space( ex: 0 0 0 ): ").split()
        goal_x, goal_y, goal_theta = input("Enter goal node information[x,y,theta(radians)] seperated by space( ex: 280 70 0  ): ").split()
        global CLEARNCE
        CLEARNCE = int(input("Input Clearance value(ex: 10): "))
        radius = int(input("Input Robot radius(ex: 1): "))
        self.start_location.x, self.start_location.y,self.start_location.theta  = int(start_x),int(start_y), float(start_theta)
        self.goal_location.x, self.goal_location.y, self.goal_location.theta = int(goal_x),int(goal_y), float(goal_theta)

        self.goal_node=None
        self.selectStart = True
        self.obstacles = self.createObstacles()
        self.obstacles_clearance = {}
        deleteFolder('results')
        createFolder('results')
        self.start_time = time.time()
        # front = [ total_cost, cost, current_node, previous node ]
        self.front = [ (0.0001+self.distance(self.start_location,self.goal_location), 0.00001, self.start_location, self.start_location) ]
        self.cameFrom = {}
        self.latestnodepop=[]

    def distance(self, start, goal):
        """
        Returns Euclidean Distance between start and goal.
        """
        return math.sqrt(math.pow(start.x-goal.x,2)+math.pow(start.y-goal.y,2))    
        # return (goal.x-start.x)+(goal.y-start.y)

    def updateEvents(self):

        # proceed events
        for event in pygame.event.get():

            # handle MOUSEBUTTONUP
            if event.type == pygame.MOUSEBUTTONUP:
                pos = pygame.mouse.get_pos()

                # vertical flip - for converting to 'background' surface's coordinate system
                pos = (pos[0], self.HEIGHT - pos[1])

                if self.selectStart:
                    self.start_location.x, self.start_location.y = pos
                    print("Start location placed at: ", pos)
                    self.selectStart = False
                else:
                    self.goal_location.x, self.goal_location.y = pos
                    print("Goal location placed at: ", pos)
                    self.selectStart = True

            # Exit if window is closed
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        
        if self.isCollision(self.start_location.x, self.start_location.y):
            print("[ERROR] Starting point is inside the obstacle!")
            return 
        if self.isCollision(self.goal_location.x, self.goal_location.y):
            print("[ERROR] Ending point is inside the obstacle!")
            return 
        if(self.isValid(self.start_location)):
            return 
        if(self.isValid(self.goal_location)):
            return

    def isValid(self,node): 
        return (0<=node.x<self.WIDTH) and (0<=node.y<self.HEIGHT) \
        and (0<=node.x<self.WIDTH) and (0<=node.y<self.HEIGHT)

    def drawAll(self):
        self.background.fill((0, 0, 0))
        self.drawNode()
        self.drawStartLocation()
        self.drawGoalLocation()
        self.drawPath()
        self.drawHeading()
        self.save()
        # time.sleep(0.01)

    def drawNode(self):

        # visited = np.array(np.squeeze(np.where(self.visited==1)))
        # for v in visited.T:
        #     color = RED_LIGHT
        #     if np.array(v).shape:
        #         pygame.draw.rect(self.background, color, (v[0]//self.SCALE, v[1]//self.SCALE, 3, 3))


        # for _,node in self.open_nodes.items():
        #     color = YELLOW
        #     pygame.draw.rect(self.background, color, (node.x, node.y, 1, 1))

        for _,node in self.obstacle_nodes.items():
            color = WHITE
            pygame.draw.rect(self.background, color, (node.x, node.y, 1, 1))
        
        for _,node in self.obstacles_clearance.items():
            color = YELLOW
            pygame.draw.rect(self.background, color, (node.x, node.y, 1, 1))


        for data in self.front:
            color = YELLOW
            pygame.draw.line(self.background, color, (data[3].x, data[3].y), (data[2].x, data[2].y), width=1)
            pygame.draw.circle(self.background, color, (data[2].x, data[2].y), 3, width=1)

        for latestnodepop in self.latestnodepop:
            color = GREEN_LIGHT
            pygame.draw.line(self.background, color, (latestnodepop[3].x, latestnodepop[3].y), (latestnodepop[2].x, latestnodepop[2].y), width=1)
            pygame.draw.circle(self.background, color, (latestnodepop[2].x, latestnodepop[2].y), 3, width=1)

        self.screen.blit(pygame.transform.flip(self.background, False, True), dest=(0, 0))
        pygame.display.update()
        pass

    def drawHeading(self):
        r=40

        nodes = [self.front[0][2],  self.start_location,  self.goal_location]
        colors = [PURPLE, RED_LIGHT, GREEN_LIGHT]
        for node, color in zip(nodes, colors):
            pygame.draw.line(self.background, color, (node.x, node.y), (node.x+r*math.cos(node.theta), node.y+r*math.sin(node.theta)), width=1)
            pygame.draw.circle(self.background, color, (node.x+r*math.cos(node.theta), node.y+r*math.sin(node.theta)), 5, width=1)
 

        if self.goal_node:
            node = self.goal_node
            pygame.draw.line(self.background, ORANGE, (node.x, node.y), (node.x+r*math.cos(node.theta), node.y+r*math.sin(node.theta)), width=1)
            pygame.draw.circle(self.background, ORANGE, (node.x+r*math.cos(node.theta), node.y+r*math.sin(node.theta)), 5, width=1)

        self.screen.blit(pygame.transform.flip(self.background, False, True), dest=(0, 0))
        pygame.display.update()

    def drawStartLocation(self):
        #### Populate the surface with Start location ####
        pygame.draw.rect(self.background, (255, 50, 50), (self.start_location.x, self.start_location.y, 5, 5))
        self.screen.blit(pygame.transform.flip(self.background, False, True), dest=(0, 0))
        pygame.display.update()

    def drawGoalLocation(self):
        #### Populate the surface with Goal Location ####
        pygame.draw.rect(self.background, (50, 255, 50), (self.goal_location.x, self.goal_location.y, 5, 5))
        self.screen.blit(pygame.transform.flip(self.background, False, True), dest=(0, 0))
        pygame.display.update()

    def drawPath(self):
        if not self.goal_node:
            return
        currentNode = self.goal_node
        while(currentNode.parent):
            pygame.draw.rect(self.background, PURPLE, (currentNode.x, currentNode.y, 5, 5))
            currentNode=currentNode.parent
        self.screen.blit(pygame.transform.flip(self.background, False, True), dest=(0, 0))
        pygame.display.update()

    def displayResults(self):
        self.stop_time = time.time()
        print("Time Taken to find the Goal: ",self.stop_time-self.start_time, " seconds")
        self.drawAll()
        print("Creating simulation video...")
        createMovie('results')
        input("Press Enter Exit")

    def save(self):
        file_name = "./results/image" + str(time.time()) +".png"
        pygame.image.save(self.screen, file_name)
            

    class Hexagon:
        def __init__(self, x, y, Dx):
            self.type = 'polygon'
            self.cx, self.cy = x, y
            a = Dx/np.sqrt(2) # side length
            DY_2 =  np.sqrt(a**2 - (Dx/2)**2)
            self.p1 = (x, y + DY_2)
            self.p2 = (x - Dx/2, y+DY_2/2)
            self.p3 = (x - Dx/2, y-DY_2/2) 
            self.p4 = (x, y - DY_2)
            self.p5 = (x + Dx/2, y-DY_2/2)
            self.p6 = (x + Dx/2, y+DY_2/2)
            self.points = np.array([self.p1, self.p2, self.p3, self.p4, self.p5, self.p6])
            print(f"Hexagon corner points: \n{self.points}")
            self.m12 = (self.p1[1]-self.p2[1])/(self.p1[0]-self.p2[0])
            self.b12 = -self.m12*self.p2[0] + self.p2[1]
            self.m23=0
            self.b23 = self.m23*self.p3[1] + self.p3[0] 
            self.m34 = (self.p3[1]-self.p4[1])/(self.p3[0]-self.p4[0])
            self.b34 = -self.m34*self.p4[0] + self.p4[1]
            self.m45 = (self.p4[1]-self.p5[1])/(self.p4[0]-self.p5[0])
            self.b45 = -self.m45*self.p5[0] + self.p5[1]
            self.m56=0
            self.b56 = self.m56*self.p5[1] + self.p5[0] 
            self.m61 = (self.p6[1]-self.p1[1])/(self.p6[0]-self.p1[0])
            self.b61 = -self.m61*self.p1[0] + self.p1[1]

        def isInside(self, x,y, clearance=True):     
            clearance_val = CLEARNCE if clearance else 0   
            side1 = (y-self.m12*x - self.b12 ) < 0 + clearance_val
            side2 = x - self.b23 > 0 - clearance_val
            side3 = (y-self.m34*x - self.b34 ) > 0 - clearance_val
            side4 = (y-self.m45*x - self.b45 ) > 0 - clearance_val
            side5 = x - self.b56 < 0 + clearance_val
            side6 = (y-self.m61*x - self.b61 ) < 0 + clearance_val
            return  side1 and side2 and side3 and side4 and side5 and side6 

    class Circle:
        def __init__(self, x, y, radius):
            self.type = 'circle'
            self.x, self.y = x,y
            self.radius = radius
            print(f"Circle at {x, y} with radius {radius}")

        def isInside(self, x,y, clearance=True):     
            clearance_val = CLEARNCE if clearance else 0   
            return  (x - self.x) **2 + (y- self.y)**2  < (self.radius+clearance_val)**2

    class Polygon:
        def __init__(self, *args):
            self.type = 'polygon'
            self.points = np.array(args)
            print(f"Polygon with corners at {self.points}")
            self.m12, self.m23, self.m34, self.m41 = -1.24, -3.2, 0.85, 0.32
            self.b12, self.b23, self.b34, self.b41 =  230,439, 112, 173

        def isInside(self, x,y, clearance=True):     
            clearance_val = CLEARNCE if clearance else 0   
            f1 = (y - self.m12* x - self.b12) > 0 - clearance_val
            f2 = (y - self.m23* x - self.b23) < 0 + 3*clearance_val  
            fmidleft = (y - (-0.1)* x - 189) < 0# + clearance_val 

            fmidright = (y - (-0.1)* x - 189) > 0# - clearance_val
            f3 = (y - self.m34* x - self.b34) > 0 - clearance_val
            f4 = (y - self.m41* x - self.b41) < 0 + clearance_val -5

            return f1 and f2 and fmidleft or fmidright and f4 and f3

    def createObstacles(self):
        circObstacle1 = Arena.Circle(10, 10, 5) 
        hexObstacle = Arena.Hexagon(200, 100, 70)      
        circObstacle = Arena.Circle(self.WIDTH-100, 185, 40)
        p1, p2, p3, p4 = (36, 185), (105, 100), (105-25, 180), (115, 210) 
        polygObstacle = Arena.Polygon(p1, p2, p3, p4)
        obstacleList=[]
        # obstacleList.append(circObstacle1)
        obstacleList.extend([circObstacle,hexObstacle,polygObstacle])
        return obstacleList
    
    def isCollision(self, x, y, clearance=True):
        states = []
        for obstacle in self.obstacles:
            states.append(obstacle.isInside(x, y, clearance)) 
        return any(states)








class AStar:

    def __init__(self,arena):
        self.stepsize = 15
        self.stepsize = int(input("Enter step size of movement in units (1<=L<=10): "))
        self.angles = [-math.pi/3, math.pi/3, -math.pi/6, math.pi/6, 0]
        self.recently_closed=[]    
        self.SCALE = arena.SCALE

    def distance(self, start, goal):
        return math.sqrt(math.pow(start.x-goal.x,2)+math.pow(start.y-goal.y,2))    
        # return (goal.x-start.x)+(goal.y-start.y)

    def cycleTheta(self,theta,delta_theta):
        theta_ = theta + delta_theta
        if theta_ >= 2*math.pi:
            theta_ = theta_ % math.pi
        elif theta_ < 0:
            theta_ = theta_ + 2*math.pi

        return theta_

    def roundtoscale(self, number):
        return round(number * self.SCALE) / self.SCALE

    def search(self, arena):
        solution_found = False

        #Loop through all the open nodes
        if arena.front:
            # pick mincost node from heap
            min_cost_node = heappop(arena.front)
            total_cost, cost, current_node, previous = min_cost_node
            
            # update the latest cache in Arena.
            arena.latestnodepop.append(min_cost_node)    
            
            # mark node visited in matrix
            current_node.theta = self.cycleTheta(current_node.theta,0)
            arena.visited[int(current_node.x*self.SCALE)][int(current_node.y*self.SCALE)][int(((current_node.theta/(2*math.pi))*360)//30)] = 1

            if current_node == arena.goal_location:
                arena.goal_node = current_node
                solution_found = True
            
            # Loop through all the possible actions            
                        
            angles=[]
            for count, deltaTheta in enumerate(self.angles):
                angles.append(self.cycleTheta(current_node.theta,deltaTheta))
            for theta_ in angles:

                # Create a new neighbor node.
                x_ = self.roundtoscale(current_node.x + self.stepsize*np.cos(theta_))
                y_ = self.roundtoscale(current_node.y + self.stepsize*np.sin(theta_))
                node = Arena.Node(x_, y_, theta_)
                node.parent=current_node

                # Check if the node is created inside the arena
                if(not arena.isValid(node)):
                    continue

                # Check if the newly created node lies inside any obstacles or already created nodes
                if (arena.visited[int(node.x*self.SCALE)][int(node.y*self.SCALE)][int(((node.theta/(2*math.pi))*360)//30)] or \
                        arena.isCollision(node.x,node.y,clearance=True)):
                    continue

                startcost = self.distance(arena.start_location, current_node)
                deltacost = self.distance(current_node, node) # distance between current node and new neighbor
                goalcost = self.distance(node, arena.goal_location)
                heappush(arena.front, (startcost+deltacost+goalcost, startcost+deltacost, node, current_node))
            
        return solution_found, arena

if __name__ == "__main__":
    arena = Arena()
    planner = AStar(arena)
    solution_found = False
    
    # Paint the obstacles node white
    for i in range(arena.WIDTH):
        for j in range(arena.HEIGHT):
            node = Arena.Node(i, j,0)
            if (arena.isCollision(i,j,clearance=False)):
                    arena.obstacle_nodes[(i, j)] = node
                    continue

            if (arena.isCollision(i,j,clearance=True)):
                arena.obstacles_clearance[(i, j)] = node
    
    arena.start_time = time.time()
    while(not solution_found): # your main loop
        # get all events
        arena.updateEvents()
        
        #Search A star
        solution_found, arena = planner.search(arena)

        # Update MAP - Pygame display
        arena.drawAll()

    arena.drawAll()
    input("Finished algorithm")
    arena.displayResults()
