# Imports 
from itertools import combinations_with_replacement
from pickletools import uint8
import random
from turtle import color
import numpy as np
from numpy import linalg as LA
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import matplotlib.colors as colors

def checkSpawnCollision(c,r,node):
    h = c[0]
    k = c[1]
    x = node[0]
    y = node[1]
    d = np.sqrt((h-x)**2 + (k-y)**2)
    return d <= r # If d <= radius, then it is within the SpawnCircle range, must remake circle

def spawnCircle(start_node, goal_node):
    circleMade = False
    while not circleMade:
        c = [random.uniform(0,100), random.uniform(0,100)]
        r = random.uniform(1,20)
        if not checkSpawnCollision(c,r,node=start_node.pos):
            if not checkSpawnCollision(c,r,node=goal_node):
                newCircle = Circle(c=c, r=r) 
                circleMade = True
    return newCircle

def randomLoc():
    # Spawns a Random Start and Goal Location
    validLocs = False
    while not validLocs:
        goal_loc = [random.uniform(0,100), random.uniform(0,100)]
        start_loc = [random.uniform(0,100), random.uniform(0,100)]
        if goal_loc != start_loc:
            validLocs = True
    
    return goal_loc, start_loc


class Node:
    def __init__(self,p_node=np.zeros(2),c_node=np.zeros(2),pos=np.zeros(2)):
        self.p_node = p_node # Position of Parent Node
        self.c_node = c_node # Position of Child Node
        self.pos = pos # Position of this Node
    
class Circle:
    def __init__(self,c,r):
        self.c = c # tuple of (x,y)
        self.r = r # radius of circle

    def makeCircle(self):
        return plt.Circle(self.c,self.r) # Plots the Circle

class Obstacles:
    def __init__(self, circles):
        # Class Variables 
        self.circles = circles # List of Circles
    
    def collision(self, p1, p2):
        # Checks for a collision between 
        # p1 = [x1,y1], p2 = [x2,y2]
        A = p1[1] - p2[1] # y1 - y2
        B = p2[0] - p1[0] # x2 - x1
        C = (p1[0] - p2[0])*p1[1] + (p2[1] - p1[1])*p1[0] # (x1 - x2)y1 + (y2 - y1)x1
        collide = False
        for circ in self.circles: # Check for a collision between each circle
            numer = np.abs(A*circ.c[0] + B*circ.c[1] + C) # |a*x_c + b*y_c + c|
            denom = np.sqrt((A)**2 + (B)**2) # sqrt(a^2 + b^2)
            d = numer / denom
            # print(f"d is {d}")
            if d <= circ.r:
                collide = True
        return collide


class RRT:
    def __init__(self,K,delta,D,q_init,q_end,obstacles):
        self.K = K # Num Vertices
        self.delta = delta # Incremental Dist
        self.D = D # Planning Domain   
        self.q_init = q_init # Inital Position/Configuration (Node type)
        self.obstacles = obstacles # Initialize RRT with some obstacles
        self.goal = q_end # End Location we want to go!
        self.path_to_goal = []

    def expand(self):
        G = []             # G is a list of vertices
        G.append(self.q_init) # Initialize G with q_init
        for i in range(0,self.K): # Repeat K times
            q_rand = self.rand_config() # Generates a rand pos in D
            q_near = self.nearest_vertex(q_rand, G) # Find closest node
            q_new, collide = self.new_config(q_near, q_rand)
            if not collide:
                # Add a vertex between q_near and q_new (Built-in to new_config)
                # print(f"Adding Vertex: {i}")
                G.append(q_new)
                if self.checkGoal(q_new):
                    print("Goal has been reached!")
                    # Start the Tree
                    
                    self.path_to_goal.append(Node(p_node=Node(pos=q_new.pos), pos=self.goal)) # Append Goal, parent is q_new
                    print("first node", self.path_to_goal[0].pos)
                    self.makePath(q_new)
                    self.grow_tree(G, goal_found=True)
                    return G
        return G

    def rand_config(self):
        # Generate Node with random position in D
        rand_pos = Node(pos = (random.uniform(0,self.D[0]), random.uniform(0,self.D[1])))
        # print(type(rand_pos)) # Confirming Node Type
        # print(rand_pos.pos)
        return rand_pos

    def nearest_vertex(self, GivenNode: Node, Graph):
        # Graph is a list of Nodes
        pos = GivenNode.pos # Gets positon as np.array
        dists = []
        for node in Graph:
            # print("Found a Node!:", node.pos)
            euc_dist = LA.norm(pos - node.pos)
            dists.append(euc_dist)
        dists = np.array(dists)
     
        # print("Index is:", np.argmin(dists))
        # print("Node is:", Graph[np.argmin(dists)].pos) # Prints pos of nearest node
        return Graph[np.argmin(dists)] # Should return Node with smallest distance
        # Might get into trouble if euc_dists are equal
    
    """ Will return a Node with the new configuration """
    def new_config(self, NearestNode, RandomNode):
        collide = False
        dir = RandomNode.pos - NearestNode.pos # Element-wise subtraction of nodes 
        dir = dir / self.mag(dir) # Get Unit Vector
        dir = dir * self.delta # Multiply by Delta

        # print("Vector to add:", dir)
        new_coords = NearestNode.pos + dir
        # print(type(new_coords), type(NearestNode.pos)) # Checking for numpy array type

        # TODO: Check if a collision occurs with this new_coords point!
        if self.obstacles.collision(NearestNode.pos, new_coords): # if True 
            # Then there E an obstacle. Do NOT make a child.
            # print("Cannot Make Child, there is an Obstacle in the way!")
            collide = True
            Child = Node()
            return Child, collide

        Child = Node(p_node=NearestNode, pos=np.array(new_coords)) # Generate new node, NearestNode as parent
        NearestNode.c_node = Child
        # print(f"My child is: {NearestNode.c_node.pos}")
        return Child, collide
    
    def checkGoal(self, q_new):
        # TODO: checkGoal should check if the goal is within a certain radius (size delta) away from
        # the newest vertex, q_new
        h = self.goal[0]
        k = self.goal[1]
        x = q_new.pos[0]
        y = q_new.pos[1]
        # radius = self.delta
        radius = 10
        d = np.sqrt((h-x)**2 + (k-y)**2)
        return d <= radius # If d <= radius, then it is near the goal

    def makePath(self, q_new):
        # Return Nothing, since we are writing to path_to_goal directly
        currNode = q_new
        start_found = False
        # self.path_to_goal.append(Node(pos=self.goal)) # Skip this since we did it before calling function
        while not start_found:
            if (currNode.p_node.pos == self.q_init.pos).all():
                print("Start is Found!")
                self.path_to_goal.append(currNode)
                # self.path_to_goal.append(currNode.p_node)
                start_found = True
            else:
                self.path_to_goal.append(currNode)
                currNode = currNode.p_node
        return 
    
    def mag(self,vector: np.array):
        return np.sqrt(vector.dot(vector))

    def over_the_edge(self, node):
        line = ([node.p_node.pos[0], node.p_node.pos[1]], [node.pos[0], node.pos[1]])
        return line

    def grow_tree(self, Graph, goal_found=False):
        x = []
        y = []
        segs = []
        cnt = 0
        for node in Graph:
            if cnt > 0:
                line = self.over_the_edge(node)
                segs.append(line)
                # print(f"My line is : {line}")
            x.append(node.pos[0])
            y.append(node.pos[1])
            cnt += 1
        
        path_segs = []
        path_x = []
        path_y = []
        if goal_found:
            print("Goal Found! Printing Path now")
            for node in self.path_to_goal:
                path_line = self.over_the_edge(node)
                print(path_line)
                path_segs.append(path_line)
                path_x.append(node.pos[0])
                path_y.append(node.pos[1])
                
        
        f, ax = plt.subplots()
        # Line Collections, for connecting Vertices
        DarkOg = colors.to_rgb('darkorange')
        line_segs = LineCollection(segs, colors=DarkOg) # Style if you want here
        ax.add_collection(line_segs)
        path_line_segs = LineCollection(path_segs, colors=DarkOg)
        ax.add_collection(path_line_segs)
    
        # Circles!
        for circle in self.obstacles.circles:
            circle_to_plot = circle.makeCircle()
            ax.add_patch(circle_to_plot)
        ax.set_aspect('equal')
        Red = colors.to_rgb('red')
        ax.plot(goal[0], goal[1], marker="o", color=Red)

        x = np.array(x)
        y = np.array(y)
        # path_x = np.array(path_x)
        # path_y = np.array(path_x)
        # print(points)
        plt.scatter(x,y) # Make Circles Smaller
        plt.show()
    

# -------------------------------------------------------------------------------------------------


# node = Node(p_node=Node(pos=np.array([50,50])), pos=([3,4]))
# print(node.p_node.pos)
# q_init = Node(pos=np.array([51,50]))
# print(type(node.p_node.pos), type(q_init.pos))

# if (node.p_node.pos==q_init.pos).all():
#     print("True")

# Collision Tests
# p1 = np.array([96,70])
# p2 = np.array([87,75])
# p3 = np.array([90,70])
# p4 = np.array([96,75])

# if Obs.collision(p1=p2, p2=p4) == True:
#     print("Collision Exists!")

# Create Obstacles
# TODO: Randomize Obstacles, Start, and Goal
# Make Sure Obstacles do not contain Start and Goal.
goal_loc, start_loc = randomLoc()
goal = goal_loc
start = Node(pos=np.array(start_loc))

circle1 = spawnCircle(start, goal)
circle2 = spawnCircle(start, goal)

# circle1 = Circle((45,39),10) # Can change colors if wanted
# circle2 = Circle((55,56),5)
circles = [circle1, circle2]
Obs = Obstacles(circles=circles)

# start = Node(pos=np.array([50,50]))
# goal = np.array([30,70])

TestRRT = RRT(K=1000,delta=1,D=np.array([100,100]),q_init=start,q_end=goal, obstacles=Obs)
GraphTest = TestRRT.expand()
# TestRRT.grow_tree(Graph=GraphTest)

# not every node has a child, but every node does have a parent
# for node in GraphTest:
#     print(f"These are my children! {node.p_node}")

# Magnitude Tests
# vec = np.array([4,3])
# print(np.sqrt(vec.dot(vec)))
# print(vec / TestRRT.mag(vec))

# Plotting Test
# points = (np.random.rand(100), np.random.rand(100))
# print("Point Size:", points)
# plt.scatter(points[0], points[1])
# plt.show()
