# Imports 
from pickletools import uint8
import random 
import numpy as np
from numpy import linalg as LA
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

class Node:
    def __init__(self,p_node=np.zeros(2),c_node=np.zeros(2),pos=np.zeros(2)):
        self.p_node = p_node # Position of Parent Node
        self.c_node = c_node # Position of Child Node
        self.pos = pos # Position of this Node
    
class Circle:
    def __init__(self,c,r):
        self.c = c # np.array of (x,y)
        self.r = r # radius of circle

    def makeCircle(self):
        return 0


# class Obstacles:
#     def __init__(self):
#         # Class Variables


class RRT:
    def __init__(self,K,delta,D,q_init):
        self.K = K # Num Vertices
        self.delta = delta # Incremental Dist
        self.D = D # Planning Domain   
        self.q_init = q_init # Inital Position/Configuration (Node type)

    def expand(self):
        G = []             # G is a list of vertices
        G.append(self.q_init) # Initialize G with q_init
        for i in range(0,self.K): # Repeat K times
            q_rand = self.rand_config() # Generates a rand pos in D
            q_near = self.nearest_vertex(q_rand, G) # Find closest node
            q_new = self.new_config(q_near, q_rand)
            G.append(q_new)
            # Add a vertex between q_near and q_new (Built-in to new_config)
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
        # self.delta needed here
        dir = RandomNode.pos - NearestNode.pos # Element-wise subtraction of nodes 
        dir = dir / self.mag(dir) # Get Unit Vector
        dir = dir * self.delta # Multiply by Delta

        # print("Vector to add:", dir)
        new_coords = NearestNode.pos + dir
        # print(new_coords)
        # Returns new np.array with shape (x,y)
        # Move delta many spaces towards direction 
        Child = Node(p_node=NearestNode, pos=np.array(new_coords)) # Generate new node, NearestNode as parent
        NearestNode.c_node = Child
        # print(f"My child is: {NearestNode.c_node.pos}")
        return Child
    
    def mag(self,vector: np.array):
        return np.sqrt(vector.dot(vector))

    def over_the_edge(self, node):
        line = ([node.p_node.pos[0], node.p_node.pos[1]], [node.pos[0], node.pos[1]])
        return line

    def grow_tree(self, Graph):
        x = []
        y = []
        segs = []
        cnt = 0
        for node in Graph:
            if cnt > 0:
                print(node.p_node.pos)
                line = self.over_the_edge(node)
                segs.append(line)
                # print(f"My line is : {line}")
            x.append(node.pos[0])
            y.append(node.pos[1])
            cnt += 1
        
        f, ax = plt.subplots()
        # ax.set_xlim(0,self.D[0])
        # ax.set_ylim(0,self.D[1])
        line_segs = LineCollection(segs) # Style if you want here
        ax.add_collection(line_segs)

        circle1 = plt.Circle((10,10),10)
        ax.add_patch(circle1)
        ax.set_aspect('equal')
        x = np.array(x)
        y = np.array(y)
        # print(points)
        plt.scatter(x,y) # Make Circles Smaller
        plt.show()
    

TestRRT = RRT(K=100,delta=1,D=np.array([100,100]),q_init=Node(pos=np.array([50,50])))
GraphTest = TestRRT.expand()
TestRRT.grow_tree(Graph=GraphTest)



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

