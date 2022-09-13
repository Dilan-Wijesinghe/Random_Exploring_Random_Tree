# Imports 
from pickletools import uint8
import random 
import numpy as np
from numpy import linalg as LA

class Node:
    def __init__(self,p_node=np.zeros(2),c_node=np.zeros(2),pos=np.zeros(2)):
        self.p_node = p_node # Position of Parent Node
        self.c_node = c_node # Position of Child Node
        self.pos = pos # Position of this Node

class RRT:
    def __init__(self,K,delta,D,q_init):
        self.K = K # Num Vertices
        self.delta = delta # Incremental Dist
        self.D = D # Planning Domain   
        self.q_init = q_init # Inital Position/Configuration

    def expand(self):
        G = np.empty()           # G is a list of vertices
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
        rand_pos = Node(pos = (random.randint(0,self.D[0]), random.randint(0,self.D[1])))
        print(type(rand_pos)) # Confirming Tuple Type
        return rand_pos

    def nearest_vertex(self, GivenNode, Graph):
        # Graph is a list of Nodes
        pos = GivenNode.pos # Gets positon as np.array
        dists = np.empty()
        for node in Graph:
            euc_dist = LA.norm(pos - node)
            dists.append(euc_dist)
        
        # print("Index is:", np.argmin(dists))
        # print("Node is:", Graph[np.argmin(dists)].pos) # Prints pos of nearest node
        return Graph[np.argmin(dists)] # Should return Node with smallest distance
        # Might get into trouble if euc_dists are equal
    
    """ Will return a Node with the new configuration """
    def new_config(self, NearestNode, RandomNode):
        # self.delta needed here
        dir = RandomNode - NearestNode # Element-wise subtraction of nodes 
        dir = dir / self.mag(dir) # Get Unit Vector
        dir = dir * self.delta # Multiply by Delta

        print("Vector to add:", dir)
        
        # Returns new np.array with shape (x,y)
        # Move delta many spaces towards direction 
        Child = Node(p_node=NearestNode, pos=np.array(dir)) # Generate new node, NearestNode as parent
        NearestNode.c_node = Child
        return NearestNode, Child
    
    def mag(self,vector: np.array):
        return np.sqrt(vector.dot(vector))

TestRRT = RRT(10,1,np.zeros((100,100)),Node(pos=np.array([50,50])))
vec = np.array([4,3])
print(np.sqrt(vec.dot(vec)))
print(vec / TestRRT.mag(vec))