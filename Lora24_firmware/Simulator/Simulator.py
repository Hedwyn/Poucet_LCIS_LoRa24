import numpy
import math
import random
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import abc

DEFAULT_ROBUSTNESS = 0.5


class Position:
    def __init__(self, x,y, z = 0):
        self.x = x
        self.y = y
        self.z = z
    
    def compute_distance_to(self, Position):
        return(math.sqrt( pow(self.x - Position.x, 2) + pow(self.y - Position.y, 2) + pow(self.z - Position.z, 2)))

    def to_string(self, format_digits = None, in_2d = False):
        if format_digits:
            pos_str = "x: " + str(self.x)[:format_digits] + " y: " + str(self.y)[:format_digits] 
            if in_2d:
                pos_str += " z: " + str(self.z)[:format_digits]
        else:
            pos_str = "x: " + str(self.x) + " y: " + str(self.y)
            if in_2d:
                pos_str += " z: " + str(self.z)
        return(pos_str)


   
class Node:
    """Represents a given device with a variable geographical position.
    -- Attributes
    position: coordinates tuples as (x,y) in 2D or (x,y,z) in 3D
    technos: list of technologies supported by the device"""
    
    def __init__(self, name = "", real_position = None, measured_position = None, technos = []):
        self.name = name
        self.real_position = real_position
        self.measured_position = measured_position
        self.technos = technos
        self.particles = []
        self.rangings = {}
    
    def generate_random_coordinates(self, map):
        x = random.uniform(0, map.width)
        y = random.uniform(0, map.length)
        if not(self.real_position):
            self.real_position = Position(x, y, 0)
        else:
            self.real_position.x = x
            self.real_position.y = y
            self.real_position.z = 0

class Map:
    """Represents the 2D map covered by the IPS"""
    def __init__(self, width, length):
        self.width = width
        self.length = length
    
class IPS:
    """An instance of an indoor position system"""
    def __init__(self, map, total_nodes):
        self.map = map
        self.nodes = []
        for n in range(total_nodes):
            new_node = Node("*** Node " + str(n))
            new_node.generate_random_coordinates(self.map)
            self.nodes.append(new_node)
    
    def display_nodes_real(self):
        for node in self.nodes:
            print(node.name + ":\n" + node.real_position.to_string() + "\n")
        

class Solver(ABC):
    @staticmethod
    @abstractmethod
    def compute_positions(ips):
        raise NotImplementedError
    
    @staticmethod
    @abstractmethod
    def compute_position(node):
        raise NotImplementedError

class WeightedCentroid(Solver):
    @staticmethod   
    def compute_positon(node):
        intermediate_solutions = []
        for anchor in node.rangings:
            distance = node.rangings[anchor]

    @staticmethod
    def trilateration_2d(pos1, r1, pos2, r2):  
        # getting the distance between both anchors
        base = pos1.compute_distance_to(pos2)

        # checking that anchors do not share the same coordinates
        if base == 0:
            return

        dx = ( pow(r1,2) - pow(r2,2) + pow(base,2) ) / ( 2 * base)

        # if the rangings have been perturbated a negative solution may be obtained
        # zeroing dx and dy in that case

        if dx < 0:
            dx = 0

        dy = pow(r1,2) - pow(dx,2)
        if (dy < 0):
            dy = 0
        else:
            dy = math.sqrt(dy)

        # coordinate changes -> calculating the coordiantes in the global cartesian system
        # vect_o denotes the vector from the origin to anchor 1;
        # vect_a denotes the vector from anchor 1 to the tag

        # z is assumed to be 0
        # calculations are exclusively based on x,y

        # calculating vect_base_x and vect_base_y coordinates in the global cartesian system
        vect_base_x  = [(pos2.x - pos1.x) / base,(pos2.y - pos1.y) / base ]
        vect_base_y = [-vect_base_x[1], vect_base_x[0]]
        vect_o = [pos1.x, pos1.y]

        # computing first solution
        vect_a = [dx, dy]
        ##print("vect_a" + str(vect_a))
        s1 = [vect_base_x[0] * vect_a[0] + vect_base_y[0] * vect_a[1]   , vect_base_x[1] * vect_a[0] + vect_base_y[1] * vect_a[1] ]
        ##print("s1" + str(s1))
        s1[0]+= vect_o[0]
        s1[1]+= vect_o[1]


        # computing second solution
        vect_a = [dx, -dy]
        vect_a = [dx, dy]
        s2 = [vect_base_x[0] * vect_a[0] + vect_base_y[0] * vect_a[1] , vect_base_x[1] * vect_a[0] + vect_base_y[1] * vect_a[1] ]
        s2[0]+= vect_o[0]
        s2[1]+= vect_o[1]

        return( [Position(s1[0], s1[1]), Position(s2[0], s2[1]) ])

  
class Particle:
    """Represents a potential position of a node"""
    def __init__(self, node, position = None, techno = None):
        self.node = node
        self.position = position
        self.techno = techno
        self.previous = None
        self.next = None

    def get_error(self):
        return(self.node.position.compute_distance_to(self.position))
    
    def link(self, particle):
        self.previous = particle
        particle.next = self

    def get_particle_chain(self):
        chain = []
        particle = self
        while (particle):
            chain.append(particle)
            particle = particle.previous()

class RenderedNode:
    """A graphical representation of a node"""
    def __init__(self, node, shape = 'o', color = None):
        self.node = node
        if color:
            self.color = color
        else:
            self.color = random.choice([c for c in mcolors.CSS4_COLORS])          

        self.shape = shape
    
    def get_real_position(self):
        return(self.node.real_position)


class GUI:
    """Graphical interface for the IPS. Displays the map, nodes, particles, etc."""
    def __init__(self, ips):
        self.ips = ips
        self.fig = plt.figure()
        self.axe = self.fig.add_subplot(111)
        self.axe.set_xlim(0, self.ips.map.width)
        self.axe.set_ylim(0, self.ips.map.length)  
        self.nodes = []
        
        for node in self.ips.nodes:
            self.nodes.append(RenderedNode(node))
    
    def display_node_real(self, node):
            self.axe.scatter([node.get_real_position().x,], [node.get_real_position().y,], c = node.color, marker = node.shape)

    def display_all_nodes_real(self, delay = 1):
        for node in self.nodes:
            self.display_node_real(node)             
        plt.draw()
        plt.pause(1)
       
class RadioProtocol:
    """Represents a radio ranging technology
    -- Attributes
    accuracy: average error expected from the technology, in meters"
    precision: typical standard deviation of the ranging measurements, in meters"
    frequency: number of localization slots per second
    max_range: typical maximum range of the technology, in meters"""
    def __init__(self, accuracy, precision, frequency, max_range, robustness = DEFAULT_ROBUSTNESS):
        self.accuracy = accuracy
        self.precision = precision
        self.frequency = frequency
        self.max_range = max_range
        self.robustness = robustness
    


if __name__ == "__main__":
    random.seed()
    p1 = Position(1, 2, 4)
    p2 = Position(3, 5, 7)
    print(p1.compute_distance_to(p2))

    my_map = Map(40, 40)
    ips = IPS(my_map, 10)
    ips.display_nodes_real()
    gui = GUI(ips)
    gui.display_all_nodes_real()
    plt.show()


