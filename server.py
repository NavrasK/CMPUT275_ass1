#   Assignment 1 Part 1
#   By Jesse Goertzen (1505959) and Navras Kamal (1505463)

import math  # For math.sqrt()
from graph import Graph
from binary_heap import BinaryHeap
<<<<<<< HEAD
from serial import Serial
from time import sleep
from enum import Enum, auto

=======
import sys
import collections
import bisect
>>>>>>> 9e32d33dd444c8e00e85368bb10d1fc633072330

# Modified solution to Exercise 3 by Jesse Goertzen
def load_edmonton_graph(filename):
    g = Graph()
    location = dict()

    # Open the data file
    graphFile = open(filename, 'r')
    lines = graphFile.readlines()

    for line in lines:
        # Split at each comma, data stored comma-separated
        line = line.split(',')

        # Depending on the first character, treat input as vertex or edge
        if line[0] == 'V':  # Vertex
            # Add the vertex to the graph, and store the coordinates of the vertex
            g.add_vertex(int(line[1]))
            coords = (int(float(line[2]) * 100000), int(float(line[3]) * 100000))
            location[int(line[1])] = coords

        if line[0] == 'E':  # Edge
            g.add_edge((int(line[1]), int(line[2])))

    # Don't forget to close the file!
    graphFile.close()

    return g, location


def get_path(reached, start, end):
    #  get_path function provided in the useful functions tar file

    if end not in reached:
        return []  # unreachable

    # Build the path in reverse order, starting at the end
    path = [end]

    while end != start:
        end = reached[end]  # step to the vertex old end was reached by
        path.append(end)  # add to the path

    path.reverse()  # reverse the path to go from start to end
    return path


class CostDistance:
    # Class used to store the location dict which contains the coordinates of each vertex
    def __init__(self, location):
        # Initialize an instance of the class with the locations of the graph
        self.locations = location

    # e is a pair of vertices (u, v)
    # return the Euclidean distance between the two
    def distance(self, e):
        start, end = self.locations[e[0]], self.locations[e[1]]  # Get coordinates of start and end
        xdiff, ydiff = ((start[0] - end[0])**2), ((start[1] - end[1])**2)
        return math.sqrt(xdiff + ydiff)

    # distance from an arbitrary point to a vertex in the graph
    # v is a key for a vertex in the graph, r is a tuple of coordinates
    def dist2Vertex(self, v, r):
        start, end = r, self.locations[v]
        xdiff, ydiff = ((start[0] - end[0])**2), ((start[1] - end[1])**2)
        return math.sqrt(xdiff + ydiff)


def least_cost_path(graph, start, dest, cost):
    # Implementation of Dijkstra's algorithm
    events = BinaryHeap()
    reached = dict()
    events.insert((start, start), 0)  # Begin at time 0, at the start vertex

    while events:
        edge, time = events.popmin()  # Get next burnt vertex
        if edge[1] not in reached:  # If the destination is not been reached
            reached[edge[1]] = edge[0]  # Keep track of where we came from
            for w in graph.neighbours(edge[1]):  # Burn the neighbours!!!!
                events.insert(((edge[1]), w), (time + cost.distance((edge[1], w))))  # Add the fuse

    return get_path(reached, start, dest)  # return the path


# Function to find the nearest vertex to a location on the map
def nearestVertex(req, location):
    cost = CostDistance(location)  # considering moving this out of the function and into main
    min = float('inf')
    key = -1

    # cycle through vertices, finding the nearest vertex
    for v in location:
        diff = cost.dist2Vertex(v, req)
        if diff < min:
            min, key = diff, v

    return key


# Read from the serial, formatting the string
def serialIn():
    with Serial("/dev/ttyACM0", baudrate=9600, timeout=1) as ser:
        read = ser.readline().decode("ASCII").rstrip("\r\n").split(' ')
    return read


# Finite state machine that processes requests from the arduino client
def finite_state_machine(edmonton_graph, location):
    class states(Enum):
        REQ = auto()  # Waiting on request
        ACK = auto()  # Waiting on acknowledgement
        END = auto()  # Send end key

    state = states.REQ
    path = []

    with Serial("/dev/ttyACM0", baudrate=9600, timeout=1) as ser:
        while True:
            response = serialIn()

            if state == states.REQ and response[0] == 'R':
                # Read coordinates of the start and destination
                start = (int(response[1]), int(response[2]))
                end = (int(response[3]), int(response[4]))

                # Find nearest vertices for the start and end points
                startKey, endKey = nearestVertex(start, location), nearestVertex(end, location)

                # Find the shortest path
                path = least_cost_path(edmonton_graph, startKey, endKey, cost).reverse()

                # Create formatted output string for the client
                length = 'N ' + ' ' + str(len(path)) + '\n'
                ser.write(length)

                state = states.ACK  # Proceed to wait on acknowledgement

                if len(path) == 0:  # Wait for another request if there is no path
                    state = states.REQ

            if state == states.ACK and response[0] == 'A':
                if len(path) == 0:  # no more waypoints, send end notification
                    state = states.END
                    continue

                next = path.pop()  # Get next waypoint

                # Format output to the client
                waypoint = 'W' + ' ' + str(location[next][0]) + ' ' + str(location[next][1]) + '\n'
                ser.write(waypoint)

            if state == states.END:
                ser.write('E\n')
                state = states.REQ


def checkRcpt():
    rcpt = input()
    if rcpt != "A":
        raise InputError('Invalid Receipt')

def minDist(location, start, end):
    tempStartDist = float('inf')
    tempEndDist = float('inf')
    for k in location:
        ds = math.sqrt((location[k][0]-start[0])**2 + (location[k][1]-start[1])**2) #delta(d) = sqrt((distanceX)^2+(distanceY)^2)
        de = math.sqrt((location[k][0]-end[0])**2 + (location[k][1]-end[1])**2)
        if ds < tempStartDist: 
            tempStartDist = ds
            startKey = k
        if de < tempEndDist:
            tempEndDist = de
            endKey = k
    return startKey, endKey


if __name__ == "__main__":
    edmonton_graph, location = load_edmonton_graph("edmonton-roads-2.0.1.txt")
<<<<<<< HEAD

    finite_state_machine(edmonton_graph, location)
=======
    request = input()
    request = request.split(' ')
    if request[0] != "R":
        raise InputError('Invalid request format.')
    else:
        start = (float(request[1])/1000000, float(request[2])/1000000)
        end = (float(request[3])/1000000, float(request[4])/1000000)
        #TODO Figure out how to find the closest value in the dict if it isn't exact
        startKey, endKey = minDist(location, start, end)
        print("start: ", startKey)
        print("end: ", endKey)



    #R 5365486 -11333915 5364728 -11335891
    #R 53430996 -113491331 53461225 -113617217
    #^ 29577354 36397020
>>>>>>> 9e32d33dd444c8e00e85368bb10d1fc633072330
