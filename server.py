#   Assignment 1 Part 1
#   By Jesse Goertzen (1505959) and Navras Kamal (1505463)

import math  # For math.sqrt()
from graph import Graph
from binary_heap import BinaryHeap
from serial import Serial
from enum import Enum


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
        return []

    path = [end]

    print(path)

    while end != start:
        end = reached[end]
        path.append(end)

    path.reverse()

    print(path)
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
    print("Finding new nearest vertex")
    cost = CostDistance(location)  # considering moving this out of the function and into main
    min = float('inf')
    key = -1

    # cycle through vertices, finding the nearest vertex
    for v in location:
        diff = cost.dist2Vertex(v, req)
        if diff < min:
            print(v)
            min, key = diff, v

    return key


# Finite state machine that processes requests from the arduino client
def finite_state_machine(edmonton_graph, location):
    cost = CostDistance(location)

    class states(Enum):
        REQ = 0  # Waiting on request
        ACK = 1  # Waiting on acknowledgement
        END = 2  # Send end key

    state = states.REQ
    path = []

    with Serial("/dev/ttyACM0", baudrate=9600, timeout=1) as ser:
        while True:
            print("Attempting to read from serial.")
            response = ser.readline().decode("ASCII").rstrip("\r\n").split(' ')
            print("read from serial:", response)

            if state == states.REQ and response[0] == 'R':
                # Read coordinates of the start and destination
                start = (int(response[1]), int(response[2]))
                end = (int(response[3]), int(response[4]))
                print(start, end)
                # Find nearest vertices for the start and end points
                # startKey, endKey = nearestVertex(start, location), nearestVertex(end, location)
                
                minStart, minEnd = float('inf'), float('inf')
                startKey, endKey = -1, -1

                # Find the nearest vertex to the start and destination request
                for v in location:
                    diffStart, diffEnd = cost.dist2Vertex(v, start), cost.dist2Vertex(v, end)
                    if diffStart < minStart:
                        minStart, startKey = diffStart, v
                    if diffEnd < minEnd:
                        minEnd, endKey = diffEnd, v

                print(startKey, endKey)
                # Find the shortest path
                print("Generating path.")
                path = least_cost_path(edmonton_graph, startKey, endKey, cost).reverse()
                print("Path generation complete")
                print(path)
                # Create formatted output string for the client
                length = 'N ' + ' ' + str(len(path)) + '\n'
                ser.write(length)
                print("Writing number of waypoints to serial.")

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
                print("Sending waypoint to client.")

            if state == states.END:
                ser.write('E\n')
                state = states.REQ


if __name__ == "__main__":
    edmonton_graph, location = load_edmonton_graph("edmonton-roads-2.0.1.txt")
    finite_state_machine(edmonton_graph, location)
