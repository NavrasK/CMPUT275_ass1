import math  # For math.sqrt()
from graph import Graph
from binary_heap import BinaryHeap

# Should be done, I used my solution from exercise 2 and added the location
# dict that they asked for in the description
def load_edmonton_graph(filename):
    """
    Description:
        Loads the graph of Edmonton from the given file.
        Returns two items
        graph: the instance of the class Graph() corresponding to the
        directed graph from edmonton-roads-2.0.1.txt
        location: a dictionary mapping the identifier of a vertex to
        the pair (lat, lon) of geographic coordinates for that vertex.
        These should be integers measuring the lat/lon in 100000-ths
        of a degree.
        In particular, the return statement in your code should be
        return graph, location
        (or whatever name you use for the variables).
        Note: the vertex identifiers should be converted to integers
        before being added to the graph and the dictionary.
    """
    g = Graph()
    location = dict()

    # Open the data file
    graphFile = open(filename, 'r')
    lines = graphFile.readlines()

    for line in lines:
        # Split at each comma, data stored comma-separated
        line = line.split(',')

        # Depending on the first character, treat input as vertex or edge
        if line[0] == 'V':
            # Add the vertex to the graph, and store the coordinates of the vertex
            g.add_vertex(line[1])
            coords = (int(float(line[2]) * 100000), int(float(line[3]) * 100000))
            location[int(line[1])] = coords

        if line[0] == 'E':
            # Add both directions of each edge
            g.add_edge((line[1], line[2]))

    # Don't forget to close the file!
    graphFile.close()

    return g, location


def get_path(reached, start, end):
  """
  Return a path from start to end, given a search tree.

  reached:
    A dictionary representing a search tree of a search
    initiated from the vertex "start".
  start:
    The vertex that was the start of the search that constructed
    the search tree
  end:
    The desired endpoint of the search

  Returns a list of vertices starting at vertex start and ending at vertex end
  representing a path between these vertices (the path in the search tree).
  If the vertex "end" was not reached (i.e. is not a key in reached),
  this simply returns the empty list []

  # the example in the docstring test is the search tree run on the graph
  # drawn using graphviz above, starting from vertex 3

  >>> reached = {3:3, 1:3, 4:3, 2:4}
  >>> get_path(reached, 3, 2)
  [3, 4, 2]
  >>> get_path(reached, 3, 3)
  [3]
  >>> get_path(reached, 3, 5)
  []
  """

  if end not in reached:
    return []

  path = [end]

  while end != start:
    end = reached[end]
    path.append(end)

  path.reverse()

  return path


# Should also be done, haven't tested
class CostDistance:
    """
    A class with a method called distance that will return the Euclidean between two given vertices.
    """

    def __init__(self, location):
        """
        Creates an instance of the CostDistance class and stores the dictionary "location"
        as a member of this class.
        """
        self.locations = location

    def distance(self, e):
        """
        Here e is a pair (u,v) of vertices.
        Returns the Euclidean distance between the two vertices u and v.
        """
        start, end = self.locations[e[0]], self.locations[e[1]]
        xdiff, ydiff = ((start[0] - end[0])**2), ((start[1] - end[1])**2)
        return math.sqrt(xdiff + ydiff)

    def anyDist(self, v, r):
        start, end = r, self.locations[v]
        xdiff, ydiff = ((start[0] - end[0])**2), ((start[1] - end[1])**2
        return math.sqrt(xdiff + ydiff)


def least_cost_path(graph, start, dest, cost):
    """
    Find and return a least cost path in graph from start vertex to dest vertex.
    Details:
        Efficiency: If E is the number of edges, the run-time is O( E log(E) ).
        Args:
            graph (Graph): The digraph defining the edges between the vertices.
            start: The vertex where the path starts. It is assumed that start is a vertex of graph.
            dest: The vertex where the path ends. It is assumed that dest is a vertex of graph.
            cost: A class with a method called "distance" that takes as input an edge (a pair of vertices)
            and returns the cost of the edge. For more details, see the CostDistance class description below.
        Returns:
            list: A potentially empty list (if no path can be found) of the vertices in the graph. If there was a path, the first
                    vertex is always start, the last is always dest in the list.
                    Any two consecutive vertices correspond to some edge in graph.
    """
    events = BinaryHeap()
    reached = dict()
    events.insert((start,start), 0)

    while(events):
        edge, time = events.popmin()
        if edge[1] not in reached:
            reached[edge[1]] = edge[0]
            for w in graph.neighbours(edge[1]):
                fuse = cost.distance((edge[1], w))
                events.insert(((edge[1]), w), (time + fuse))

    return get_path(reached, start, dest)


def checkRcpt():
    rcpt = input()
    if rcpt != "A":
        raise InputError('Invalid Receipt')

    return True

if __name__ == "__main__":
    edmonton_graph, location = load_edmonton_graph("edmonton-roads-2.0.1.txt")
    request = input()
    request = request.split(' ')
    cost = CostDistance(location)

    if request[0] != "R":
        raise InputError('Invalid request format.')
    start = (request[1], request[2])
    end = (request[3], request[4])
    minStart, minEnd = float('inf'), float('inf')
    startKey, endKey = -1, -1

    for v in locations:
        diffStart, diffEnd= cost.anyDist(v, start), cost.anyDist(v, end)
        if diffStart < minStart:
            minStart, startKey = diffStart, v
        if diffEnd < minEnd:
            minEnd, endKey = diffEnd, v

    path = least_cost_path(edmonton_graph, startKey, endKey, cost)

    print('N', len(path))
    path = path.reverse()
    while path:
        if !checkRcpt():
            break
        waypoint = path.pop()
        print('W', location[waypoint][0], location[waypoint][1])

    print('E')
