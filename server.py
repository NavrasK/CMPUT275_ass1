import graph

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
            g.add_vertex(line[1])
            location[int(line[1])] = (line[2], line[3])

        if line[0] == 'E':
            # Add both directions of each edge
            g.add_edge((line[1], line[2]))

    # Don't forget to close the file!
    graphFile.close()

    return g, location



class CostDistance:
    """
    A class with a method called distance that will return the Euclidean
    between two given vertices.
    """
    def __init__(self, location):
        """
        Creates an instance of the CostDistance class and stores the 3 dictionary "location" as a member of this class.
        """
    def distance(self, e):
        """
        Here e is a pair (u,v) of vertices.
        Returns the Euclidean distance between the two vertices u and v.
        """


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
