import random
import networkx as nx
from parse import read_input_file, write_output_file, read_output_file
from utils import is_valid_solution, calculate_score, node_diff, edge_diff
import sys
from os.path import basename, normpath
import glob




def solve(G):
    """
    Args:
        G: networkx.Graph
    Returns:
        c: list of cities to remove
        k: list of edges to remove
    """
    return simulatedAnnealing(0.5, G)


def simulatedAnnealing(initialTreshold, G):
    G = G.copy()
    endNode = len(G.nodes) - 1
    deletedEdges, deletedNodes = [], []

    def naiveDistribution(length):
        return [1/length for i in range(length)]

    def nodeRemover(threshold, nodesRemoved):
        if nodesRemoved >= MAX_NODES_REMOVED or threshold >= 1:
            return
        allnodes = nx.dijkstra_path(G, 0, endNode)
        improvementScore = []
        noPossibleUpdates = True
        for i in allnodes:
            if i == 0 or i == endNode:
                improvementScore.append(0)
                continue
            score = node_diff(G, i, endNode)
            if score:
                noPossibleUpdates = False
            improvementScore.append(score if score else 0)
        if noPossibleUpdates:
            return 
        chosenNode = random.choices(allnodes, weights=improvementScore, k = 1)
        deletedNodes.append(chosenNode[0])
        G.remove_node(chosenNode[0])
        return nodeRemover(threshold + 0.001, nodesRemoved + 1)
    
    def edgeRemover(threshold, edgesRemoved):
        if edgesRemoved >= MAX_EDGES_REMOVED or threshold >= 1:
            return
        previous = 0
        edges, distribution, improvementScore = [], [], []
        noPossibleUpdates = True
        for i in nx.dijkstra_path(G, 0, endNode)[1:]:              
            edges.append((previous, i))
            #distribution.append(G[previous][i]["weight"])
            score = edge_diff(G, (previous, i), endNode)
            if score:
                noPossibleUpdates = False
            improvementScore.append(score if score else 0)
            previous = i 
        if noPossibleUpdates:
            return 
        chosenEdge = random.choices(edges, weights=improvementScore, k = 1)
        deletedEdges.append(chosenEdge[0])
        G.remove_edge(chosenEdge[0][0],chosenEdge[0][1])
        return edgeRemover(threshold + 0.001, edgesRemoved + 1)

    
    nodeRemover(initialTreshold, 0)
    edgeRemover(initialTreshold, 0)
    
    return deletedNodes, deletedEdges

# Here's an example of how to run your solver.

# Usage: python3 solver.py test.in

"""
if __name__ == '__main__':
     assert len(sys.argv) == 2
     path = sys.argv[1]
     G = read_input_file(path)
     resultc, resultk, largest = None, None, 0
     for i in range(20):
        c, k = solve(G)
        if not is_valid_solution(G, c, k):
            continue
        currentScore = calculate_score(G, c, k)
        if largest < currentScore:
            resultc, resultk = c, k
            largest = currentScore
     print("Shortest Path Difference: {}".format(largest))
     write_output_file(G, c, k, 'outputs/small-1.out')
     print("Reference Shortest Score: " + str(calculate_score(G, [1], [(0, 27), (24, 29), (26, 29), (20, 16)])))
"""

# For testing a folder of inputs to create a folder of outputs, you can use glob (need to import it)
if __name__ == '__main__':
    for i in range(5):
        inputs = glob.glob('inputs/small/*')
        count = 1
        for input_path in inputs:
            output_path = 'outputs/small/' + basename(normpath(input_path))[:-3] + '.out'
            G = read_input_file(input_path)
            num_nodes = G.number_of_nodes()
            num_edges = G.number_of_edges()
            if 20 <= num_nodes <= 30:
                MAX_EDGES_REMOVED = 15
                MAX_NODES_REMOVED = 1
            elif 31 <= num_nodes <= 50:
                MAX_EDGES_REMOVED = 50
                MAX_NODES_REMOVED = 3
            elif 51 <= num_nodes <= 100:
                MAX_EDGES_REMOVED = 100
                MAX_NODES_REMOVED = 5
            resultc, resultk, largest = None, None, 0
            for i in range(20):
                c, k = solve(G)
                if not is_valid_solution(G, c, k):
                    continue
                currentScore = calculate_score(G, c, k)
                if largest < currentScore:
                    resultc, resultk = c, k
                    largest = currentScore

            existingSol = read_output_file(G, output_path)
            if largest > existingSol:
                write_output_file(G, resultc, resultk, output_path)
                print("enhanced by: " + str((largest - existingSol)/existingSol))
            print(str(count) + " out of " +str(len(inputs)) + " Done.")
            count += 1
        
