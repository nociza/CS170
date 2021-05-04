import random
import networkx as nx
from itertools import islice
from parse import read_input_file, write_output_file, read_output_file
from utils import is_valid_solution, calculate_score, node_diff, edge_diff
import sys
from os.path import basename, normpath
import glob

# def edges_from_path(path):
#     q = []
#     ret = []
#     for i in path:
#         q.append(i)
#         if len(q) == 2:
#             ret.append(tuple(q))
#             q.pop(0)
#     return ret

def k_shortest_paths(G, source, target, k, weight=None):
    return list(islice(nx.shortest_simple_paths(G, source, target, weight=weight), k))

def disconnect(G, v, target):
    new_G = G.copy()
    new_G.remove_node(v)
    return not nx.has_path(new_G,0,target) or not nx.is_connected(new_G)


def remove_k(G, k, n, target):
    new_G = G.copy()
    source = 0
    curr_k_shortest = k_shortest_paths(new_G, source, target, n, "weight")
    shortest = curr_k_shortest[0]
    if k == 0 or len(shortest) == 0:
        return []
    else:
        ret = []
        for i in shortest:
            if i == source or i == target:
                continue
            if disconnect(new_G, i, target):
                continue
            counter = 0
            for j in curr_k_shortest:
                if i in j:
                    counter += 1
                else:
                    break
            counter2 = 0
            for j in curr_k_shortest:
                if i in j:
                    counter2 += 1
            ret.append((i, counter, counter2))
        if len(ret) == 0:
            return []
        min1 = max(ret, key=lambda x: x[1])[1]
        ret = [a for a in ret if a[1] == min1]
        min2 = max(ret, key=lambda x: x[2])
        new_G.remove_node(min2[0])
        return [min2[0]] + remove_k(new_G, k - 1, n, target)









# def find_k_shortest(G, k):
#     new_G = G.copy()
#     source = 0
#     target = G.number_of_nodes() - 1
#     shortest_path = nx.dijkstra_path(new_G, 0, target)
#     paths = []
#     paths.append(shortest_path)
#     for j in range(k):
#         shortests = []
#         curr_edges = edges_from_path(shortest_path)
#         for i in curr_edges:
#             v_from = i[0]
#             v_to = i[1]
#             curr_G = new_G.copy()
#             curr_G.remove_edge(v_from, v_to)
#             curr_Dist = nx.dijkstra_path_length(curr_G, source, target)
#             shortests.append((i, curr_Dist))
#
#         shortest = min(shortests, key=lambda x: x[1])




def solve1(G):
    """
    Args:
        G: networkx.Graph
    Returns:
        c: list of cities to remove
        k: list of edges to remove
    """
    num_nodes = G.number_of_nodes()
    num_edges = G.number_of_edges()
    if 20 <= num_nodes <= 30:
        k_max = 15
        c_max = 1
    elif 31 <= num_nodes <= 50:
        k_max = 50
        c_max = 3
    elif 51 <= num_nodes <= 100:
        k_max = 100
        c_max = 5

    c = remove_k(G, c_max, min(20, num_nodes), num_nodes - 1)
    k = []
    new_G = G.copy()
    for i in c:
        new_G.remove_node(i)
    while k_max > 0:
        shortest_path = nx.dijkstra_path(new_G, 0, num_nodes - 1)
        edge_weights = [new_G[shortest_path[i]][shortest_path[i+1]]["weight"] for i in range(len(shortest_path) - 1)]

        removed = False
        while min(edge_weights) < 9999 and not removed:
            min_index = edge_weights.index(min(edge_weights))
            min_edge = (shortest_path[min_index], shortest_path[min_index + 1])
            H = new_G.copy()
            H.remove_edge(shortest_path[min_index], shortest_path[min_index + 1])
            if nx.is_connected(H):
                k.append(min_edge)
                new_G.remove_edge(shortest_path[min_index], shortest_path[min_index + 1])
                k_max -= 1
                removed = True
            else:
                edge_weights[min_index] = 9999
        if not removed:
            break
    return c, k



# Here's an example of how to run your solver.

# Usage: python3 solver.py test.in

# if __name__ == '__main__':
#
#     assert len(sys.argv) == 2
#     path = sys.argv[1]
#     G = read_input_file(path)
#     c, k = solve(G)
#     assert is_valid_solution(G, c, k)
#     print("Shortest Path Difference: {}".format(calculate_score(G, c, k)))
#     write_output_file(G, c, k, 'outputs/small/small-100.out')

# For testing a folder of inputs to create a folder of outputs, you can use glob (need to import it)

# For testing a folder of inputs to create a folder of outputs, you can use glob (need to import it)
if __name__ == '__main__':
    inputs = glob.glob('inputs/large/*')
    count = 1
    for input_path in inputs:
        output_path = 'outputs/large/' + basename(normpath(input_path))[:-3] + '.out'
        G = read_input_file(input_path)

        c, k = solve1(G)
        assert is_valid_solution(G, c, k)
        currentScore = calculate_score(G, c, k)

        existingSol = read_output_file(G, output_path)
        if currentScore > existingSol:
            write_output_file(G, c, k, output_path)
            print("enhanced by: " + str(currentScore - existingSol))
        print(str(count) + " out of " +str(len(inputs)) + " Done.")
        count += 1