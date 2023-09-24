import networkx as nx
from collections import deque

# Crie um grafo direcionado
grafo = nx.DiGraph()

# Adicionar nós com suas heurísticas estimadas
grafo.add_node('A', heuristic=10)
grafo.add_node('B', heuristic=5)
grafo.add_node('C', heuristic=8)
grafo.add_node('D', heuristic=10)
grafo.add_node('E', heuristic=1)
grafo.add_node('F', heuristic=13)
grafo.add_node('G', heuristic=2)
grafo.add_node('H', heuristic=3)
grafo.add_node('I', heuristic=0)
grafo.add_node('J', heuristic=8)
grafo.add_node('K', heuristic=0)

# Adicionar arestas com os custos
grafo.add_edge('A', 'B', weight=5)
grafo.add_edge('A', 'C', weight=2)
grafo.add_edge('B', 'D', weight=9)
grafo.add_edge('B', 'E', weight=4)
grafo.add_edge('C', 'F', weight=9)
grafo.add_edge('C', 'G', weight=10)
grafo.add_edge('E', 'H', weight=7)
grafo.add_edge('E', 'I', weight=7)
grafo.add_edge('G', 'J', weight=4)
grafo.add_edge('G', 'K', weight=3)

# a) Algoritmo de Busca em Largura
bfs_tree = nx.bfs_tree(grafo, source='A')
bfs_path = list(nx.shortest_path(bfs_tree, source='A', target='K'))
print("Busca em Largura:", bfs_path)

# b) Algoritmo de Busca em Profundidade (implementação personalizada)
def depth_first_search(graph, start, goal):
    visited = set()
    stack = [(start, [start])]

    while stack:
        node, path = stack.pop()
        if node == goal:
            return path
        if node not in visited:
            visited.add(node)
            neighbors = list(graph.neighbors(node))  # Obter vizinhos em ordem alfabética
            neighbors.sort()
            for neighbor in neighbors:
                if neighbor not in visited:
                    stack.append((neighbor, path + [neighbor]))

dfs_path = depth_first_search(grafo, start='A', goal='K')
print("Busca em Profundidade:", dfs_path)

# c) Algoritmo de Busca Gulosa
def greedy_search(graph, start, goal):
    path = []
    current = start
    visited = set()

    while current != goal:
        visited.add(current)
        neighbors = [n for n in graph.neighbors(current) if n not in visited]
        if not neighbors:
            return None  # Não há caminho possível
        next_node = min(neighbors, key=lambda node: graph.nodes[node]['heuristic'])
        path.append(next_node)
        current = next_node
    return path

greedy_path = greedy_search(grafo, start='A', goal='K')
if greedy_path:
    print("Busca Gulosa:", greedy_path)
else:
    print("Busca Gulosa: Nenhum caminho encontrado")

# d) Algoritmo A*
def astar_search(graph, start, goal):
    open_set = set([start])
    came_from = {}
    g_score = {node: float('inf') for node in graph.nodes}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph.nodes}
    f_score[start] = graph.nodes[start]['heuristic']

    while open_set:
        current = min(open_set, key=lambda node: f_score[node])
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return list(reversed(path))

        open_set.remove(current)
        for neighbor in graph.neighbors(current):
            tentative_g_score = g_score[current] + graph[current][neighbor]['weight']
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + graph.nodes[neighbor]['heuristic']
                if neighbor not in open_set:
                    open_set.add(neighbor)

astar_path = astar_search(grafo, start='A', goal='K')
if astar_path:
    print("A*:", astar_path)
else:
    print("A*: Nenhum caminho encontrado")
