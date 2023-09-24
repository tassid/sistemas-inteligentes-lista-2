import heapq
import networkx as nx
import matplotlib.pyplot as plt

# Definir o grafo com os custos das arestas em ordem alfabética
graph = {
    'A': {'B': 8, 'C': 14},
    'B': {'D': 38},
    'C': {'B': 9, 'D': 24, 'E': 7},
    'D': {'G': 9},
    'E': {'D': 13, 'G': 29, 'F': 9},
    'F': {},
    'G': {}
}

# Função para encontrar o caminho mínimo usando o algoritmo A*
def a_star(graph, start, goal):
    # Inicializar listas de nós abertos e nós visitados
    open_list = [(0, start)]
    visited = set()

    # Dicionários para armazenar custos e pais dos nós
    g_costs = {node: float('inf') for node in graph}
    g_costs[start] = 0
    parents = {}

    while open_list:
        # Obter o nó com o menor custo estimado f
        f, current_node = heapq.heappop(open_list)

        # Verificar se chegamos ao objetivo
        if current_node == goal:
            path = []
            while current_node:
                path.append(current_node)
                current_node = parents.get(current_node)
            return path[::-1]

        # Marcar o nó como visitado
        visited.add(current_node)

        # Explorar os vizinhos do nó atual
        for neighbor, cost in graph[current_node].items():
            if neighbor not in visited:
                # Calcular o custo total até este vizinho
                tentative_g = g_costs[current_node] + cost

                # Se o custo total é menor do que o custo atual, atualize-o
                if tentative_g < g_costs[neighbor]:
                    parents[neighbor] = current_node
                    g_costs[neighbor] = tentative_g
                    f = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f, neighbor))

    # Se não houver caminho até o objetivo, retorna None
    return None

# Função heurística (neste caso, a distância heurística é zero)
def heuristic(node, goal):
    return 0

# Função para criar e salvar o grafo como imagem PNG
def plot_graph(graph):
    G = nx.DiGraph()

    for node, neighbors in graph.items():
        for neighbor, cost in neighbors.items():
            G.add_edge(node, neighbor, weight=cost)

    # Usar 'shell_layout' para organizar os nós em círculos concêntricos
    pos = nx.shell_layout(G)

    labels = nx.get_edge_attributes(G, 'weight')
    nx.draw(G, pos, with_labels=True, node_size=500, node_color='skyblue', font_size=10, font_color='black')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
    plt.savefig("graph.png", format="PNG")
    plt.show()

# Encontrar o caminho mínimo de A para G usando A*
start_node = 'A'
goal_node = 'G'
path = a_star(graph, start_node, goal_node)

if path:
    print(f'Caminho mínimo de {start_node} para {goal_node}: {" -> ".join(path)}')
else:
    print(f'Não foi possível encontrar um caminho de {start_node} para {goal_node}.')

# Gerar e salvar o grafo como imagem PNG com layout "shell_layout"
plot_graph(graph)
