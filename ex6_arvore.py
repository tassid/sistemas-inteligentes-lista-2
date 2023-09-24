import networkx as nx
import matplotlib.pyplot as plt
import pydot
from networkx.drawing.nx_pydot import graphviz_layout

# Criar um grafo direcionado
G = nx.DiGraph()

# Adicionar nós com suas heurísticas estimadas
G.add_node('A', heuristic=10)
G.add_node('B', heuristic=5)
G.add_node('C', heuristic=8)
G.add_node('D', heuristic=10)
G.add_node('E', heuristic=1)
G.add_node('F', heuristic=13)
G.add_node('G', heuristic=2)
G.add_node('H', heuristic=3)
G.add_node('I', heuristic=0)
G.add_node('J', heuristic=8)
G.add_node('K', heuristic=0)

# Marcar nós objetivos com círculo duplo (*)
G.nodes['I']['is_goal'] = True
G.nodes['K']['is_goal'] = True

# Adicionar arestas com os custos
G.add_edge('A', 'B', weight=5)
G.add_edge('A', 'C', weight=2)
G.add_edge('B', 'D', weight=9)
G.add_edge('B', 'E', weight=4)
G.add_edge('C', 'F', weight=9)
G.add_edge('C', 'G', weight=10)
G.add_edge('E', 'H', weight=7)
G.add_edge('E', 'I', weight=7)
G.add_edge('G', 'J', weight=4)
G.add_edge('G', 'K', weight=3)

# Criar uma árvore gráfica usando o layout do Graphviz
pos = graphviz_layout(G, prog='dot')

# Labels dos nós com as heurísticas estimadas
node_labels = {node: f'{node}\nHeurística: {data["heuristic"]}' for node, data in G.nodes(data=True)}

# Labels dos nós objetivos com círculo duplo (*)
goal_node_labels = {node: f'{node}*\nHeurística: {data["heuristic"]}' for node, data in G.nodes(data=True) if data.get('is_goal', False)}

# Plotar o grafo com os labels dos nós
plt.figure(figsize=(12, 8))
nx.draw(G, pos, labels=node_labels, with_labels=True, node_size=800, node_color='lightblue', font_size=10, font_weight='bold', arrows=True)
nx.draw_networkx_labels(G, pos, labels=goal_node_labels, font_size=10, font_weight='bold', font_color='red')
edge_labels = nx.get_edge_attributes(G, 'weight')
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
plt.title("Árvore de Busca")
plt.savefig("arvore.png", format="PNG")
plt.show()
