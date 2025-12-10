from digrafo import Digrafo

print("Carregando dígrafo...")
G = Digrafo.ler_arquivo('USA-road-d.NY.gr')

print("a) Menor grau:")
menor_grau = G.mind()
print(f" {menor_grau}")
print()

print("b) Maior grau:")
maior_grau = G.maxd()
print(f"{maior_grau}")
print()

print("c) Caminho com >= 10 arestas:")
d, pi = G.bfs(0)
vertice_distante = None
for v in range(G.n()):
    if d[v] >= 10 and d[v] != float('inf'):
        vertice_distante = v
        break

if vertice_distante:
    caminho = G.reconstruir_caminho(pi, 0, vertice_distante)
    num_arestas = len(caminho) - 1
    print(f"   Quantidade de arestas: {num_arestas}")
    print(f"   d (distância até vértice {vertice_distante + 1}): {d[vertice_distante]}")
    print(f"   pi (predecessor): {pi[vertice_distante] + 1 if pi[vertice_distante] is not None else None}")
    caminho_base1 = [v + 1 for v in caminho]
    if len(caminho_base1) <= 15:
        print(f"   Caminho: {' -> '.join(map(str, caminho_base1))}")
    else:
        print(f"   Caminho: {' -> '.join(map(str, caminho_base1[:5]))} -> ... -> {' -> '.join(map(str, caminho_base1[-5:]))}")
print()

print("d) Ciclo com >= 5 arestas:")
ciclo = G.encontrar_ciclo(minimo_arestas=5)
if ciclo:
    num_arestas = len(ciclo) - 1
    print(f" Quantidade de arestas: {num_arestas}")
    ciclo_base1 = [v + 1 for v in ciclo]
    if len(ciclo_base1) <= 15:
        print(f"   {' -> '.join(map(str, ciclo_base1))}")
    else:
        print(f"   {' -> '.join(map(str, ciclo_base1[:5]))} -> ... -> {' -> '.join(map(str, ciclo_base1[-5:]))}")
print()

print("e) Vértice mais distante do vértice 129:")
d, pi = G.dijkstra(128)
max_dist = 0
vertice_mais_distante = None
for v in range(G.n()):
    if d[v] != float('inf') and d[v] > max_dist:
        max_dist = d[v]
        vertice_mais_distante = v

if vertice_mais_distante:
    print(f"   Vértice: {vertice_mais_distante + 1}")
    print(f"   Distância: {max_dist}")
print()

print("f) Coloração própria:")
c, k = G.coloracao_propria()
print(f" Número de cores: {k}")
print()