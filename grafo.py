from collections import deque

class Grafo:
    # Definição do construtor da classe
    def __init__(self, n_vertices, ponderado):
        self.n_vertices = n_vertices
        self.ponderado = ponderado
        self.grau = [0] * n_vertices
        self.adj = [[] for _ in range(n_vertices)]
        self._mind = 0
        self._maxd = 0

    # Função que retorna o número de vértices
    def n(self):
        return self.n_vertices
    
    # Função que retorna o número de arestas
    def m(self):
        return sum(len(vizinhos) for vizinhos in self.adj) // 2

    # Função que retorna os vizinhos de um vértice
    def viz(self, v):
        return [vizinho for vizinho, _ in self.adj[v]]
    
    # Função que retorna o grau de um vértice
    def d(self, v):
        return len(self.adj[v])
    
    # Função que retorna o peso da aresta entre dois vértices
    def w(self, v1, v2):
        for vizinho, peso in self.adj[v1]:
            if vizinho == v2:
                return peso if self.ponderado else 1
            
        raise ValueError(f"Aresta ({v1}, {v2}) não existe.")

    # Função para adicionar vértices (e funcionar com o mind e maxd)
    def add_vertice(self, u, v, peso=1):
        if u < 0 or u >= self.n_vertices or v < 0 or v >= self.n_vertices:
            raise ValueError("Vértice inválido")
        
        if self.ponderado:
            self.adj[u].append((v, peso))
            self.adj[v].append((u, peso))
        else:
            self.adj[u].append((v, 1))
            self.adj[v].append((u, 1))

        self.grau[u] += 1
        self.grau[v] += 1

        self._maxd = max(self._maxd, self.grau[u], self.grau[v])

    # funções de mínimo e máximo
    def mind(self):
        return self._mind

    def maxd(self):
        return self._maxd

    # bfs desconsiderando o ponderamento dos vértices
    def bfs(self, v):
        d = [float('inf')] * self.n_vertices
        pi = [None] * self.n_vertices
        visitado = [False] * self.n_vertices
        fila = deque()

        d[v] = 0
        visitado[v] = True
        fila.append(v)

        while fila:
            u = fila.popleft()

            for vizinho, _ in self.adj[u]:
                if not visitado[vizinho]:
                    visitado[vizinho] = True
                    d[vizinho] = d[u] + 1
                    pi[vizinho] = u
                    fila.append(vizinho)

        return d, pi

    # função de dfs :)
    def dfs(self, v):
        pi = [None] * self.n_vertices
        visitado = [False] * self.n_vertices
        v_ini = [0] * self.n_vertices
        v_fim = [0] * self.n_vertices
        tempo = 0

        def visita_dfs(u):
            nonlocal tempo
            visitado[u] = True

            tempo += 1
            v_ini[u] = tempo

            for vizinho, _ in self.adj[u]:
                if not visitado[vizinho]:
                    pi[vizinho] = u
                    visita_dfs(vizinho)

            tempo += 1
            v_fim[u] = tempo

        visita_dfs(v)
        return pi, v_ini, v_fim

    # Bellman-Ford: algoritmo para encontrar caminhos mínimos
    def bf(self, origem):
        dist = [float('inf')] * self.n_vertices
        pi = [None] * self.n_vertices
        dist[origem] = 0
        
        # Relaxa as arestas n-1 vezes
        for k in range(self.n_vertices - 1):
            for u in range(self.n_vertices):
                if dist[u] != float('inf'):
                    for vizinho, peso in self.adj[u]:
                        if dist[u] + peso < dist[vizinho]:
                            dist[vizinho] = dist[u] + peso
                            pi[vizinho] = u
        
        # Verifica ciclo negativo
        for u in range(self.n_vertices):
            if dist[u] != float('inf'):
                for vizinho, peso in self.adj[u]:
                    if dist[u] + peso < dist[vizinho]:
                        raise ValueError("Grafo contém ciclo de peso negativo")
        
        return dist, pi
    
    # Dijkstra: algoritmo mais eficiente para caminhos mínimos (não funciona com pesos negativos)
    def dijkstra(self, origem):
        import heapq
        
        dist = [float('inf')] * self.n_vertices
        pi = [None] * self.n_vertices
        dist[origem] = 0
        
        # Heap de prioridade com (distância, vértice)
        heap = [(0, origem)]
        visitados = set()
        
        while heap:
            dist_atual, u = heapq.heappop(heap)
            
            if u in visitados:
                continue
                
            visitados.add(u)
            
            # Relaxa as arestas adjacentes
            for vizinho, peso in self.adj[u]:
                if vizinho not in visitados:
                    nova_dist = dist[u] + peso
                    if nova_dist < dist[vizinho]:
                        dist[vizinho] = nova_dist
                        pi[vizinho] = u
                        heapq.heappush(heap, (dist[vizinho], vizinho))
        
        return dist, pi
    
    # Coloração própria usando algoritmo guloso (heurística Welsh-Powell)
    def coloracao_propria(self):
        cores = [0] * self.n_vertices
        
        # Ordena vértices por grau decrescente (heurística)
        vertices = list(range(self.n_vertices))
        vertices.sort(key=lambda v: self.d(v), reverse=True)
        
        for v in vertices:
            # Encontra cores usadas pelos vizinhos
            cores_usadas = set()
            for vizinho, _ in self.adj[v]:
                if cores[vizinho] != 0:
                    cores_usadas.add(cores[vizinho])
            
            # Atribui a menor cor disponível
            cor = 1
            while cor in cores_usadas:
                cor += 1
            cores[v] = cor
        
        # Número total de cores usadas
        num_cores = max(cores) if cores else 0
        return cores, num_cores
    
    # Método auxiliar para reconstruir caminho a partir dos predecessores
    def reconstruir_caminho(self, pi, origem, destino):
        if pi[destino] is None and destino != origem:
            return None  # Não há caminho
        
        caminho = []
        atual = destino
        while atual is not None:
            caminho.append(atual)
            atual = pi[atual]
        
        caminho.reverse()
        return caminho