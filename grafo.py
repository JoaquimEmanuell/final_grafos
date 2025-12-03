class Grafo:
    def __init__(self, n_vertices, ponderado):
        self.n_vertices = n_vertices
        self.ponderado = ponderado

        self.adj = [[] for _ in range(n_vertices)]

    def n(self):
        return self.n_vertices
    
    def m(self):
        return sum(len(vizinhos) for vizinhos in self.adj) // 2
    
    def viz(self, v):
        return [vizinho for vizinho, peso in self.adj[v]]
    
    def d(self, v):
        return len(self.adj[v])
    
    def w(self, v1, v2):
        for vizinho, peso in self.adj[v1]:
            if vizinho == v2:
                return peso if self.ponderado else 1
            
        raise ValueError(f"Aresta ({v1}, {v2}) não existe.")