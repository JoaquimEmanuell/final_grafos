from collections import deque
import heapq

class Digrafo:
    """
    Classe que representa um dígrafo (grafo direcionado).
    Utiliza lista de adjacência para armazenar os arcos.
    """
    
    def __init__(self, n_vertices, ponderado=True):
        """
        Inicializa o dígrafo.
        
        Args:
            n_vertices: Número de vértices
            ponderado: Se True, considera pesos nas arestas
        """
        self.n_vertices = n_vertices
        self.ponderado = ponderado
        # Lista de adjacência para arcos de saída
        self.adj_saida = [[] for _ in range(n_vertices)]
        # Lista de adjacência para arcos de entrada
        self.adj_entrada = [[] for _ in range(n_vertices)]
    
    @classmethod
    def ler_arquivo(cls, caminho_arquivo):
        """
        Lê dígrafo de arquivo no formato DIMACS.
        
        Formato:
        - Linhas com 'p': p sp <vertices> <arcos>
        - Linhas com 'a': a <origem> <destino> <peso>
        
        Args:
            caminho_arquivo: Caminho para o arquivo
            
        Returns:
            Instância de Digrafo carregada
        """
        n_vertices = 0
        arcos = []
        
        with open(caminho_arquivo, 'r') as f:
            for linha in f:
                linha = linha.strip()
                if not linha:
                    continue
                
                partes = linha.split()
                
                if partes[0] == 'p':
                    n_vertices = int(partes[2])
                    n_arcos = int(partes[3])
                    
                elif partes[0] == 'a':
                    origem = int(partes[1]) - 1  # Converte para índice base 0
                    destino = int(partes[2]) - 1
                    peso = int(partes[3])
                    arcos.append((origem, destino, peso))
        
        dg = cls(n_vertices, ponderado=True)
        
        for origem, destino, peso in arcos:
            dg.adicionar_arco(origem, destino, peso)
        
        print(f"Dígrafo carregado: {n_vertices} vértices, {len(arcos)} arcos")
        return dg
    
    def adicionar_arco(self, origem, destino, peso=1):
        """
        Adiciona arco direcionado de origem para destino.
        
        Args:
            origem: Vértice de origem
            destino: Vértice de destino
            peso: Peso do arco (padrão 1)
        """
        if origem < 0 or origem >= self.n_vertices or destino < 0 or destino >= self.n_vertices:
            raise ValueError("Vértice inválido")
        
        self.adj_saida[origem].append((destino, peso))
        self.adj_entrada[destino].append((origem, peso))
    
    def n(self):
        """Retorna o número de vértices."""
        return self.n_vertices
    
    def m(self):
        """Retorna o número de arcos."""
        return sum(len(lista) for lista in self.adj_saida)
    
    def viz(self, v):
        """
        Retorna a vizinhança do vértice v.
        No dígrafo: viz(v) = in_neighborhood(v) + out_neighborhood(v)
        
        Args:
            v: Vértice
            
        Returns:
            Lista de vértices adjacentes (sem repetição)
        """
        viz_entrada = [u for u, _ in self.adj_entrada[v]]
        viz_saida = [u for u, _ in self.adj_saida[v]]
        return list(set(viz_entrada + viz_saida))
    
    def out_neighborhood(self, v):
        """Retorna vizinhos de saída do vértice v."""
        return [u for u, _ in self.adj_saida[v]]
    
    def in_neighborhood(self, v):
        """Retorna vizinhos de entrada do vértice v."""
        return [u for u, _ in self.adj_entrada[v]]
    
    def d(self, v):
        """
        Retorna o grau do vértice v.
        No dígrafo: d(v) = indegree(v) + outdegree(v)
        
        Args:
            v: Vértice
            
        Returns:
            Grau total do vértice
        """
        return self.indegree(v) + self.outdegree(v)
    
    def indegree(self, v):
        """Retorna o grau de entrada do vértice v."""
        return len(self.adj_entrada[v])
    
    def outdegree(self, v):
        """Retorna o grau de saída do vértice v."""
        return len(self.adj_saida[v])
    
    def w(self, origem, destino):
        """
        Retorna o peso do arco de origem para destino.
        
        Args:
            origem: Vértice de origem
            destino: Vértice de destino
            
        Returns:
            Peso do arco
            
        Raises:
            ValueError: Se o arco não existe
        """
        for viz, peso in self.adj_saida[origem]:
            if viz == destino:
                return peso if self.ponderado else 1
        
        raise ValueError(f"Arco ({origem}, {destino}) não existe.")
    
    def mind(self):
        """Retorna o menor grau presente no dígrafo."""
        return min(self.d(v) for v in range(self.n_vertices))
    
    def maxd(self):
        """Retorna o maior grau presente no dígrafo."""
        return max(self.d(v) for v in range(self.n_vertices))
    
    def bfs(self, v):
        """
        Executa busca em largura (BFS) a partir do vértice v.
        Segue apenas arcos de saída.
        
        Args:
            v: Vértice de origem
            
        Returns:
            Tupla (d, pi) onde:
            - d: Lista de distâncias de v até cada vértice
            - pi: Lista de predecessores no caminho de v até cada vértice
        """
        d = [float('inf')] * self.n_vertices
        pi = [None] * self.n_vertices
        d[v] = 0
        
        fila = deque([v])
        
        while fila:
            u = fila.popleft()
            
            for vizinho, _ in self.adj_saida[u]:
                if d[vizinho] == float('inf'):
                    d[vizinho] = d[u] + 1
                    pi[vizinho] = u
                    fila.append(vizinho)
        
        return d, pi
    
    def dfs(self, v):
        """
        Executa busca em profundidade (DFS) a partir do vértice v.
        Segue apenas arcos de saída.
        
        Args:
            v: Vértice de origem
            
        Returns:
            Tupla (pi, v_ini, v_fim) onde:
            - pi: Lista de predecessores na árvore de busca
            - v_ini: Lista de tempos de início da visita
            - v_fim: Lista de tempos de término da visita
        """
        pi = [None] * self.n_vertices
        v_ini = [None] * self.n_vertices
        v_fim = [None] * self.n_vertices
        tempo = [0]
        
        def visita_dfs(u):
            tempo[0] += 1
            v_ini[u] = tempo[0]
            
            for vizinho, _ in self.adj_saida[u]:
                if v_ini[vizinho] is None:
                    pi[vizinho] = u
                    visita_dfs(vizinho)
            
            tempo[0] += 1
            v_fim[u] = tempo[0]
        
        visita_dfs(v)
        return pi, v_ini, v_fim
    
    def bf(self, v):
        """
        Executa o algoritmo de Bellman-Ford a partir do vértice v.
        
        Args:
            v: Vértice de origem
            
        Returns:
            Tupla (d, pi) onde:
            - d: Lista de distâncias mínimas de v até cada vértice
            - pi: Lista de predecessores no caminho mínimo
            
        Raises:
            ValueError: Se o dígrafo contém ciclo de peso negativo
        """
        d = [float('inf')] * self.n_vertices
        pi = [None] * self.n_vertices
        d[v] = 0
        
        # Relaxa as arestas n-1 vezes
        for _ in range(self.n_vertices - 1):
            for u in range(self.n_vertices):
                if d[u] != float('inf'):
                    for vizinho, peso in self.adj_saida[u]:
                        if d[u] + peso < d[vizinho]:
                            d[vizinho] = d[u] + peso
                            pi[vizinho] = u
        
        # Verifica ciclo negativo
        for u in range(self.n_vertices):
            if d[u] != float('inf'):
                for vizinho, peso in self.adj_saida[u]:
                    if d[u] + peso < d[vizinho]:
                        raise ValueError("Dígrafo contém ciclo de peso negativo")
        
        return d, pi
    
    def dijkstra(self, v):
        """
        Executa o algoritmo de Dijkstra a partir do vértice v.
        Mais eficiente que Bellman-Ford, mas não funciona com pesos negativos.
        
        Args:
            v: Vértice de origem
            
        Returns:
            Tupla (d, pi) onde:
            - d: Lista de distâncias mínimas de v até cada vértice
            - pi: Lista de predecessores no caminho mínimo
        """
        d = [float('inf')] * self.n_vertices
        pi = [None] * self.n_vertices
        d[v] = 0
        
        # Heap de prioridade com (distância, vértice)
        heap = [(0, v)]
        visitados = set()
        
        while heap:
            dist_atual, u = heapq.heappop(heap)
            
            if u in visitados:
                continue
            
            visitados.add(u)
            
            # Relaxa as arestas adjacentes
            for vizinho, peso in self.adj_saida[u]:
                if vizinho not in visitados:
                    nova_dist = d[u] + peso
                    if nova_dist < d[vizinho]:
                        d[vizinho] = nova_dist
                        pi[vizinho] = u
                        heapq.heappush(heap, (d[vizinho], vizinho))
        
        return d, pi
    
    def coloracao_propria(self):
        """
        Executa coloração própria do dígrafo.
        Considera o grafo subjacente (ignora direção dos arcos).
        Usa algoritmo guloso com heurística Welsh-Powell.
        
        Returns:
            Tupla (c, k) onde:
            - c: Lista com cores atribuídas a cada vértice (inteiros de 1...k)
            - k: Número de cores utilizadas
        """
        cores = [0] * self.n_vertices
        
        # Ordena vértices por grau decrescente (heurística Welsh-Powell)
        vertices = list(range(self.n_vertices))
        vertices.sort(key=lambda v: self.d(v), reverse=True)
        
        for v in vertices:
            # Encontra cores usadas pelos vizinhos
            cores_usadas = set()
            
            # Considera vizinhos de saída
            for vizinho, _ in self.adj_saida[v]:
                if cores[vizinho] != 0:
                    cores_usadas.add(cores[vizinho])
            
            # Considera vizinhos de entrada
            for vizinho, _ in self.adj_entrada[v]:
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
    
    def reconstruir_caminho(self, pi, origem, destino):
        """
        Reconstrói o caminho de origem até destino usando predecessores.
        
        Args:
            pi: Lista de predecessores
            origem: Vértice de origem
            destino: Vértice de destino
            
        Returns:
            Lista com a sequência de vértices do caminho, ou None se não existe
        """
        if pi[destino] is None and destino != origem:
            return None
        
        caminho = []
        atual = destino
        while atual is not None:
            caminho.append(atual)
            atual = pi[atual]
        
        caminho.reverse()
        return caminho
    
    def encontrar_ciclo(self, min_arestas=1):
        """
        Procura um ciclo no dígrafo usando DFS.
        
        Args:
            min_arestas: Número mínimo de arestas que o ciclo deve ter
            
        Returns:
            Lista com a sequência de vértices do ciclo, ou None se não encontrar
        """
        # Cores: 0=branco (não visitado), 1=cinza (em processamento), 2=preto (processado)
        cores = [0] * self.n_vertices
        pi = [None] * self.n_vertices
        
        def visita_dfs(u):
            cores[u] = 1  # Marca como cinza
            
            for v, _ in self.adj_saida[u]:
                if cores[v] == 0:  # Branco - não visitado
                    pi[v] = u
                    resultado = visita_dfs(v)
                    if resultado:
                        return resultado
                elif cores[v] == 1:  # Cinza - aresta de retorno (ciclo!)
                    # Reconstrói o ciclo
                    ciclo = [v]
                    atual = u
                    while atual != v and atual is not None:
                        ciclo.append(atual)
                        atual = pi[atual]
                    ciclo.append(v)
                    ciclo.reverse()
                    
                    # Verifica tamanho mínimo
                    if len(ciclo) - 1 >= min_arestas:
                        return ciclo
            
            cores[u] = 2  # Marca como preto
            return None
        
        # Tenta de cada vértice não visitado
        for v in range(self.n_vertices):
            if cores[v] == 0:
                resultado = visita_dfs(v)
                if resultado:
                    return resultado
        
        return None
