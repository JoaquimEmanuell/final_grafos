from collections import deque
import heapq


class Digrafo:
    """
    Representa um dígrafo (grafo direcionado) usando lista de adjacência.
    
    Um dígrafo é composto por vértices e arcos direcionados.
    Cada arco vai de um vértice de origem para um vértice de destino
    e pode ter um peso associado.
    """
    
    def __init__(self, numero_vertices, ponderado=True):
        """
        Cria um novo dígrafo vazio.
        
        Args:
            numero_vertices: Quantidade de vértices do dígrafo
            ponderado: Se True, os arcos têm pesos; se False, todos os pesos são 1
        """
        self.numero_vertices = numero_vertices
        self.eh_ponderado = ponderado
        
        # Armazena os vizinhos de saída de cada vértice (para onde o vértice aponta)
        self.vizinhos_saida = [[] for _ in range(numero_vertices)]
        
        # Armazena os vizinhos de entrada de cada vértice (quem aponta para o vértice)
        self.vizinhos_entrada = [[] for _ in range(numero_vertices)]
    
    @classmethod
    def ler_arquivo(cls, caminho_arquivo):
        """
        Carrega um dígrafo a partir de um arquivo no formato DIMACS.
        
        O formato DIMACS usa:
        - Linha com 'p': define o problema -> p sp <num_vertices> <num_arcos>
        - Linhas com 'a': definem arcos -> a <origem> <destino> <peso>
        
        Args:
            caminho_arquivo: Caminho do arquivo a ser lido
            
        Returns:
            Nova instância de Digrafo preenchida com os dados do arquivo
        """
        numero_vertices = 0
        lista_arcos = []
        
        with open(caminho_arquivo, 'r') as arquivo:
            for linha in arquivo:
                linha = linha.strip()
                
                # Ignora linhas vazias
                if not linha:
                    continue
                
                partes = linha.split()
                
                # Linha de definição do problema: p sp <vertices> <arcos>
                if partes[0] == 'p':
                    numero_vertices = int(partes[2])
                    numero_arcos = int(partes[3])
                
                # Linha de arco: a <origem> <destino> <peso>
                elif partes[0] == 'a':
                    origem = int(partes[1]) - 1      # Converte para índice base 0
                    destino = int(partes[2]) - 1
                    peso = int(partes[3])
                    lista_arcos.append((origem, destino, peso))
        
        # Cria o dígrafo e adiciona todos os arcos
        digrafo = cls(numero_vertices, ponderado=True)
        
        for origem, destino, peso in lista_arcos:
            digrafo.adicionar_arco(origem, destino, peso)
        
        print(f"Dígrafo carregado com sucesso: {numero_vertices} vértices e {len(lista_arcos)} arcos")
        return digrafo
    
    def adicionar_arco(self, origem, destino, peso=1):
        """
        Adiciona um arco direcionado do vértice origem para o vértice destino.
        
        Args:
            origem: Vértice de onde o arco sai
            destino: Vértice para onde o arco aponta
            peso: Peso (custo) do arco (padrão: 1)
            
        Raises:
            ValueError: Se os vértices forem inválidos (fora do intervalo válido)
        """
        # Valida se os vértices existem
        if origem < 0 or origem >= self.numero_vertices:
            raise ValueError(f"Vértice de origem {origem} é inválido")
        if destino < 0 or destino >= self.numero_vertices:
            raise ValueError(f"Vértice de destino {destino} é inválido")
        
        # Adiciona o arco nas duas representações
        self.vizinhos_saida[origem].append((destino, peso))
        self.vizinhos_entrada[destino].append((origem, peso))
    
    # ========== MÉTODOS BÁSICOS DE CONSULTA ==========
    
    def n(self):
        """Retorna o número total de vértices do dígrafo."""
        return self.numero_vertices
    
    def m(self):
        """Retorna o número total de arcos do dígrafo."""
        return sum(len(vizinhos) for vizinhos in self.vizinhos_saida)
    
    def viz(self, vertice):
        """
        Retorna todos os vizinhos do vértice (entrada + saída, sem repetição).
        
        No dígrafo, um vértice v é vizinho de u se existe arco u->v ou v->u.
        
        Args:
            vertice: Vértice a ser consultado
            
        Returns:
            Lista com todos os vértices adjacentes (sem repetição)
        """
        vizinhos_que_recebem = [u for u, _ in self.vizinhos_entrada[vertice]]
        vizinhos_que_enviam = [u for u, _ in self.vizinhos_saida[vertice]]
        
        # Remove duplicatas usando set e converte de volta para lista
        return list(set(vizinhos_que_recebem + vizinhos_que_enviam))
    
    def out_neighborhood(self, vertice):
        """
        Retorna os vizinhos de saída (para onde o vértice aponta).
        
        Args:
            vertice: Vértice a ser consultado
            
        Returns:
            Lista com vértices que recebem arcos do vértice
        """
        return [u for u, _ in self.vizinhos_saida[vertice]]
    
    def in_neighborhood(self, vertice):
        """
        Retorna os vizinhos de entrada (quem aponta para o vértice).
        
        Args:
            vertice: Vértice a ser consultado
            
        Returns:
            Lista com vértices que enviam arcos para o vértice
        """
        return [u for u, _ in self.vizinhos_entrada[vertice]]
    
    def d(self, vertice):
        """
        Retorna o grau total do vértice (soma do grau de entrada e saída).
        
        Args:
            vertice: Vértice a ser consultado
            
        Returns:
            Grau total = indegree + outdegree
        """
        return self.indegree(vertice) + self.outdegree(vertice)
    
    def indegree(self, vertice):
        """
        Retorna o grau de entrada (quantos arcos chegam no vértice).
        
        Args:
            vertice: Vértice a ser consultado
            
        Returns:
            Número de arcos que apontam para o vértice
        """
        return len(self.vizinhos_entrada[vertice])
    
    def outdegree(self, vertice):
        """
        Retorna o grau de saída (quantos arcos saem do vértice).
        
        Args:
            vertice: Vértice a ser consultado
            
        Returns:
            Número de arcos que saem do vértice
        """
        return len(self.vizinhos_saida[vertice])
    
    def w(self, origem, destino):
        """
        Retorna o peso do arco que vai de origem para destino.
        
        Args:
            origem: Vértice de onde sai o arco
            destino: Vértice para onde vai o arco
            
        Returns:
            Peso do arco (ou 1 se o dígrafo não é ponderado)
            
        Raises:
            ValueError: Se o arco não existe
        """
        # Procura o destino entre os vizinhos de saída da origem
        for vizinho, peso in self.vizinhos_saida[origem]:
            if vizinho == destino:
                return peso if self.eh_ponderado else 1
        
        # Se não encontrou, o arco não existe
        raise ValueError(f"Não existe arco de {origem} para {destino}")
    
    def mind(self):
        """
        Retorna o menor grau encontrado entre todos os vértices.
        
        Returns:
            Menor grau do dígrafo
        """
        return min(self.d(vertice) for vertice in range(self.numero_vertices))
    
    def maxd(self):
        """
        Retorna o maior grau encontrado entre todos os vértices.
        
        Returns:
            Maior grau do dígrafo
        """
        return max(self.d(vertice) for vertice in range(self.numero_vertices))
    
    # ========== ALGORITMOS DE BUSCA ==========
    
    def bfs(self, vertice_inicial):
        """
        Busca em Largura (Breadth-First Search) a partir de um vértice inicial.
        
        Explora o dígrafo camada por camada:
        1. Começa pelo vértice inicial
        2. Visita todos os vizinhos diretos
        3. Depois visita os vizinhos dos vizinhos, e assim por diante
        
        Args:
            vertice_inicial: Vértice de onde começa a busca
            
        Returns:
            Tupla (distancias, predecessores) onde:
            - distancias[v]: número de arcos do vertice_inicial até v (inf se inalcançável)
            - predecessores[v]: vértice que vem antes de v no caminho (None se v é raiz)
        """
        # Inicializa todas as distâncias como infinito (ainda não visitados)
        distancias = [float('inf')] * self.numero_vertices
        predecessores = [None] * self.numero_vertices
        
        # O vértice inicial tem distância 0 de si mesmo
        distancias[vertice_inicial] = 0
        
        # Fila de vértices a serem processados (FIFO: primeiro a entrar, primeiro a sair)
        fila = deque([vertice_inicial])
        
        # Processa vértices enquanto houver vértices na fila
        while fila:
            vertice_atual = fila.popleft()
            
            # Explora todos os vizinhos de saída do vértice atual
            for vizinho, _ in self.vizinhos_saida[vertice_atual]:
                # Se o vizinho ainda não foi visitado
                if distancias[vizinho] == float('inf'):
                    # A distância até o vizinho é distância atual + 1
                    distancias[vizinho] = distancias[vertice_atual] + 1
                    # O vértice atual é predecessor do vizinho
                    predecessores[vizinho] = vertice_atual
                    # Adiciona vizinho na fila para ser processado
                    fila.append(vizinho)
        
        return distancias, predecessores
    
    def dfs(self, vertice_inicial):
        """
        Busca em Profundidade (Depth-First Search) a partir de um vértice inicial.
        
        Explora o dígrafo indo o mais fundo possível antes de voltar:
        1. Começa pelo vértice inicial
        2. Vai para um vizinho e continua indo fundo
        3. Quando não há mais vizinhos não visitados, volta
        
        Args:
            vertice_inicial: Vértice de onde começa a busca
            
        Returns:
            Tupla (predecessores, tempos_descoberta, tempos_finalizacao) onde:
            - predecessores[v]: vértice que vem antes de v na árvore DFS
            - tempos_descoberta[v]: momento em que v foi descoberto
            - tempos_finalizacao[v]: momento em que v foi completamente explorado
        """
        # Inicializa estruturas
        predecessores = [None] * self.numero_vertices
        tempos_descoberta = [None] * self.numero_vertices
        tempos_finalizacao = [None] * self.numero_vertices
        
        # Contador de tempo (relógio global)
        tempo = [0]
        
        def visitar_vertice(vertice):
            """
            Visita recursivamente um vértice e seus descendentes.
            
            Args:
                vertice: Vértice a ser visitado
            """
            # Incrementa o tempo e marca momento de descoberta
            tempo[0] += 1
            tempos_descoberta[vertice] = tempo[0]
            
            # Explora todos os vizinhos de saída
            for vizinho, _ in self.vizinhos_saida[vertice]:
                # Se o vizinho ainda não foi descoberto
                if tempos_descoberta[vizinho] is None:
                    # Marca predecessor e visita recursivamente
                    predecessores[vizinho] = vertice
                    visitar_vertice(vizinho)
            
            # Incrementa o tempo e marca momento de finalização
            # (após ter visitado todos os descendentes)
            tempo[0] += 1
            tempos_finalizacao[vertice] = tempo[0]
        
        # Inicia a busca a partir do vértice inicial
        visitar_vertice(vertice_inicial)
        
        return predecessores, tempos_descoberta, tempos_finalizacao
    
    # ========== ALGORITMOS DE CAMINHO MÍNIMO ==========
    
    def bf(self, vertice_inicial):
        """
        Algoritmo de Bellman-Ford para encontrar caminhos mínimos.
        
        Calcula a distância mínima (considerando pesos) do vértice inicial
        para todos os outros vértices. Funciona mesmo com pesos negativos.
        
        Args:
            vertice_inicial: Vértice de onde começar os caminhos
            
        Returns:
            Tupla (distancias, predecessores) onde:
            - distancias[v]: menor distância (soma dos pesos) até v
            - predecessores[v]: vértice anterior no caminho mínimo
            
        Raises:
            ValueError: Se houver ciclo com peso negativo (problema sem solução)
        """
        # Inicializa todas as distâncias como infinito
        distancias = [float('inf')] * self.numero_vertices
        predecessores = [None] * self.numero_vertices
        distancias[vertice_inicial] = 0
        
        # Relaxa todos os arcos (n-1) vezes
        # (onde n é o número de vértices)
        for _ in range(self.numero_vertices - 1):
            for vertice in range(self.numero_vertices):
                # Só processa vértices alcançáveis
                if distancias[vertice] != float('inf'):
                    # Tenta melhorar distância dos vizinhos
                    for vizinho, peso in self.vizinhos_saida[vertice]:
                        nova_distancia = distancias[vertice] + peso
                        if nova_distancia < distancias[vizinho]:
                            distancias[vizinho] = nova_distancia
                            predecessores[vizinho] = vertice
        
        # Verifica se existe ciclo negativo
        # (se ainda dá para melhorar, há ciclo negativo)
        for vertice in range(self.numero_vertices):
            if distancias[vertice] != float('inf'):
                for vizinho, peso in self.vizinhos_saida[vertice]:
                    if distancias[vertice] + peso < distancias[vizinho]:
                        raise ValueError("Dígrafo contém ciclo de peso negativo")
        
        return distancias, predecessores
    
    def dijkstra(self, vertice_inicial):
        """
        Algoritmo de Dijkstra para encontrar caminhos mínimos.
        
        Mais rápido que Bellman-Ford, mas não funciona com pesos negativos.
        Usa fila de prioridade para sempre processar o vértice mais próximo.
        
        Args:
            vertice_inicial: Vértice de onde começar os caminhos
            
        Returns:
            Tupla (distancias, predecessores) onde:
            - distancias[v]: menor distância (soma dos pesos) até v
            - predecessores[v]: vértice anterior no caminho mínimo
        """
        # Inicializa estruturas
        distancias = [float('inf')] * self.numero_vertices
        predecessores = [None] * self.numero_vertices
        distancias[vertice_inicial] = 0
        
        # Fila de prioridade: sempre processa vértice com menor distância
        # Formato: (distancia, vertice)
        fila_prioridade = [(0, vertice_inicial)]
        vertices_processados = set()
        
        while fila_prioridade:
            # Pega vértice com menor distância
            distancia_atual, vertice_atual = heapq.heappop(fila_prioridade)
            
            # Se já foi processado, pula
            if vertice_atual in vertices_processados:
                continue
            
            # Marca como processado
            vertices_processados.add(vertice_atual)
            
            # Tenta melhorar distância dos vizinhos
            for vizinho, peso in self.vizinhos_saida[vertice_atual]:
                if vizinho not in vertices_processados:
                    nova_distancia = distancias[vertice_atual] + peso
                    
                    # Se encontrou caminho melhor
                    if nova_distancia < distancias[vizinho]:
                        distancias[vizinho] = nova_distancia
                        predecessores[vizinho] = vertice_atual
                        # Adiciona na fila para processar depois
                        heapq.heappush(fila_prioridade, (distancias[vizinho], vizinho))
        
        return distancias, predecessores
    
    # ========== MÉTODOS AUXILIARES ==========
    
    def coloracao_propria(self):
        """
        Colore os vértices do dígrafo de forma que vizinhos não tenham a mesma cor.
        
        Usa algoritmo guloso com heurística de Welsh-Powell:
        - Ordena vértices por grau (maiores graus primeiro)
        - Para cada vértice, atribui a menor cor disponível
        
        Returns:
            Tupla (cores, numero_cores) onde:
            - cores[v]: cor atribuída ao vértice v (número de 1 a k)
            - numero_cores: total de cores usadas
        """
        # Inicializa todos os vértices sem cor (cor 0 = sem cor)
        cores = [0] * self.numero_vertices
        
        # Ordena vértices por grau decrescente (heurística de Welsh-Powell)
        # Vértices com mais vizinhos são coloridos primeiro
        vertices_ordenados = list(range(self.numero_vertices))
        vertices_ordenados.sort(key=lambda v: self.d(v), reverse=True)
        
        # Colore cada vértice
        for vertice in vertices_ordenados:
            # Descobre quais cores os vizinhos já usaram
            cores_dos_vizinhos = set()
            
            # Verifica cores dos vizinhos de saída
            for vizinho, _ in self.vizinhos_saida[vertice]:
                if cores[vizinho] != 0:
                    cores_dos_vizinhos.add(cores[vizinho])
            
            # Verifica cores dos vizinhos de entrada
            for vizinho, _ in self.vizinhos_entrada[vertice]:
                if cores[vizinho] != 0:
                    cores_dos_vizinhos.add(cores[vizinho])
            
            # Encontra a menor cor que nenhum vizinho está usando
            cor_escolhida = 1
            while cor_escolhida in cores_dos_vizinhos:
                cor_escolhida += 1
            
            # Atribui a cor ao vértice
            cores[vertice] = cor_escolhida
        
        # Conta quantas cores foram usadas no total
        numero_cores = max(cores) if cores else 0
        return cores, numero_cores
    
    def reconstruir_caminho(self, predecessores, origem, destino):
        """
        Reconstrói o caminho de um vértice até outro usando a lista de predecessores.
        
        Args:
            predecessores: Lista onde predecessores[v] é o vértice antes de v
            origem: Vértice onde o caminho começa
            destino: Vértice onde o caminho termina
            
        Returns:
            Lista com a sequência de vértices do caminho, ou None se não houver caminho
        """
        # Se não há predecessor e o destino não é a origem, não há caminho
        if predecessores[destino] is None and destino != origem:
            return None
        
        # Reconstrói o caminho de trás para frente
        caminho = []
        vertice_atual = destino
        
        while vertice_atual is not None:
            caminho.append(vertice_atual)
            vertice_atual = predecessores[vertice_atual]
        
        # Inverte para ficar na ordem origem -> destino
        caminho.reverse()
        return caminho
    
    def encontrar_ciclo(self, minimo_arestas=1):
        """
        Procura um ciclo no dígrafo que tenha pelo menos um número mínimo de arestas.
        
        Usa DFS com coloração de vértices:
        - Branco (0): vértice ainda não visitado
        - Cinza (1): vértice sendo processado (na pilha de recursão)
        - Preto (2): vértice completamente processado
        
        Quando encontra arco para vértice cinza, há um ciclo!
        
        Args:
            minimo_arestas: Número mínimo de arestas que o ciclo deve ter
            
        Returns:
            Lista com os vértices do ciclo, ou None se não encontrar ciclo válido
        """
        # Cores: 0=branco (não visitado), 1=cinza (processando), 2=preto (finalizado)
        cores = [0] * self.numero_vertices
        predecessores = [None] * self.numero_vertices
        
        def visitar_buscando_ciclo(vertice):
            """
            Visita vértice procurando por ciclos.
            
            Args:
                vertice: Vértice a ser visitado
                
            Returns:
                Lista com ciclo se encontrar, None caso contrário
            """
            # Marca vértice como "em processamento"
            cores[vertice] = 1
            
            # Explora vizinhos de saída
            for vizinho, _ in self.vizinhos_saida[vertice]:
                
                # Se vizinho é branco (não visitado), visita recursivamente
                if cores[vizinho] == 0:
                    predecessores[vizinho] = vertice
                    resultado = visitar_buscando_ciclo(vizinho)
                    if resultado:
                        return resultado
                
                # Se vizinho é cinza, encontrou ciclo!
                # (arco para vértice que ainda está sendo processado)
                elif cores[vizinho] == 1:
                    # Reconstrói o ciclo
                    ciclo = [vizinho]
                    vertice_atual = vertice
                    
                    # Volta seguindo predecessores até chegar no início do ciclo
                    while vertice_atual != vizinho and vertice_atual is not None:
                        ciclo.append(vertice_atual)
                        vertice_atual = predecessores[vertice_atual]
                    
                    # Fecha o ciclo
                    ciclo.append(vizinho)
                    ciclo.reverse()
                    
                    # Verifica se o ciclo tem arestas suficientes
                    numero_arestas = len(ciclo) - 1
                    if numero_arestas >= minimo_arestas:
                        return ciclo
            
            # Marca vértice como completamente processado
            cores[vertice] = 2
            return None
        
        # Tenta começar DFS de cada vértice não visitado
        for vertice in range(self.numero_vertices):
            if cores[vertice] == 0:
                resultado = visitar_buscando_ciclo(vertice)
                if resultado:
                    return resultado
        
        # Não encontrou ciclo válido
        return None
