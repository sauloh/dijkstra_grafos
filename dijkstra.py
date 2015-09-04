#!/usr/bin/python3
import sys
import time
from math import sqrt
from re import findall

# Retorna grafo da Matriz
def leMatriz(conteudo_arquivo, tipo, caminho=[], K=10):
    INF=99999
    g = Grafo(tipo) # Inicializa grafo tipo 1

    if tipo == 2: # Matriz do tipo 2 (coordenadas X Y)
        for v1,vertice1 in enumerate(conteudo_arquivo): # Para cada linha do arquivo. Cada linha eh um vertice
            if v1 == K:
                break
            if v1 in caminho:
                continue
            x1, y1 = vertice1.split() # Cordenadas do vertice 1
            for v2,vertice2 in enumerate(conteudo_arquivo): # Para cada linha do arquivo. Cada linha eh um vertice
                if v2 == K:
                    break
                if v2 in caminho:
                    continue
                x2, y2 = vertice2.split() # Cordenadas do vertice 2
                valorX = ( float(x1)-float(x2) )**2
                valorY = ( float(y1)-float(y2) )**2
                valor =  sqrt(valorX + valorY )
            
                g.add_aresta(v1, v2, valor)
            
    else:  # Matriz tipo 1 ou 3
        # Itera por cada linha do arquivo
        for l,linha in enumerate(conteudo_arquivo):
            if l == K: # Para quando alcancar tamanho maximo do grafo
                break
            if l in caminho: # Se linha ja foi usada (vertice no caminho)
                continue
            for c,valor in enumerate(linha.split()): # Itera por cada valor na linha
                if int(valor) == 0:
                    valor = INF

                if tipo == 1: # Se a matriz for do Tipo=1
                    if l+c == K:
                        break
                    if l+c in caminho: # Se coluna ja foi usada (vertice no caminho)
                        continue
                    if l > 0:
                        g.add_aresta(l, l+c, int(valor))
                    else:
                        g.add_aresta(l, c, int(valor))

                elif tipo == 3:
                    if c == K: # Para quando alcancar tamanho maximo do vertice
                        break
                    if c in caminho: # Se coluna ja foi usada (vertice no caminho)
                        continue
                    g.add_aresta(l, c, int(valor))
            
    return g # Retorna novo grafo

class Vertice:
    INF=99999

    def __init__(self, no):
        self.id = no # Identificador do no
        self.adjacente = {} # Inicializa dicionario de adjacencias como vazio
        self.distancia = self.INF # Inicializa distancia do vertice como infinito
        self.visitado = False # Marca como NaoVisitado
        self.anterior = None # No anterior

    # Retorna ID do no
    def get_id(self):
        return self.id

    # Adiciona no vizinho e distancia entre os nos
    def add_vizinho(self, vizinho, valor=0):
        self.adjacente[vizinho] = valor

    # Remove vizinho
    def remove_vizinhos(self, vertice):
        self.adjacente.pop(vertice, None)

    # Retorna lista de nos vizinhos
    def get_conexoes(self):
        return self.adjacente.keys()
    
    # Retorna valor entre um no e seu vizinhos v
    def get_valor(self, vizinho):
        return self.adjacente[vizinho]
        
    # Retorna distancia do No
    def get_dist(self):
        return self.distancia

    # Define distancia do no
    def set_dist(self, valor):
        self.distancia = valor
    
    # Define no anterior
    def set_anterior(self, ant): 
        self.anterior = ant
    
    # Marca no como visitado
    def set_visitado(self):
        self.visitado = True

    def __lt__(self, v):
        return self.distancia < v.distancia
        
    # Retorno legivel do tipo 'vetice'. Nome_do_Vertice Adjacentes: Lista_de_Vertices_Adjacentes
    def __str__(self):
        return str(self.id) + ' Adjacentes: ' + str([x.id for x in self.adjacente])

class Grafo:
    def __init__(self, tipo):
        self.dic_vertices = {} # Dicionario de Vertices
        self.num_vertices = 0 # Inicializa grafo com 0 vertices
    
    def __iter__(self):
        return iter(self.dic_vertices.values())
    
    # Adiciona vertice ao grafo
    def add_vertice(self, no):
        self.num_vertices = self.num_vertices + 1 # Incrementa contador de vertices do grafo
        novo_vertice = Vertice(no) # Cria novo vertice
        self.dic_vertices[no] = novo_vertice # Adiciona vertice ao Grafo
        return novo_vertice # Retorna novo vertice

    # Le vertice do Grafo
    def get_vertice(self, n):
        if n in self.dic_vertices: # Verifica se o vertice pertence ao grafo
            return self.dic_vertices[n] # Retorna vertice n
        else:
            return None # Nao exite vertice n

    # Adiciona aresta ao grafo
    def add_aresta(self, orig, dest, valor = 0):
        if orig not in self.dic_vertices:
            self.add_vertice(orig) # Adiciona vertice de origiem se ainda nao existe
        if dest not in self.dic_vertices:
            self.add_vertice(dest) # Adiciona vertice de destino se ainda nao existe
      
        # Se tipo == 1, adiciona a aresta de mesmo valor nos dois sentidos
        if tipo == 1: 
            self.dic_vertices[orig].add_vizinho(self.dic_vertices[dest], valor) # Adiciona aresta orig->dest
            self.dic_vertices[dest].add_vizinho(self.dic_vertices[orig], valor) # Adiciona aresta dest->orig
        # Se tipo == 2 ou tipo == 3, adiciona apenas um sentido da aresta 
        else:
            self.dic_vertices[orig].add_vizinho(self.dic_vertices[dest], valor) # Adiciona aresta orig->dest

    # Remove arestas de um vertice
    def remove_arestas(self, orig):
        for v in self.get_todos_vertices():
             self.dic_vertices[orig].remove_vizinhos(self.dic_vertices[v]) # Remove aresta orig->dest
             self.dic_vertices[v].remove_vizinhos(self.dic_vertices[orig]) # Remove aresta dest->orig
    
    # Retorna todos os nos que estao no grafo
    def get_todos_vertices(self):
        return self.dic_vertices.keys() # Retorna as chaves do dicionario(os nos)

    def set_anterior(self, atual):
        self.anterior = atual
   
    def get_anterior(self, atual):
        return self.anterior

    def matriz_adjacencia(self):
        vertices = self.dic_vertices
        
        for vkey, v in vertices.items():
            for v2key, v2 in vertices.items():
                try:
                    print("%4d(%d,%d)" % (v.adjacente[v2], vkey, v2key), end=" ")
                except:
                    print('X', end=" ")
            print("")

    # Retorna Conexoes do Grafo
    def retorna_conexoes(self):
        conexoes = []
        for v in g:
            for w in v.get_conexoes():
                vid = v.get_id()
                wid = w.get_id()
                conexoes.append( "%s, %s, %3d" % (vid, wid, v.get_valor(w)))
        return conexoes    

# Dijkstra
def dijkstra(grafo, inicio, fim):
    import heapq # Impor para trabalhar com filas

    inicio.set_dist(0) # Define distancia do no inicial como 0. Pois a distancia dele para ele mesmo eh 0
    nao_visitados = [(v.get_dist(), v) for v in grafo] # Gera lista de nos nao visitados
    #print(nao_visitados)
    heapq.heapify(nao_visitados) # Coloca a lista em forma de fila

    while len(nao_visitados): # Enquanto existem nos nao visitados 
        nv = heapq.heappop(nao_visitados) # Retira da lista de nao visitado vertice com menor distancia
        atual = nv[1] 
        atual.set_visitado()
         
        for proximo in atual.adjacente:
            if proximo.visitado: # Se no ja foi visitado, nao faz nada
                continue 
            nova_dist = atual.get_dist() + atual.get_valor(proximo) # No nao visitado, calcula nova distancia
            
            # Relaxa distancia
            if nova_dist < proximo.get_dist():
                proximo.set_dist(nova_dist)
                proximo.set_anterior(atual)
        
        # Atualiza fila
        while len(nao_visitados):
            heapq.heappop(nao_visitados)
        nao_visitados = [(v.get_dist(), v) for v in grafo if not v.visitado] # Gera lista de nos nao visitados
        heapq.heapify(nao_visitados) # Coloca a lista em forma de fila

def menor_caminho(v, caminho):
    if v.anterior:
        caminho.append(v.anterior.get_id())
        menor_caminho(v.anterior, caminho)
    return
        
    
#### Programa Principal ####
if __name__ == '__main__':

    if len(sys.argv) != 3: # Se numero de argumentos diferente de 2, mostra como usar
        print("Uso:\n\t{0} <caminho_do_arquivo> #K".format(sys.argv[0]))
        exit()

    arquivo = sys.argv[1] # Salva nome do arquivo a ser lido na variavel 'arquivo'
    K = int(sys.argv[2]) # Tamanho K de divisao do grafo

    with open(arquivo) as fp: # Le conteudo do arquivo e salva em 'conteudo_arquivo'
        conteudo_arquivo = fp.read().splitlines() 
    
    tipo = int(findall("""Tipo=(\d*) """, conteudo_arquivo[0])[0]) # Le o Tipo da matrix como inteiro
    N = int(findall("""N=(\d*) """, conteudo_arquivo[0])[0]) # Le numero de vertices na matriz
    conteudo_arquivo.pop(0) # Remove primeira linha do arquivo, pois contem dado que nao interessam mais.

    # Inicializa variaveis
    PRIM_ORIGEM = 0 # Ponto de Saida que se mantem constante
    ORIGEM = 0     # Origem. Ponto de Saida
    DESTINO = 0    # Destino, Ponto de Chegada
    TOTAL_CAM = [0]  # Tamanho total do caminho final
    TOTAL_VERT = 0 # Total de vertices no caminho final
    SOMA = 0 # Soma do caminho final
    J = 0 # Numero a mais

    startTime = time.clock()
    while(TOTAL_VERT <= N): # Executa algoritmo enquanto nao rodar Dijkstra para todos os vertices
        NVERTICES = 0  # Numero de vertices no caminho 
        TAMANHO = 9999 # Tamanho do caminho 
        VERTICE = -1   # Vertice inicial
        CAMINHO = []   # Caminho Final
        del TOTAL_CAM[-1]

        g = leMatriz(conteudo_arquivo, tipo, TOTAL_CAM, K+J) # Passa conteudo do arquivo e tipo da matriz e recebe como retorno o novo grafo
 
        dijkstra(g, g.get_vertice(ORIGEM), g.get_vertice(DESTINO)) # Executa Dijskstra

        for t in g.get_todos_vertices(): # Para cada vertice, encontra melhor caminho
            target = g.get_vertice(t)
            caminho = [t]
            menor_caminho(target, caminho)

            nvertices = len(caminho) # Calcula numero de vertices no caminho
            tamanho = g.get_vertice(t).distancia
            if nvertices > NVERTICES:    # Se o caminho for maior que o anterior, atualiza
                NVERTICES = nvertices
                TAMANHO = tamanho
                VERTICE = t
                CAMINHO = caminho[::-1]
            elif nvertices == NVERTICES: # Se contem o mesmo numero de vertices. Verifica o tamanho do caminho
                if tamanho < TAMANHO: # Se o caminho tem menor distancia
                    NVERTICES = nvertices
                    TAMANHO = tamanho
                    VERTICE = t
                    CAMINHO = caminho[::-1]

            #print('O menor caminho para %s: %s = %d' % (t, caminho[::-1], g.get_vertice(t).distancia))
            
        print('O maior caminho de menor tamanho e maior numero de vertices eh para %s, com %d vertices: %s = %d' % (VERTICE, NVERTICES, CAMINHO, TAMANHO))

        TOTAL_CAM.extend(CAMINHO)
        SOMA = SOMA + TAMANHO
        TOTAL_VERT = TOTAL_VERT + NVERTICES
        if TOTAL_VERT < N:
            TOTAL_VERT = TOTAL_VERT -1

        ORIGEM = CAMINHO[-1] # Origem agora sera o ultimo ponto onde o algoritmo parou
        DESTINO = ORIGEM # Escolhe qualquer vertice que ainda esta presente no grafo

        if (TOTAL_VERT == K-1+J): # Se alcancou o numero de vertices esperado
            J = J + K

    g = leMatriz(conteudo_arquivo, tipo, [], K=N) # Passa conteudo do arquivo e tipo da matriz e recebe como retorno o novo grafo
    ultimo_caminho = (g.get_vertice(TOTAL_CAM[-1]).get_valor(g.get_vertice(PRIM_ORIGEM)))
    SOMA = SOMA + ultimo_caminho
    TOTAL_CAM.append(PRIM_ORIGEM)
    
    endTime = time.clock()
    print("Caminho Total: {0} = {1}".format(TOTAL_CAM, SOMA))
    print("Tempo de Execucao {0}".format(endTime-startTime))

    # Descomente as 3 linhas seguintes para imprimir as conexoes do Grafo
    #conexoes = g.retorna_conexoes() 
    #for con in conexoes:
    #    print(con)

    # Descomente a linha seguinte para imprimir a matriz de adjacencia do grafo
    #g.matriz_adjacencia()
    
    #print("Menor Distancia entre {0} e {1} = {2}".format(ORIGEM, DESTINO, g.get_vertice(DESTINO).distancia)) # Imprime menor distancia da origem ao destino
