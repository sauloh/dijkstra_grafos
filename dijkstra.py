#!/usr/bin/python3
import sys
from re import findall

# Retorna grafo da Matriz Tipo=1
def leTipo1(conteudo_arquivo):
    # Inicializa Grafo
    g = Grafo(1) # Passa argumento tipo 1
    
    # Itera por cada linha do arquivo
    for l,linha in enumerate(conteudo_arquivo):
        # Itera por cada valor na linha
        for c,valor in enumerate(linha.split()): 
            if l > 0:
                g.add_aresta(l, l+c, int(valor))
            else:
                g.add_aresta(l, c, int(valor))
            
    return g # Retorna novo grafo
    

class Vertice:
    INF=99999

    def __init__(self, no):
        self.id = no # Identificador do no
        self.adjacente = {} # Inicializa dicionario de adjacencias como vazio
        self.distancia = self.INF # Inicializa distancia do vertice como infinito

    def add_vizinho(self, vizinho, valor=0):
        self.adjacente[vizinho] = valor

    def get_conexoes(self):
        return self.adjacente.keys()

    def get_id(self):
        return self.id
    
    def get_valor(self, vizinho):
        return self.adjacente[vizinho]
        
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
    
    # Retorna todos os nos que estao no grafo
    def get_todos_vertices(self):
        return self.dic_vertices.keys() # Retorna as chaves do dicionario(os nos)

    def matriz_adjacencia(self):
        vertices = self.dic_vertices
        
        for vkey, v in vertices.items():
            for v2key, v2 in vertices.items():
                try:
                    print("%4d" % (v.adjacente[v2]), end=" ")
                except:
                    print('X', end=" ")
            print("")

    # Imprime Conexoes do Grafo
    def imprime_conexoes(self):
        for v in g:
            for w in v.get_conexoes():
                vid = v.get_id()
                wid = w.get_id()
                print( "%s, %s, %3d" % (vid, wid, v.get_valor(w)))

#### Programa Principal ####

# Verifica argumentos.
# Se numero de argumentos menor que 1, mostra como usar
if len(sys.argv) != 2:
    print("Uso:\n\t{0} <caminho_do_arquivo>".format(sys.argv[0]))
    exit()

# Salva nome do arquivo a ser lido na variavel 'arquivo'
arquivo = sys.argv[1]

# Le conteudo do arquivo e salva em 'conteudo_arquivo'
with open(arquivo) as fp:
    conteudo_arquivo = fp.read().splitlines() 

# Le o Tipo da matrix como inteiro
tipo = int(findall("""Tipo=(\d*) """, conteudo_arquivo[0])[0])

# Remove primeira linha do arquivo, pois contem dado que nao interessam mais.
conteudo_arquivo.pop(0)

# Se tipo da matriz for 1, chama metodo para ler tipo1
if tipo == 1:
    g = leTipo1(conteudo_arquivo) # Passa conteudo do arquivo e tamanho da matriz e recebe como retorno o novo grafo

    # Descomente a linha seguinte para imprimir as conexoes do Grafo
    #g.imprime_conexoes() 

    # Descomente a linha seguinte para imprimir a matriz de adjacencia do grafo
    #g.matriz_adjacencia()
    

