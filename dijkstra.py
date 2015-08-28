#!/usr/bin/python3
import sys
from re import findall

# Retorna grafo da Matriz Tipo=1
def leTipo1(conteudo_arquivo, N):
    # Inicializa Grafo
    g = Grafo(1) # Passa argumento tipo 1
    
    # Adiciona N vertices ao Grafo
    for vertice in range(0,N): 
        g.add_vertice(vertice)

    # Itera por cada linha do arquivo
    for l,linha in enumerate(conteudo_arquivo):
        # Itera por cada valor na linha
        for c,valor in enumerate(linha.split()): 
            print(c, valor)
        print('\n\n')

class Vertice:
    INF=99999

    def __init__(self, no):
        self.id = no # Identificador do no
        self.adjacente = {} # Inicializa dicionario de adjacencias como vazio
        self.distancia = self.INF # Inicializa distancia do vertice como infinito
        
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
        if dest not in serlf.dic_vertices:
            self.add_vertice(dest) # Adiciona vertice de destino se ainda nao existe
      
        # Se tipo == 1, adiciona a aresta de mesmo valor nos dois sentidos
        if tipo == 1: 
            self.dic_vertices[orig].add_vizinho(self.dic_vertices[dest], valor) # Adiciona aresta orig->dest
            self.dic_vertices[orig].add_vizinho(self.dic_vertices[dest], valor) # Adiciona aresta dest->orig
        # Se tipo == 2, adiciona apenas um sentido da aresta 
        elif tipo == 2:
            self.dic_vertices[orig].add_vizinho(self.dic_vertices[dest], valor) # Adiciona aresta orig->dest

    # Retorna todos os nos que estao no grafo
    def get_todos_vertices(self):
        return self.dic_vertices.keys() # Retorna as chaves do dicionario(os nos)


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
N = int(findall("""N=(\d*) """, conteudo_arquivo[0])[0])-1

# Remove primeira linha do arquivo, pois contem dado que nao interessam mais.
conteudo_arquivo.pop(0)

# Se tipo da matriz for 1, chama metodo para ler tipo1
if tipo == 1:
    leTipo1(conteudo_arquivo, N) # Passa conteudo do arquivo e tamanho da matriz
