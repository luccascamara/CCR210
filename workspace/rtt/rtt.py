from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from matplotlib import pyplot as plt
import numpy as np

pgmf = open('map.pgm', 'rb')
matrix = plt.imread(pgmf)
print (matrix)

matrix = 1.0 * (matrix > 250)
plt.imshow(matrix, interpolation='nearest', cmap='gray')
plt.show()

x_inicio = 50
y_inicio = 50

x_final = 100
y_final = 100

fatc = 0.5
goal = 0.2
id_atual = 0

#grid = Grid(matrix=matrix)


#-------------------------------------------------------------------------------------------

class grid:
        def __init__(self, x, y ,id=-1, pai=None):
            self.id = id
            self.pai = pai
            self.x = x
            self.y = y

class Graph:
    def __init__(self):
        self.vertices = []  # Lista de nós (grid.node)
        self.edges = []     # Lista de arestas (conexões entre nós)

    #Adiciona um nó (grid.node) à lista de vértices
    def add_vertex(self, node): 
        if node not in self.vertices:  # Evita duplicatas
            self.vertices.append(node)

    # Adiciona uma aresta (conexão) entre dois nós  
    def add_edge(self, node1, node2): 
        if node1 in self.vertices and node2 in self.vertices:
            self.edges.append((node1, node2))  # Adiciona a conexão

#-------------------------------------------------------------------------------------------

qinit = grid(x_inicio, y_inicio, id_atual,-1)

def arvore():
    global matrix

    Arvore = Graph()  
    Arvore.add_vertex(qinit)
    plt.imshow(matrix, interpolation='nearest', cmap='gray')

    while True:
        qrand = rand_sample()
        qnear = nearest_vertex(qrand, Arvore.vertices)
        qnew = new_sample(qnear, qrand, fatc)  
        Arvore.add_vertex(qnew)
        Arvore.add_edge(qnear, qnew)
        
        plt.scatter(x=qnew.x, y=qnew.y, c='r', s=5)
        plt.show()

        if (np.sqrt((qnew.x - x_final) ** 2 + (qnew.y - y_final) ** 2) < goal):
            print("Meta alcançada!")
            break   
    
    return qnew   


def rand_sample():
    x = np.random.randint(0, matrix.shape[1])
    y = np.random.randint(0, matrix.shape[0])
    return grid(x, y)


def nearest_vertex(qrand, vertices):
    min_dist = float('inf')
    nearest = None
    for vertex in vertices:
        dist = np.sqrt((vertex.x - qrand.x) ** 2 + (vertex.y - qrand.y) ** 2)
        if dist < min_dist:
            min_dist = dist
            nearest = vertex
    return nearest


def new_sample(qnear: grid, qrand: grid, f):

    # Calcula a distância euclidiana entre qnear e qrand
    dist = np.sqrt((qrand.x - qnear.x) ** 2 + (qrand.y - qnear.y) ** 2)
    
    # Se a distância for maior que o fator de crescimento, ajusta para f
    if dist > f:
        razao = f / dist  # Razão para reduzir a distância
        x_new = qnear.x + razao * (qrand.x - qnear.x)
        y_new = qnear.y + razao * (qrand.y - qnear.y)
    else:
        # Caso contrário, mantém qrand como o novo ponto
        x_new = qrand.x
        y_new = qrand.y
    
    global id_atual
    id_atual += 1
    return grid(x_new, y_new, id_atual, qnear.id)



#-------------------------------------------------------------------------------------------

objetivo = arvore()
path = []

while True:
    path.append(objetivo)
    pai = objetivo.pai
    objetivo = pai
    if pai is None:
        break

plt.imshow(matrix, interpolation='nearest', cmap='gray')
for cell in path:
    plt.scatter(x=cell.x, y=cell.y, c='r', s=5)

plt.show()
