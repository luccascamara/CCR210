from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from matplotlib import pyplot as plt
import numpy as np

pgmf = open('map.pgm', 'rb')
matrix = plt.imread(pgmf)
#print (matrix)

matrix = 1.0 * (matrix > 250)

x_inicio = 70
y_inicio = 30

x_final = 260
y_final = 200

plt.imshow(matrix, interpolation='nearest', cmap='gray', origin='upper')
plt.scatter([x_inicio], [y_inicio], c='g', s=20, label='Início')
plt.scatter([x_final], [y_final], c='b', s=20, label='Meta')
plt.legend()
plt.show()

fatc = 10
goal = 10
id_atual = 0

#grid = Grid(matrix=matrix)


#-------------------------------------------------------------------------------------------
# Definição das classes de nó e grafo

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

qinit = grid(x_inicio, y_inicio, id_atual, None)

def arvore():

    global matrix, id_atual

    Arvore = Graph()  
    Arvore.add_vertex(qinit)
    
    # Configuração inicial do plot
    plt.ion()  # Modo interativo
    fig, ax = plt.subplots()
    ax.imshow(matrix, interpolation='nearest', cmap='gray', origin='upper')
    scatter = ax.scatter([], [], c='r', s=5)
    ax.scatter([x_inicio], [y_inicio], c='g', s=20, label='Início')
    ax.scatter([x_final], [y_final], c='b', s=20, label='Meta')
    plt.draw()

    # Verifica se o ponto inicial e final estão livres
    
    while True:
        qrand = rand_sample()
        qnear = nearest_vertex(qrand, Arvore.vertices)
        qnew = new_sample(qnear, qrand, fatc)

        # 1) se o ponto final está em parede, descarta
        if not is_free_xy(qnew.x, qnew.y):
            continue

        # 2) verifica o segmento entre qnear e qnew
        adjusted = last_free_before_obstacle(qnear, qnew)
        if adjusted is None:
            # obstáculo imediatamente à frente de qnear -> descartado
            continue
        if (adjusted[0], adjusted[1]) != (qnew.x, qnew.y):
            qnew.x, qnew.y = adjusted

        id_atual += 1
        qnew.id = id_atual
        Arvore.add_vertex(qnew)
        Arvore.add_edge(qnear, qnew)

        # Adiciona apenas o novo ponto ao scatter
        novo_ponto = np.array([[qnew.x, qnew.y]])
        offsets = scatter.get_offsets()
        scatter.set_offsets(np.vstack([offsets, novo_ponto]))
        fig.canvas.draw_idle()
        plt.pause(0.001)  # Pequena pausa para atualização (mais confiável)

        if (np.hypot(qnew.x - x_final, qnew.y - y_final) < goal):
            print("Meta alcançada!")
            break

    return qnew

#-------------------------------------------------------------------------------------------
# Funções auxiliares

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
    
    # Não atribuir id aqui; id será atribuído quando o nó for aceito na árvore
    return grid(x_new, y_new, -1, qnear)



def is_free_xy(x, y):
    col = int(round(x))
    row = int(round(y))
    if row < 0 or row >= matrix.shape[0] or col < 0 or col >= matrix.shape[1]:
        return False
    return matrix[row, col] == 1.0


def last_free_before_obstacle(qnear, qnew):
    dx = qnew.x - qnear.x
    dy = qnew.y - qnear.y
    steps = int(max(int(round(abs(dx))), int(round(abs(dy))), 1))
    # percorre do qnear em direção a qnew por passos por pixel
    for i in range(1, steps + 1):
        t = i / float(steps)
        x = qnear.x + dx * t
        y = qnear.y + dy * t
        if not is_free_xy(x, y):
            # se o primeiro passo já é obstáculo, retorna None
            if i == 1:
                return None
            ultimo_t = (i - 1) / float(steps)
            return (qnear.x + dx * ultimo_t, qnear.y + dy * ultimo_t)
    # não encontrou obstáculo
    return (qnew.x, qnew.y)

#-------------------------------------------------------------------------------------------

objetivo = arvore()
path = []
chegada = grid(x_final, y_final, id_atual + 1, objetivo)
path.append(chegada)

while objetivo is not None:
    path.append(objetivo)
    objetivo = objetivo.pai

print("Caminho encontrado com", len(path), "pontos.")

# Plot final do caminho encontrado
fig, ax = plt.subplots()
ax.imshow(matrix, interpolation='nearest', cmap='gray', origin='upper')
# Marcar início e fim
ax.scatter([x_inicio], [y_inicio], c='g', s=20, label='Início')
ax.scatter([x_final], [y_final], c='b', s=20, label='Meta')

# Plotar todos os pontos do caminho
path_x = [cell.x for cell in path]
path_y = [cell.y for cell in path]
ax.plot(path_x, path_y, 'r-', linewidth=2, label='Caminho')
ax.scatter(path_x, path_y, c='r', s=5)

plt.legend()
plt.title('Caminho RRT')

# Adiciona o texto centralizado na parte inferior do gráfico
ax.text(
    0.5, 0.07, 
    f'Caminho encontrado com {len(path)} pontos.', 
    transform=ax.transAxes, 
    fontsize=11, 
    color='red', 
    horizontalalignment='center',
    verticalalignment='bottom'
)

plt.savefig('rrt_resultado.png')
plt.ioff()  # Desativa modo interativo
plt.show()
