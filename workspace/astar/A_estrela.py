from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from matplotlib import pyplot as plt

pgmf = open('map.pgm', 'rb')
matrix = plt.imread(pgmf)
print (matrix)

matrix = 1.0 * (matrix > 250)
plt.imshow(matrix, interpolation='nearest', cmap='gray')
plt.show()

x_inicio = 180
y_inicio = 50

x_final = 260
y_final = 200

grid = Grid(matrix=matrix)
start = grid.node(x_inicio, y_inicio)
end   = grid.node(x_final , y_final)

finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
path, runs = finder.find_path(start, end, grid)
print('operations:', runs, 'path length:', len(path))

plt.imshow(matrix, interpolation='nearest', cmap='gray')
for cell in path:
    plt.scatter(x=cell.x, y=cell.y, c='r', s=5)

plt.show()
