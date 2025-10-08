# pip install pathfinding
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