matrix = [[0] * 5] * 2

for i in range(0,2):
    for j in range(0,5):
        matrix[i][j] = j + j * i
        print matrix[i][j],
    print "\n",
